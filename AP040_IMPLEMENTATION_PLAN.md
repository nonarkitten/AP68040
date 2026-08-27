# AP040 Implementation Plan

This repo holds two CPU cores, not one:

- **`rtl_old/`** -- the working, feature-complete sequential-FSM MC68040 core.
  Boots NetBSD/amiga and AmigaOS 3.x on real hardware, passes 3,776/3,801
  slices of the WinUAE `cputest` corpus (the rest are documented generator
  defects, not core bugs -- see `doc/CPUTEST_UPSTREAM_REPORT.md`). This is the
  **reference**: correct, but a multi-cycle-per-instruction FSM, not a
  pipeline. Its own test suite is `tb/run_tests.sh`.
- **`rtl/`** -- a from-scratch six-stage pipelined replacement
  (`ap040_pipe_core.v` and its stage files), built incrementally, one
  instruction/mechanism at a time, verified bit-for-bit against `rtl_old`'s
  decode/execute logic at every step. This is the active work. Its test suite
  is `tb/run_pipe_tests.sh`, and it needs nothing from `rtl_old/` or vice
  versa -- deleting either directory cannot affect the other (see
  `ap040_pipe_core.v`'s header).

The brute-force plan is exactly that: read what a stage does in `rtl_old`,
port its *intent* (not its FSM structure) into the matching pipeline stage,
add a testbench that would fail if the port were wrong, and move on. Sections
5-6 below are the actual TODO list; sections 1-4 are the parts of the
original architecture spec still worth keeping as reference.

## 1. Integration contract (unchanged from `rtl_old`)

Both cores present the same TG68K-shaped port set so either can drop into a
host that already speaks that interface (this is `rtl_old/ap040_tg68k_compat.v`
today; the pipeline will grow the same adapter once it's far enough along to
run real programs -- see section 6):

- clock/reset: `clk`, active-low `nreset`, `clkena_in` as the only advance
  enable
- external bus: `addr_out[31:0]`, `data_in[15:0]`, `data_write[15:0]`,
  `nWr/nUDS/nLDS`, `busstate[1:0]` (fetch/idle/read/write), `longword`, `FC[2:0]`
- MMU/cache sideband: logical/physical address, cache-inhibit, walker
  request/ack channel, cache req/ack/burst
- parameters: `AP040_HAS_MMU`, `AP040_HAS_FPU`, `AP040_ENABLE_CACHE`

`rtl_old`'s internal file split (one `ap040_core.v` FSM containing decode/EA/
exceptions/sequencing, plus standalone `ap040_alu.v`/`ap040_mmu.v`/
`ap040_cache.v`/`ap040_fpu.v`/`ap040_regfile.v`) is what actually got built and
proven, not the finer-grained module list an earlier draft of this plan
proposed -- if you're looking for `ap040_ea.v` or `ap040_sequencer.v`, they
don't exist; that logic lives inside `ap040_core.v`.

## 2. Non-goals (still true)

- cycle-accurate MC68040 external pin timing
- a physical 32-bit data bus exposed to a 16-bit host fabric
- multiprocessor bus-snoop intervention beyond what a specific host needs
- exact six-stage timing at the cycle level -- "architecturally-equivalent,
  not cycle-exact" (this is why the pipeline's branch/DBcc misprediction
  recovery, for example, doesn't replicate the real chip's shadow-register
  timing exactly -- see `ap040_pipe_core.v`'s milestone-4/5 notes)
- full hardware FPU before the pipeline reaches LC040-class MMU/cache parity
- replacing `rtl_old` before `rtl/` is independently proven

## 3. `rtl_old`: what's done, for reference

Feature-complete: full integer ISA, 68010/020/030/040 addressing modes,
68040 exception model (formats `$0/$1/$2/$3/$7`), 040 MMU (TTRs, 64-entry
4-way ATCs, three-level table walker, history bits), split I/D cache with
copyback, and the FPU (extended precision + FPSP trap path). Its own
regression suite (`tb/run_tests.sh`) covers all of this; treat any pipeline
milestone as unfinished until it matches `rtl_old`'s behavior for the same
instruction, not just "looks plausible."

`rtl_old`'s test suite needs `vasmm68k_mot` (vbcc) to assemble the `.s` test
programs in `tb/asm/`, in addition to `iverilog`. Neither is preinstalled on
a fresh machine; `iverilog` is a one-line `brew install icarus-verilog`,
`vasmm68k_mot` is not in Homebrew and needs building from the vbcc/vasm
sources separately. The non-assembly legs of that suite (reset, double
fault, walker CDC, bus16 gap, bus timeout, cache snoop) need only `iverilog`
and can be run standalone as a partial check.

## 4. `rtl/`: pipeline status

Six real stages (IF/ID/EA-calc/EA-fetch/EX/WB), synchronous stall/flush
chain, register forwarding (EX-forward mux + regfile write-through),
architectural CCR with its own write-through forward, and a "guess branches
taken, recover on misprediction" front end matching the real 68040's
documented policy (no BTB, always-assume-taken, one-cycle recovery --
see `ap040_pipe_core.v`'s milestone-1 note on the "shadow register" research).

Completed milestones (`tb/run_pipe_tests.sh`, one file per milestone):

| # | Adds | Test |
|---|------|------|
| 1 | Six-stage skeleton, NOP drains cleanly | `tb_ap040_pipe_nop.v` |
| 2 | MOVEQ, register-direct MOVE.L, register forwarding | `tb_ap040_pipe_moveq.v` |
| 3 | ADD.L Dn,Dm, dual-operand forwarding | `tb_ap040_pipe_add.v` |
| 4 | BRA.B, zero-bubble redirect | `tb_ap040_pipe_bra.v` |
| 5 | Bcc.B, misprediction recovery, CCR forwarding | `tb_ap040_pipe_bcc.v` |
| 6 | Scc.B Dn, folder independence (`rtl/` no longer touches `rtl_old/`) | `tb_ap040_pipe_scc.v` |
| 7 | Bcc.W/Bcc.L, multi-word decode gather, `id_next_pc` | `tb_ap040_pipe_bccw.v`, `tb_ap040_pipe_bccl.v` |
| 8 | DBcc Dn,\<label\>, dynamic (runtime-conditioned) register write | `tb_ap040_pipe_dbcc.v` |

Everything implemented so far is register-direct/immediate only -- **no
memory-referencing instruction exists yet**. That's the single biggest gap;
see section 6.

## 5. Verification discipline (keep doing this)

This is what has actually kept the pipeline correct across eight milestones
of rewrites; don't relax it for speed:

- **Bit-exact opcode/semantics verification against `rtl_old`.** Every
  decode pattern and execute-stage behavior added to `rtl/` cites the
  `rtl_old/ap040_core.v` line range it was checked against, not just "looks
  like the manual." When the two disagree, `rtl_old` wins (it's the
  cputest-validated one) unless there's a documented reason (see DBcc's
  deferred odd-target-parity check for the shape of that reason).
- **Mutation testing.** After a testbench passes, break the mechanism it's
  supposed to prove (force a mux to always miss, remove a term from a
  condition, etc.) and confirm the test actually fails. Several real test
  bugs (wrong immediates, poison instructions reachable via the CORRECT
  path because they sat next in memory, mutations that happen to converge
  on the same value as the correct path) were only caught this way -- see
  the milestone notes inside `ap040_decode.v`/`ap040_execute.v` and
  `tb_ap040_pipe_dbcc.v` for worked examples. A test that has never been
  seen to fail on broken RTL proves nothing.
- **Folder independence.** `rtl/ap040_pipe_*` never reaches into `rtl_old/`
  and vice versa (milestone 6). If a shared constant or helper looks
  tempting to factor out, don't -- fork it, the same way `ap040_pipe_alu.v`/
  `ap040_pipe_regfile.v`/`ap040_pipe_defs.svh` already do.
- **Scope discipline.** Defer, explicitly and in a comment, anything that
  needs a mechanism that doesn't exist yet (e.g. TRAPcc and DBcc's
  odd-target fault both need exception delivery -- neither is silently
  half-implemented to avoid admitting that).

## 6. Remaining scope, roughly in dependency order

1. **First memory-referencing instruction.** The real gap. `ap040_ea_fetch.v`
   currently only ever resolves register-direct/immediate operands (see its
   header). Needs: a memory port on the pipeline (start with a flat
   synchronous test memory, like `rtl_old`'s own testbenches use, before
   worrying about the real 16-bit adapter), an EA-calc stage that actually
   computes an address (`(An)` register-indirect is the natural first mode),
   and a stall path when the memory response isn't ready same-cycle. Suggest
   `MOVE.L (An),Dn` as the first target -- simplest addressing mode, reuses
   the existing MOVE.L decode/ALU path, only adds the fetch itself.
2. **BSR / JMP / JSR.** BSR needs the pipeline's first real memory *write*
   (stack push) and RTS needs a read; JMP/JSR need real effective-address
   modes, not just displacement math. Natural follow-on once (1) lands.
2b. **DBcc's deferred odd-target address-error** and **TRAPcc** both need
   the next item before they can be finished, not before they can be
   started -- see their notes in `ap040_execute.v`/`AP040_IMPLEMENTATION_PLAN.md`
   history for why a partial version was rejected rather than built.
3. **Illegal-instruction / exception delivery.** Vector fetch + supervisor
   stack frame push (format `$0`/`$1`/`$2` to start). This is the
   prerequisite for TRAPcc, the DBcc branch-target parity check, address
   error, and privilege violation -- all currently deferred specifically
   because there's nowhere to deliver the fault yet.
4. **Byte/word-sized ALU ops and MOVE** (current pipeline is Long-only
   throughout `ap040_pipe_alu.v`'s size port is already there and unused).
5. **MMU and cache integration**, once enough of the integer ISA exists that
   testing them against real address translation is meaningful. Reuse the
   architectural requirements from `rtl_old/ap040_mmu.v`/`ap040_cache.v`
   directly -- don't re-derive the 040 table-walk/ATC/TTR rules from the
   manual a second time.
6. **FPU integration**, gated on the above the same way `rtl_old` staged it
   (LC040 trap-only mode before any hardware arithmetic).
7. **Superscalar/dual-issue** is an explicit stretch goal, not a
   prerequisite for anything above -- 68060-class pairing rules, do it last,
   and only if single-issue CPI is still the bottleneck once the ISA is
   complete.

Performance-tuning work that depends on a specific FPGA target (clock
domain, ALM/DSP budget, Quartus fit) is out of scope for this repo entirely
-- it belongs in whichever downstream project integrates this core, the same
way `rtl_old`'s own Minimig-specific timing/fit history lived in that
project's tree, not here.

## 7. Test infrastructure

- `tb/run_pipe_tests.sh` -- the active suite. Needs only `iverilog`
  (`brew install icarus-verilog`; nothing else). Every `tb_ap040_pipe_*.v`
  bench pokes its program directly into `ap040_inst_fetch.v`'s ROM at time 0
  (`dut.u_if.rom[N] = ...`) -- no assembler, no host project.
- `tb/run_tests.sh` -- the `rtl_old` reference suite. Needs `iverilog` +
  `vasmm68k_mot`; see section 3 for the toolchain gap.
- Both suites are self-contained: no host-project sources, so a failure is
  the CPU's, not an integration artifact (a downstream project like the one
  this core is designed to be pulled into adds its own co-simulation
  benches against real chipset/memory-controller RTL; those don't belong
  here).

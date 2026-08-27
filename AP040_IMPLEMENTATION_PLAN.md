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
| 9a | Unified L1 (`ap040_pipe_l1.v`) replaces IF's inline ROM; port B reserved for data | all of the above, unchanged behavior |
| 9b | MOVE.L (An),Dn -- first memory read, first genuine pipeline stall | `tb_ap040_pipe_move_mem.v` |
| 10 | MOVE.L (d16,An),Dn -- first real EA displacement arithmetic; found and fixed a real L1/stall bug (see below) | `tb_ap040_pipe_move_disp.v` |

Register-direct/immediate/register-indirect(+displacement) only -- no
indexed/absolute/PC-relative EA modes, no byte/word sizes, no writes to
memory yet. See section 6.

## 5a. The unified-L1 decision (2026-08-27)

Went with a single dual-port memory shared between instruction fetch and
(eventually) data, instead of separate I/D storage or a private flat-memory
stub per testbench, on the reasoning below -- see `ap040_pipe_l1.v`'s header
for the implementation-level detail.

- **Coherency for free.** A write through the data port is visible to a
  later instruction-fetch read with no explicit invalidation, just from both
  ports addressing the same array -- unlike a split Harvard I/D cache (what
  `rtl_old/ap040_cache.v` and the real 68040 both are), which needs CINV/
  CPUSH to move data from the D side to the I side. This is a real,
  intentional deviation from the real chip's cache architecture, not an
  oversight: `rtl_old` already IS the split-cache reference if bit-exact
  cache behavior is ever needed, and this repo has no backing store of its
  own to make a real miss/fill/replacement policy meaningful yet (no SDRAM/
  DDR model -- unlike the Minimig-AGA tree this core was extracted from).
- **Consequence, not free:** CINV/CPUSH can only ever be decoded/accepted as
  no-ops on this substrate -- there is no staleness for them to fix, so
  their actual observable effect (stale-until-flush) is untestable here.
  `rtl_old`'s `t_cache` suite is where that semantic is actually exercised;
  don't expect the pipeline to ever reproduce it against this L1.
- **CACR's independent I/D enable bits** (the real 68040 lets software
  disable the data cache while leaving the instruction cache on, or vice
  versa) are not a blocker: they become an access-side gate ("does this
  read consult the array or bypass it") over the one shared array, not a
  reason to keep two cache instances. Not implemented yet -- there's no
  CACR in the pipeline at all -- but the port shape doesn't foreclose it.
- **Timing: this was a drop-in, not a stall-infrastructure change**, and
  that was verified, not assumed: `ap040_inst_fetch.v`'s old inline ROM read
  was already registered (1-cycle address-to-data latency, same as a real
  BRAM), so lifting the array out to `ap040_pipe_l1.v` and having IF drive
  its address combinationally reproduces the exact same timing. All 9
  existing pipe tests pass unmodified (only their hierarchical preload path
  changed, `dut.u_if.rom[N]` -> `dut.u_l1.mem[N]`, since the storage moved).
- **Decided (2026-08-27): registered + stalled**, not combinational. Port
  B's read matches `ap040_pipe_l1.v`'s own `q_b <= mem[address_b]` (and
  IF's port A precedent), not `ap040_pipe_regfile.v`'s zero-added-latency
  ports. Chosen over combinational specifically to avoid building a second
  timing model that would need redoing once a real cache-miss path exists
  later.

**DONE (milestone 9b, same day): MOVE.L (An),Dn.** Port B is 32-bit
(read/write, both adjacent 16-bit words in one access -- see
`ap040_pipe_l1.v`'s header for why: this core's internal memory interface
is meant to be 32-bit-native per section 1, with 16-bit-bus splitting left
to a future host adapter, not built into the core). `ap040_ea_fetch.v`
gained a two-state `mem_issue`/`mem_complete` FSM that needed no new
register-forwarding logic at all -- An's value already resolves through the
EXACT SAME `operand_a` priority mux (regfile read + EX-forward) every
register-direct source operand has used since milestone 2, because decode
sets `eac_src_reg` to An's own unified regfile index (8+n) rather than
inventing a separate address-register field. The one-cycle stall reuses the
existing freeze-on-`stall_in` behavior every stage has had since milestone 1
(asserting `eaf_stall` for exactly the issuing cycle freezes EA-calc's
output, so it still describes the same instruction on the completing cycle)
-- no separate pending-instruction latch needed in EA-fetch.

Two real bugs surfaced during development, worth recording:
1. **A genuine RTL bug**, caught by the test before it shipped: the first
   draft had EA-fetch address L1 port B as an ABSOLUTE byte address
   (`operand_a >> 1`), while `ap040_inst_fetch.v` addresses port A
   PC_RESET-RELATIVE (`(fetch_pc - PC_RESET) >> 1`) -- two different
   conventions into the SAME shared array silently breaks the entire point
   of unifying it. Fixed by giving `ap040_ea_fetch.v` the same `PC_RESET`
   parameter and using the identical convention; confirmed the mismatch is
   real and now caught by mutation-testing it back in (reproduces the exact
   `4e714e71` symptom the real bug produced).
2. **A test-methodology bug, not RTL**: the test seeds `A0` via a
   hierarchical poke (`dut.u_regfile.areg[0] = ...`, no MOVEA/LEA yet to
   load it architecturally) placed immediately after `nreset = 1`. A classic
   blocking/non-blocking race: `ap040_pipe_regfile.v`'s own reset block
   samples `nreset` in the SAME edge's active region (still 0) and schedules
   `areg[i] <= 0` as a non-blocking update, which then commits AFTER the
   poke's blocking assignment in the same timestep, silently overwriting it.
   `ap040_pipe_l1.v`'s ROM pokes never hit this because `mem[]` has no reset
   logic at all. Fixed by waiting one more clock edge past `nreset = 1`
   before poking. General lesson for any FUTURE test that pokes a resettable
   register (not memory): the reset edge itself is not a safe time to poke
   across.

Three independent mutations confirmed the mechanism is actually exercised,
not passing by coincidence: dropping `mem_issue` from `eaf_stall` (EA-calc
advances early -> the memory value lands in the WRONG register, D1 instead
of D0, not just a wrong value in the right one), swapping the L1 word order
(byte-swapped-at-the-word-level result), and the PC_RESET-omission mutation
above.

Not exercised by this test, explicitly deferred rather than assumed: EX-
forwarding INTO the address computation (a producer writing An immediately
before this instruction reads it) -- mechanically covered by the reused
`operand_a` mux, but nothing tests it yet since MOVEA/LEA don't exist to
generate that producer. Revisit once they do.

**DONE (milestone 10, 2026-08-27): MOVE.L (d16,An),Dn**, and with it a real,
previously-shipped bug in the unified L1 itself -- found because this was
the first test where a stall from one instruction's own memory access
happened to overlap with a SECOND instruction's multi-word gather still in
flight, a combination milestone 9b's test never produced.

- **The EA arithmetic itself needed almost no new machinery.** `id_imm`
  (previously only MOVEQ's immediate) now carries the sign-extended
  displacement, reusing `ap040_decode.v`'s existing gather machinery (a
  third trigger alongside Bcc.W and DBcc) and the same `gather_disp` wire
  those two already compute. `ap040_ea_fetch.v`'s address became
  `operand_a + eac_imm` unconditionally -- 0 for plain `(An)`, the real
  displacement for `(d16,An)` -- one formula, not a per-mode branch.
- **A real, needed guard**: every prior gather user (Bcc.W/L, DBcc) wants
  IF's speculative redirect; `(d16,An)`'s "displacement" is a memory offset,
  not a branch target, and would have hijacked IF into jumping to a garbage
  address had `redirect_from_gather` not been explicitly gated
  `&& !held_is_move_disp`. Confirmed load-bearing by mutation (dropping the
  guard breaks the test, not just theoretically).
- **A real, load-bearing companion fix**: `id_imm` was previously set to the
  sign-extended opcode low byte for EVERY instruction (harmless when nothing
  consumed it for non-MOVEQ cases). Once `ap040_ea_fetch.v` started actually
  ADDING `eac_imm` to every memory address, the plain `(An)` case would have
  silently added its own opcode's low byte as a phantom displacement.
  Zeroed explicitly for `is_move_mem_l`; confirmed load-bearing by mutation
  (breaks `tb_ap040_pipe_move_mem.v`, the milestone 9b test, when reverted).

**The real find: `ap040_pipe_l1.v` port A had no enable, and would silently
desynchronize from `if_opcode` across any stall longer than the instant it
started.** `ap040_inst_fetch.v`'s internal `pc` register is *always* one
step ahead of `if_pc`/`if_opcode` by design (normal single-stage-prefetch
shape: `pc` is the next fetch address, `if_pc`/`if_opcode` is what's
currently presented, one cycle younger). A stall correctly freezes both
registers together -- but `pc` freezes at its own, already-more-advanced
value. Port A's `q_a` had no idea any of this happened and kept
`<= mem[address_a]` on every single clock edge regardless, and since
`address_a` is a combinational function of `pc`, it kept reading the
address `pc` had *already* moved to -- one word past what `if_opcode` was
supposed to keep presenting -- silently overwriting it after just one
stalled cycle. `tb_ap040_pipe_move_mem.v` never surfaced this because its
own stall's collateral damage always landed on a harmless NOP; this
milestone's case B (a second gather needing its extension word held steady
across an EARLIER, unrelated instruction's memory stall) is what finally
landed the overwrite on a real, non-inert word (`MOVEQ #9,D2`'s own
opcode), turning silent corruption into a wrong register value. Fixed with
a new `en_a` port on `ap040_pipe_l1.v`, wired from `ap040_pipe_core.v` as
`ce && !id_stall` -- the EXACT condition already gating
`ap040_inst_fetch.v`'s own `if_pc`/`pc` registers, so port A now freezes in
lockstep with what it's supposed to represent instead of free-running past
it. Confirmed load-bearing by mutation, reproducing the exact `4e714e71`
garbage-NOP symptom the original bug produced. Port B needed no equivalent
fix -- its address is a direct combinational function of `ap040_ea_calc.v`'s
own already-stall-frozen output, not an independently-advancing register
the way IF's `pc` is; see `ap040_pipe_l1.v`'s header for the full argument
and the flag for revisiting it if that ever changes.

General lesson, worth carrying forward: **a stalled pipeline register
freezing correctly is not sufficient -- everything that register's value
combinationally FEEDS must also stop advancing, including free-running
memories with no natural concept of "stall."** This class of bug is
specifically invisible to single-instruction tests and to tests whose
stalls never overlap with another in-flight multi-cycle operation; it took
a second gather colliding with a first instruction's own memory stall to
surface it. Worth deliberately constructing scenarios like this again once
more stall sources exist (a real L1 miss, a store's write-then-read
hazard), not just testing one mechanism at a time in isolation.

## 5. Verification discipline (keep doing this)

This is what has actually kept the pipeline correct across ten milestones
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
- **Test stall OVERLAP, not just stalls in isolation.** Milestone 10 found a
  real, already-shipped bug (`ap040_pipe_l1.v` port A free-running past a
  stall -- see section 5a) that no single-mechanism test could have caught:
  it only manifests when a stall from one in-flight operation collides with
  a SECOND, independent multi-cycle operation (a gather) still assembling.
  When adding a new stall source, deliberately construct a test where it
  overlaps something else already in flight, not just a test of the new
  stall by itself.

## 6. Remaining scope, roughly in dependency order

1. **DONE (milestones 9b/10): `MOVE.L (An),Dn` and `MOVE.L (d16,An),Dn`** --
   see section 5a. Next EA-mode candidates, in increasing order of new
   machinery needed: `(An)+`/`-(An)` (An update -- needs a real WRITE to the
   regfile from EA-calc/EA-fetch, not just a read, and a decision on WHEN
   the update commits relative to a fault), then indexed/absolute/
   PC-relative.
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

# Handoff: AP68040 pipeline integration

This file's previous contents were a hardware-debugging log (sdram_ctrl,
ram_cs_guard, MiSTer board access) for the Minimig-AGA_MiSTer monorepo this
core was extracted from -- none of those files (`Minimig.sv`, `sdram_ctrl.v`,
`ram_cs_guard.v`, ...) exist in this repo, and that investigation has no
bearing on the standalone core. That history is still real and still lives
in the original project's tree/git history if it's ever needed again; it
just doesn't belong in this repo's handoff doc. Replaced with a handoff for
the actual work here.

For the full architecture spec, current milestone status, and remaining
scope, see `AP040_IMPLEMENTATION_PLAN.md` -- that's the document to keep
current going forward. This file is for short, dated session-to-session
continuity notes.

## 2026-08-26: repo extraction cleanup + milestone 8 (DBcc)

The pipeline branch was moved into this new standalone repo
(`rtl_old/` = the working sequential core, `rtl/` = the in-progress pipeline,
same layout `ap040_pipe_core.v`'s header already documented). Two things
were actually broken, both environmental, not RTL:

- **`iverilog` wasn't installed on this machine at all** -- every
  `tb_ap040_pipe_*.v` bench was correctly written and would have passed the
  whole time; there was nothing to fix in the RTL or the tests themselves.
  Fixed with `brew install icarus-verilog` (13.0). No test file changed.
- **`tb/run_tests.sh` (the `rtl_old` reference suite) still pointed
  `RTL=../rtl`**, which is now the pipeline directory and doesn't contain
  `ap040_core.v`/`ap040_tg68k_compat.v`/etc. Repointed to `RTL=../rtl_old`.
  That suite's assembly-based legs also need `vasmm68k_mot` (vbcc), which
  isn't in Homebrew and wasn't installed here -- confirmed the script now
  fails at the (expected, documented) assembly step rather than the wrong
  place, and confirmed the six non-assembly benches (reset, double fault,
  walker CDC, bus16 gap, bus timeout, cache snoop) pass standalone against
  `rtl_old` with just `iverilog`.
- Added `tb/run_pipe_tests.sh` so the pipeline suite has a real runner
  instead of being invoked by hand per-file (which is how the milestone-1
  through 7 testbenches had been getting run/verified up to this point).

Then continued the actual integration work: milestone 8, **DBcc
Dn,\<label\>**, following the plan's own note that it was the natural next
target (its prerequisite, the multi-word decode gather, already existed
from milestone 7). Verified bit-for-bit against `rtl_old/ap040_core.v`'s
`S_DBCC1`/`S_DBCC2` (lines ~3040-3059) and its `0x5` ADDQ/SUBQ/Scc/DBcc
decode group (lines ~5415-5421). Summary (full reasoning is in
`ap040_execute.v`/`ap040_decode.v`'s header comments, cited by line number
the same way every prior milestone's has been):

- Reuses the Bcc.W gather mechanism as-is (DBcc is always exactly one
  extension word) and the existing `ex_mispredict`/`ex_recovery_pc` wiring
  completely unchanged -- both of DBcc's "don't branch" outcomes (condition
  true, or the decremented counter hit `$FFFF`) are, from the front end's
  point of view, identical to a not-taken Bcc.
- New mechanism: DBcc is the first instruction whose register write depends
  on a **runtime** value, not a static decode-time bit (`writes_reg_resolved`
  in `ap040_execute.v`, gated on `!cond_result`). Three independent
  mutations (disable the dynamic gate entirely, drop DBcc from the
  mispredict condition, remove the cond_result gate but keep the branch
  logic) were each confirmed to break `tb_ap040_pipe_dbcc.v` in a
  distinguishable way before trusting the test.
- Explicitly deferred, matching the same discipline TRAPcc's deferral used
  in milestone 6: the real 68040 checks the branch target's parity *before*
  the condition (an odd target always faults, even on an iteration that
  wouldn't have branched -- `rtl_old`'s own S_DBCC1 comment cites cputest
  68040_ae DBcc.W for this). That needs exception delivery, which the
  pipeline doesn't have yet; not implemented rather than guessed at.
- Compressed `AP040_IMPLEMENTATION_PLAN.md` from a 1380-line narrative log
  (much of it Minimig-fitting/performance-tuning detail that doesn't apply
  to this standalone repo) into a ~200-line current-state-plus-roadmap
  document; the detailed per-milestone reasoning it used to duplicate is
  already anchored in the RTL file header comments with line citations, so
  nothing was actually lost, just de-duplicated.

Next candidate per the plan: the first memory-referencing instruction
(suggest `MOVE.L (An),Dn`) -- see `AP040_IMPLEMENTATION_PLAN.md` section 6.
That's the actual biggest remaining gap; everything implemented through
milestone 8 is register-direct/immediate only.

## 2026-08-27: CRITIQUE.md review + milestone 9a (unified L1)

The user pasted an external LLM-generated `CRITIQUE.md` (68040-dev-review
style) and asked about two things it raised: CCR timing, and whether to
build a unified Von Neumann L1 (dual-port BRAM, shared I/D) ahead of the
first memory instruction, prefilled directly by testbenches the way IF's ROM
already was.

**CRITIQUE.md accuracy notes** (for whoever reads it next -- it's still in
the repo as the user's own artifact, not corrected in place):
- Its "SCCR, DCCR: two separate CCRs for supervisor/data modes" row is
  fabricated -- no such registers exist on the MC68040. There is exactly one
  5-bit CCR (X/N/Z/V/C), the low byte of the 16-bit SR.
- Its CCR-forwarding recommendation ("shift the write-back of ccr to the ALU
  stage instead of WB") is already effectively true: `ccr_resolved` in
  `ap040_pipe_core.v` write-through-forwards the EX-computed flags to a
  same-cycle consumer, the identical mechanism the regfile itself uses for
  GPRs. This was mutation-tested in milestone 5 (the "immediately adjacent
  flag-setter" case in `tb_ap040_pipe_bcc.v`) specifically to prove the
  tightest-gap case works. No change was needed.
- Its "SR register... not modeled" row is a real, correctly-identified gap
  (S/M/T0/T1/IPL don't exist, only the 5-bit CCR does), but it's not urgent
  in isolation -- it's naturally sequenced with exception delivery (section
  6 item 3), since privilege/trace bits are meaningless without anywhere to
  trap to.
- Everything else in it is a reasonable, correctly-scoped observation about
  a work-in-progress core (single-word fetch, no addressing modes yet, no
  cache/stall logic) -- it's describing this pipeline accurately at the
  stage it caught it at, not raising alarms about actual bugs.

**Unified L1 decision:** agreed with the user's proposal and built the first
slice -- see `AP040_IMPLEMENTATION_PLAN.md` section 5a for the full
reasoning (coherency-for-free vs. the CINV/CPUSH semantic gap it creates,
why CACR's independent I/D enables aren't a blocker, why this was a
zero-risk drop-in for IF's timing). Concretely: `ap040_pipe_l1.v` (a fork of
`rtl_old/primitives/dpram.v`'s port convention, true dual-port, NOP-filled
by default), `ap040_inst_fetch.v` now drives its address combinationally
instead of owning storage, port B sits reserved and tied off at
`ap040_pipe_core.v` waiting for the first memory instruction. All 9 existing
pipe tests pass with zero behavior change (only their hierarchical preload
path moved, `dut.u_if.rom[N]` -> `dut.u_l1.mem[N]`).

Left open, deliberately: whether port B's read (once milestone 9b actually
consumes it) is combinational or a genuine registered/stalled read -- this
is the first time any of this pipeline's `*_stall` wiring would do
something real, so it's worth a deliberate answer rather than a default.

## 2026-08-27 (later): milestone 9b, MOVE.L (An),Dn

User picked registered+stalled for L1 port B. Built it same session --
`ap040_ea_fetch.v` gained a two-state `mem_issue`/`mem_complete` FSM; needed
no new forwarding logic since An's value already flows through the existing
`operand_a` mux (decode just sets `eac_src_reg` to An's unified index, 8+n,
same 4-bit space every register-direct source already used). Full reasoning
and the two real bugs caught during development (a genuine RTL addressing-
convention mismatch between IF's and EA-fetch's L1 ports, and a testbench
NBA-race poking a register too early) are in
`AP040_IMPLEMENTATION_PLAN.md` section 5a -- worth reading in full before
touching `ap040_ea_fetch.v` again, especially the PC_RESET-relative
addressing convention, which is easy to silently break a second way.

`tb_ap040_pipe_move_mem.v` mutation-tested three ways (drop the stall,
swap the L1 word order, drop the PC_RESET base) -- all caught distinctly.
Full 10-file `run_pipe_tests.sh` green.

Next: an EA mode that needs real EA-calc arithmetic (`(d16,An)` is the
natural candidate -- reuses the existing multi-word decode gather) or
`(An)+`/`-(An)` (needs the pipeline's first register WRITE originating from
EA-calc/EA-fetch, not just EX) -- see `AP040_IMPLEMENTATION_PLAN.md`
section 6 item 1.

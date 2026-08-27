//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 8: DBcc)            //
//                                                                          //
// ap040_pipe_core.v - top level: wires IF/ID/EA-calc/EA-fetch/EX/WB into a //
// real six-stage pipeline with a synchronous stall/flush chain,            //
// instantiates this directory's own register file and ALU forks and a new //
// architectural CCR, drives the commit (regfile write + CCR update)        //
// directly off EX's registered output, and exposes debug taps for the      //
// test.                                                                    //
//                                                                          //
// Deliberately named ap040_pipe_core, not ap040_core: it must never        //
// collide with, or be silently substitutable for, the working sequential   //
// core in rtl/ap040/ap040_core.v (see the note in                          //
// rtl/ap040/experimental/README about ap040_core_fetchbuffer.v reusing     //
// that name). This module is not referenced by ap040.qip, cpu_wrapper.v    //
// or Minimig.sv -- it is built and tested standalone until a later         //
// milestone wires it in deliberately.                                     //
//                                                                          //
// Folder independence (new this milestone): earlier milestones reused      //
// rtl/ap040/ap040_regfile.v and rtl/ap040/ap040_alu.v directly, adding a    //
// couple of small, deliberately-additive changes to the shared files       //
// (a WRITE_THROUGH parameter, a debug tap). The user decided that's not    //
// the structure wanted: rtl/ap040/ stays completely untouched from here    //
// on, and rtl/ap040_pipe/ is fully self-contained -- deleting either        //
// directory can never affect the other. Both shared files are forked into  //
// this directory as ap040_pipe_regfile.v/ap040_pipe_alu.v (same collision- //
// avoidance naming as this module itself), and ap040_pipe_defs.svh no      //
// longer reaches into rtl/ap040/ for constants either.                     //
//                                                                          //
// Stall handshake: each stage's *_stall output is driven straight from its //
// own stall_in input (no stage has multi-cycle work yet), so the six-stage //
// chain currently collapses to "never stall" -- but the wiring is real,    //
// stage by stage, so a later milestone only has to change one stage's      //
// local stall condition rather than restructure the pipeline. The whole    //
// core advances only when ce is high, matching the clock-enable discipline //
// used throughout rtl/ap040/ap040_core.v.                                  //
//                                                                          //
// Commit and forwarding: the register file is instantiated here (not      //
// inside a stage module), mirroring where the existing sequential core     //
// instantiates it (ap040_core.v:222-233). The write port is driven         //
// directly off exe_* (EX's registered output -- the instruction            //
// conceptually "in WB" this cycle). A same-cycle read of the register      //
// being written is resolved inside the regfile itself (unconditional in    //
// this fork -- see ap040_pipe_regfile.v); the other hazard case -- a        //
// producer still combinationally computing in EX, one stage ahead of a      //
// consumer's EA-fetch cycle -- is resolved in ap040_ea_fetch.v via the      //
// ex_fwd_* bus below. See ap040_ea_fetch.v's header comment for the full    //
// picture, including the mutation test that confirmed a separate            //
// WB-forward mux there would have been redundant.                          //
//                                                                          //
// CCR forwarding: the same write-through shape used for GPRs is reused     //
// for the single CCR register: ccr_resolved = commit_ccr ? exe_result_     //
// flags : ccr. This is the only forwarding source ap040_execute.v's Bcc     //
// condition check (and, this milestone, Scc's byte fill) needs -- see its   //
// header comment for why an EX-live tap (the second source GPR forwarding   //
// needed) isn't necessary here.                                            //
//                                                                          //
// commit_reg vs commit_ccr (new this milestone): these were a single        //
// `commit` signal through milestone 5, correct because every writing        //
// instruction so far (MOVEQ/MOVE.L/ADD.L) always did both together. Scc     //
// breaks that -- it writes a register but must never touch CCR (confirmed   //
// against ap040_core.v:2122-2130's EK_SCC) -- so the regfile write enable   //
// and the CCR update enable are now genuinely separate signals, gated by    //
// exe_writes_reg and exe_writes_ccr respectively.                          //
//                                                                          //
// Field set note (2026-08-21 code-quality pass): the original milestone-2  //
// draft threaded opcode/is_nop/is_move_rr/unimpl through every stage as    //
// one-hot-ish flags. A review found most of that unread past the stage     //
// that produced it, and the scheme didn't generalize. Decode now emits a   //
// compact control word instead; see ap040_decode.v's header for the        //
// reasoning, confirmed again by milestone 3 (ADD) and milestone 6 (Scc)     //
// each needing only new fields/values, never a restructure.                //
//                                                                          //
// Milestone 4 added BRA.B's redirect as a genuinely separate, orthogonal   //
// path (id_redirect_valid/id_redirect_pc, decode -> IF) rather than         //
// folding it into the control word -- it isn't an ALU operation or a        //
// register write, it's IF's own next-fetch address.                        //
//                                                                          //
// Milestone 5 generalized that redirect to the whole Bcc family (BRA is     //
// just Bcc's always-true case, per ap040_decode.v's header) and added the   //
// real misprediction path: ex_mispredict/ex_recovery_pc (from EX, once the  //
// real condition is known) take PRIORITY over decode's speculative guess    //
// at IF's redirect mux, and broadcast as `flush` to ID/EA-calc/EA-fetch to  //
// discard whatever was speculatively advanced down the (wrong) guess.       //
//                                                                          //
// Milestone 7 adds Bcc.W/Bcc.L (16-/32-bit displacement) via a multi-word   //
// gather state machine confined entirely to ap040_decode.v -- IF still      //
// hands over one word per cycle, oblivious to instruction boundaries; see   //
// its header comment. The one change visible at this level is *_next_pc,   //
// threaded alongside *_pc through every stage: ex_recovery_pc now reads     //
// eaf_next_pc directly instead of computing eaf_pc + 32'd2, since that       //
// arithmetic assumed every instruction was exactly one word (true by        //
// coincidence until Bcc.W/Bcc.L).                                          //
//                                                                          //
// Milestone 8 adds DBcc via a THIRD gate on the same commit signals rather   //
// than new ones: *_is_dbcc threads alongside *_is_branch/*_is_scc through    //
// every stage (mechanical pass-through, same shape as those two), and        //
// ap040_execute.v's writes_reg_resolved -- not this file -- is what makes     //
// commit_reg dynamic for it (DBcc writes Dn only when its runtime condition   //
// says so; see ap040_execute.v's header). ex_mispredict/ex_recovery_pc are    //
// reused completely unchanged: DBcc's "don't branch" outcomes are, from       //
// this file's point of view, indistinguishable from a not-taken Bcc.         //
//--------------------------------------------------------------------------//

module ap040_pipe_core
#(
	parameter [31:0] PC_RESET   = 32'h0000_0400,
	parameter         PROG_WORDS = 10,
	parameter         L1_AW      = 12   // ap040_pipe_l1.v size: 2**L1_AW words
)
(
	input  clk,
	input  nreset,
	input  ce,

	output        dbg_if_valid,  output [31:0] dbg_if_pc,
	output        dbg_id_valid,  output [31:0] dbg_id_pc,
	output        dbg_eac_valid, output [31:0] dbg_eac_pc,
	output        dbg_eaf_valid, output [31:0] dbg_eaf_pc,
	output        dbg_ex_valid,  output [31:0] dbg_ex_pc,
	output        dbg_wb_valid,  output [31:0] dbg_wb_pc,

	output [31:0] dbg_d0, output [31:0] dbg_d1, output [31:0] dbg_d2, output [31:0] dbg_d3,
	output [31:0] dbg_d4, output [31:0] dbg_d5, output [31:0] dbg_d6, output [31:0] dbg_d7,
	output  [4:0] dbg_ccr
);

wire        if_valid;  wire [31:0] if_pc;  wire [15:0] if_opcode;

wire        id_valid;  wire [31:0] id_pc;  wire [31:0] id_next_pc;
wire  [3:0] id_dest_reg, id_src_reg;
wire [31:0] id_imm;
wire  [5:0] id_alu_op;
wire        id_src_a_is_imm, id_writes_reg, id_writes_ccr;
wire        id_is_branch, id_is_scc, id_is_dbcc, id_is_mem_src, id_is_jmp;
wire  [3:0] id_cond;
// id_unimpl: decode's "opcode not recognized" bit. Not consumed past decode
// yet -- nothing traps on it (writes_reg=0 already makes an unrecognized
// opcode inert) -- kept as a real output, not threaded further, because the
// illegal-instruction-exception milestone needs it and re-deriving "which
// opcodes decode() actually classifies" from scratch at that point would be
// wasted work. Reserved the same way raddr_b is below, not dead.
wire        id_unimpl;

wire        eac_valid; wire [31:0] eac_pc; wire [31:0] eac_next_pc;
wire  [3:0] eac_dest_reg, eac_src_reg;
wire [31:0] eac_imm;
wire  [5:0] eac_alu_op;
wire        eac_src_a_is_imm, eac_writes_reg, eac_writes_ccr;
wire        eac_is_branch, eac_is_scc, eac_is_dbcc, eac_is_mem_src, eac_is_jmp;
wire  [3:0] eac_cond;

wire        eaf_valid; wire [31:0] eaf_pc; wire [31:0] eaf_next_pc;
wire  [3:0] eaf_dest_reg;
wire [31:0] eaf_operand_a, eaf_operand_b;
wire  [5:0] eaf_alu_op;
wire        eaf_writes_reg, eaf_writes_ccr;
wire        eaf_is_branch, eaf_is_scc, eaf_is_dbcc, eaf_is_jmp;
wire  [3:0] eaf_cond;

wire        exe_valid; wire [31:0] exe_pc;
wire  [3:0] exe_dest_reg;
wire [31:0] exe_result_data;
wire        exe_writes_reg, exe_writes_ccr;
wire  [4:0] exe_result_flags;

wire        wb_valid;  wire [31:0] wb_pc;

// Backward stall chain: WB has no downstream, so its stall_in is tied 0;
// every earlier stage's stall_in is the next stage's *_stall output.
wire id_stall, ea_stall, eaf_stall, ex_stall, wb_stall;

// Bcc/BRA speculative redirect (decode -> IF), see ap040_decode.v's header.
wire        id_redirect_valid;
wire [31:0] id_redirect_pc;

// Misprediction recovery (EX -> everything upstream of it), see
// ap040_execute.v's header for why the check happens at EX.
wire        ex_mispredict;
wire [31:0] ex_recovery_pc;

// Recovery takes priority over a fresh speculative guess -- fixing a
// confirmed wrong guess matters more than starting a new one the same
// cycle (and in practice they concern different instructions anyway).
wire        final_redirect_valid = ex_mispredict || id_redirect_valid;
wire [31:0] final_redirect_pc    = ex_mispredict ? ex_recovery_pc : id_redirect_pc;

// Broadcast flush: discards whatever ID/EA-calc/EA-fetch are currently
// holding, all speculatively advanced down the (wrong) assumed-taken guess.
wire flush = ex_mispredict;

// EA-fetch's regfile operand read ports.
wire  [3:0] raddr_a, raddr_b;
wire [31:0] rdata_a, rdata_b;

// EX-forward tap (combinational, live this cycle). The other hazard case --
// a producer committing the same cycle a consumer reads it -- needs no
// separate top-level wiring: it's resolved inside ap040_pipe_regfile.v
// itself. See ap040_ea_fetch.v's header comment for the mutation test that
// confirmed this empirically.
wire        ex_fwd_valid;
wire  [3:0] ex_fwd_dest;
wire [31:0] ex_fwd_data;

// The instruction committing this cycle. Two separate gates, not one --
// see this file's header comment on commit_reg vs commit_ccr.
wire commit_reg = exe_valid && exe_writes_reg;
wire commit_ccr = exe_valid && exe_writes_ccr;

// Architectural CCR (XNZVC), updated at commit_ccr (not commit_reg -- Scc
// writes a register without touching CCR).
reg [4:0] ccr;

always @(posedge clk) begin
	if (!nreset)
		ccr <= 5'h0;
	else if (ce && commit_ccr)
		ccr <= exe_result_flags;
end

assign dbg_ccr = ccr;

// CCR write-through forward, mirroring the regfile's own write-through
// bypass: a producer committing THIS cycle is visible to a same-cycle
// reader without waiting for the ccr register's own next edge. This is the
// only CCR-forwarding source ap040_execute.v's Bcc/Scc condition check
// needs -- see its header comment. Gated on commit_ccr, not commit_reg:
// Scc committing must not forward its (meaningless, discarded) ALU flags.
wire [4:0] ccr_resolved = commit_ccr ? exe_result_flags : ccr;

//---------------------------------------------------------------------------
// Register file: this directory's own fork (ap040_pipe_regfile.v, see its
// header), instantiated here rather than inside a stage, mirroring where
// the sequential core instantiates its own (ap040_core.v:222-233). Read
// port B is the destination register's current value, read unconditionally
// for every instruction -- see ap040_ea_fetch.v's header comment for why
// that's safe and why no control bit gates it.
//---------------------------------------------------------------------------

ap040_pipe_regfile u_regfile
(
	.clk      (clk),
	.ce       (ce),
	.nreset   (nreset),

	.sr_s     (1'b1),   // supervisor, matches AP040_SR_RESET's S=1/M=0;
	.sr_m     (1'b0),   // nothing in this milestone touches A7/stack

	.we       (commit_reg),
	.waddr    (exe_dest_reg),
	.wdata    (exe_result_data),

	.raddr_a  (raddr_a),
	.rdata_a  (rdata_a),
	.raddr_b  (raddr_b),
	.rdata_b  (rdata_b),

	.aux_we   (1'b0),
	.aux_sel  (2'd0),
	.aux_wdata(32'h0),
	.usp_q    (),
	.isp_q    (),
	.msp_q    (),

	.dbg_d0   (dbg_d0),
	.dbg_d1   (dbg_d1),
	.dbg_d2   (dbg_d2),
	.dbg_d3   (dbg_d3),
	.dbg_d4   (dbg_d4),
	.dbg_d5   (dbg_d5),
	.dbg_d6   (dbg_d6),
	.dbg_d7   (dbg_d7),
	.dbg_a0   (),
	.dbg_a7   ()
);

//---------------------------------------------------------------------------
// Stages
//---------------------------------------------------------------------------

// Unified L1. Port A is IF's (milestone 9a). Port B is EA-fetch's data
// read (milestone 9b, new) -- write side (data_b/wren_b/wr_busy) still tied
// off: nothing writes memory yet, reserved for a future store instruction
// the same "reserved, not speculative" way raddr_b was until Scc needed it.
wire [L1_AW-1:0] l1_addr_a;
wire      [15:0] l1_rdata_a;
wire [L1_AW-1:0] l1_addr_b;
wire       [31:0] l1_q_b;
wire              l1_wr_busy;

ap040_pipe_l1 #(
	.AW(L1_AW),
	.DW(16)
) u_l1
(
	.clock     (clk),
	.nreset    (nreset),

	.address_a (l1_addr_a),
	.data_a    (16'h0),
	.wren_a    (1'b0),
	// Must match ap040_inst_fetch.v's OWN if_pc/pc update condition exactly
	// -- see ap040_pipe_l1.v's header (milestone 10 fix: without this, a
	// stall lets q_a free-run past the word if_opcode is supposed to keep
	// presenting).
	.en_a      (ce && !id_stall),
	.q_a       (l1_rdata_a),

	.address_b (l1_addr_b),
	.data_b    (32'h0),
	.wren_b    (1'b0),
	.wr_busy   (l1_wr_busy),
	.q_b       (l1_q_b)
);

ap040_inst_fetch #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS),
	.L1_AW     (L1_AW)
) u_if
(
	.clk       (clk),
	.nreset    (nreset),
	.ce        (ce),
	.stall_in  (id_stall),

	.redirect_valid (final_redirect_valid),
	.redirect_pc    (final_redirect_pc),

	.l1_addr_a  (l1_addr_a),
	.l1_rdata_a (l1_rdata_a),

	.if_valid  (if_valid),
	.if_pc     (if_pc),
	.if_opcode (if_opcode)
);

ap040_decode u_id
(
	.clk             (clk),
	.nreset          (nreset),
	.ce              (ce),
	.stall_in        (ea_stall),
	.flush           (flush),

	.if_valid        (if_valid),
	.if_pc           (if_pc),
	.if_opcode       (if_opcode),

	.id_stall        (id_stall),

	.id_redirect_valid (id_redirect_valid),
	.id_redirect_pc    (id_redirect_pc),

	.id_valid        (id_valid),
	.id_pc           (id_pc),
	.id_next_pc      (id_next_pc),
	.id_dest_reg     (id_dest_reg),
	.id_src_reg      (id_src_reg),
	.id_imm          (id_imm),
	.id_alu_op       (id_alu_op),
	.id_src_a_is_imm (id_src_a_is_imm),
	.id_writes_reg   (id_writes_reg),
	.id_writes_ccr   (id_writes_ccr),
	.id_is_branch    (id_is_branch),
	.id_is_scc       (id_is_scc),
	.id_is_dbcc      (id_is_dbcc),
	.id_is_mem_src   (id_is_mem_src),
	.id_is_jmp       (id_is_jmp),
	.id_cond         (id_cond),
	.id_unimpl       (id_unimpl)
);

ap040_ea_calc u_eac
(
	.clk              (clk),
	.nreset           (nreset),
	.ce               (ce),
	.stall_in         (eaf_stall),
	.flush            (flush),

	.id_valid         (id_valid),
	.id_pc            (id_pc),
	.id_next_pc       (id_next_pc),
	.id_dest_reg      (id_dest_reg),
	.id_src_reg       (id_src_reg),
	.id_imm           (id_imm),
	.id_alu_op        (id_alu_op),
	.id_src_a_is_imm  (id_src_a_is_imm),
	.id_writes_reg    (id_writes_reg),
	.id_writes_ccr    (id_writes_ccr),
	.id_is_branch     (id_is_branch),
	.id_is_scc        (id_is_scc),
	.id_is_dbcc       (id_is_dbcc),
	.id_is_mem_src    (id_is_mem_src),
	.id_is_jmp        (id_is_jmp),
	.id_cond          (id_cond),

	.ea_stall         (ea_stall),

	.eac_valid        (eac_valid),
	.eac_pc           (eac_pc),
	.eac_next_pc      (eac_next_pc),
	.eac_dest_reg     (eac_dest_reg),
	.eac_src_reg      (eac_src_reg),
	.eac_imm          (eac_imm),
	.eac_alu_op       (eac_alu_op),
	.eac_src_a_is_imm (eac_src_a_is_imm),
	.eac_writes_reg   (eac_writes_reg),
	.eac_writes_ccr   (eac_writes_ccr),
	.eac_is_branch    (eac_is_branch),
	.eac_is_scc       (eac_is_scc),
	.eac_is_dbcc      (eac_is_dbcc),
	.eac_is_mem_src   (eac_is_mem_src),
	.eac_is_jmp       (eac_is_jmp),
	.eac_cond         (eac_cond)
);

ap040_ea_fetch #(
	.PC_RESET (PC_RESET),
	.L1_AW    (L1_AW)
) u_eaf
(
	.clk              (clk),
	.nreset           (nreset),
	.ce               (ce),
	.stall_in         (ex_stall),
	.flush            (flush),

	.eac_valid        (eac_valid),
	.eac_pc           (eac_pc),
	.eac_next_pc      (eac_next_pc),
	.eac_dest_reg     (eac_dest_reg),
	.eac_src_reg      (eac_src_reg),
	.eac_imm          (eac_imm),
	.eac_alu_op       (eac_alu_op),
	.eac_src_a_is_imm (eac_src_a_is_imm),
	.eac_writes_reg   (eac_writes_reg),
	.eac_writes_ccr   (eac_writes_ccr),
	.eac_is_branch    (eac_is_branch),
	.eac_is_scc       (eac_is_scc),
	.eac_is_dbcc      (eac_is_dbcc),
	.eac_is_mem_src   (eac_is_mem_src),
	.eac_is_jmp       (eac_is_jmp),
	.eac_cond         (eac_cond),

	.raddr_a          (raddr_a),
	.rdata_a          (rdata_a),
	.raddr_b          (raddr_b),
	.rdata_b          (rdata_b),

	.ex_fwd_valid     (ex_fwd_valid),
	.ex_fwd_dest      (ex_fwd_dest),
	.ex_fwd_data      (ex_fwd_data),

	.l1_addr_b        (l1_addr_b),
	.l1_q_b           (l1_q_b),

	.eaf_stall        (eaf_stall),

	.eaf_valid        (eaf_valid),
	.eaf_pc           (eaf_pc),
	.eaf_next_pc      (eaf_next_pc),
	.eaf_dest_reg     (eaf_dest_reg),
	.eaf_operand_a    (eaf_operand_a),
	.eaf_operand_b    (eaf_operand_b),
	.eaf_alu_op       (eaf_alu_op),
	.eaf_writes_reg   (eaf_writes_reg),
	.eaf_writes_ccr   (eaf_writes_ccr),
	.eaf_is_branch    (eaf_is_branch),
	.eaf_is_scc       (eaf_is_scc),
	.eaf_is_dbcc      (eaf_is_dbcc),
	.eaf_is_jmp       (eaf_is_jmp),
	.eaf_cond         (eaf_cond)
);

ap040_execute u_ex
(
	.clk              (clk),
	.nreset           (nreset),
	.ce               (ce),
	.stall_in         (wb_stall),

	.eaf_valid        (eaf_valid),
	.eaf_pc           (eaf_pc),
	.eaf_next_pc      (eaf_next_pc),
	.eaf_dest_reg     (eaf_dest_reg),
	.eaf_operand_a    (eaf_operand_a),
	.eaf_operand_b    (eaf_operand_b),
	.eaf_alu_op       (eaf_alu_op),
	.eaf_writes_reg   (eaf_writes_reg),
	.eaf_writes_ccr   (eaf_writes_ccr),
	.eaf_is_branch    (eaf_is_branch),
	.eaf_is_scc       (eaf_is_scc),
	.eaf_is_dbcc      (eaf_is_dbcc),
	.eaf_is_jmp       (eaf_is_jmp),
	.eaf_cond         (eaf_cond),

	.ccr_in           (ccr_resolved),

	.ex_stall         (ex_stall),

	.ex_fwd_valid     (ex_fwd_valid),
	.ex_fwd_dest      (ex_fwd_dest),
	.ex_fwd_data      (ex_fwd_data),

	.ex_mispredict    (ex_mispredict),
	.ex_recovery_pc   (ex_recovery_pc),

	.exe_valid        (exe_valid),
	.exe_pc           (exe_pc),
	.exe_dest_reg     (exe_dest_reg),
	.exe_result_data  (exe_result_data),
	.exe_writes_reg   (exe_writes_reg),
	.exe_writes_ccr   (exe_writes_ccr),
	.exe_result_flags (exe_result_flags)
);

ap040_writeback u_wb
(
	.clk              (clk),
	.nreset           (nreset),
	.ce               (ce),
	.stall_in         (1'b0),

	.exe_valid        (exe_valid),
	.exe_pc           (exe_pc),

	.wb_stall         (wb_stall),

	.wb_valid         (wb_valid),
	.wb_pc            (wb_pc)
);

assign dbg_if_valid  = if_valid;  assign dbg_if_pc  = if_pc;
assign dbg_id_valid  = id_valid;  assign dbg_id_pc  = id_pc;
assign dbg_eac_valid = eac_valid; assign dbg_eac_pc = eac_pc;
assign dbg_eaf_valid = eaf_valid; assign dbg_eaf_pc = eaf_pc;
assign dbg_ex_valid  = exe_valid; assign dbg_ex_pc  = exe_pc;
assign dbg_wb_valid  = wb_valid;  assign dbg_wb_pc  = wb_pc;

endmodule

//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 13: BSR, JSR)       //
//                                                                          //
// ap040_ea_fetch.v - EA-fetch stage                                       //
//                                                                          //
// This is where the pipeline's register forwarding lives. The top-level   //
// regfile (ap040_pipe_core.v, its own ap040_pipe_regfile.v fork) is read    //
// combinationally here on BOTH ports -- port A at eac_src_reg, port B at   //
// eac_dest_reg -- and each resolved, independently, as a flat priority mux //
// rather than a chain of ternaries:                                       //
//                                                                          //
//   ex_fwd_*  the instruction currently in EX (still combinational this   //
//             cycle, not yet committed) -- "producer one stage ahead"     //
//                                                                          //
// Port B (eac_dest_reg) is read unconditionally, for every instruction,   //
// not just ones that need it: it's a pure regfile read with no side       //
// effect, and AP040_ALU_MOVE (MOVEQ/MOVE.L's op) never references its `b` //
// input at all (ap040_pipe_alu.v:139-142), so fetching it for those two is //
// simply unused, not wrong -- and the same is true for a branch, whose      //
// eac_dest_reg field is meaningless leftover bits, harmless because         //
// eac_writes_reg is always 0 for one. Scc, this milestone, is the first     //
// instruction that actually NEEDS port B's value for something other than  //
// forwarding to a later instruction: it's the register Scc's byte-merge     //
// preserves the upper bits of (ap040_execute.v). This is the                //
// generalization the milestone-2 altitude review asked for instead of a     //
// per-opcode special case: adding another instruction needs no new mux      //
// shape here.                                                               //
//                                                                          //
// The other hazard case -- a producer committing to the regfile the exact //
// same cycle a consumer reads it -- does NOT need a second mux on either   //
// port: it's resolved inside ap040_pipe_regfile.v itself (its write-through //
// bypass, unconditional in that file -- see its header). A mutation test    //
// during milestone 2 development confirmed this empirically for port A --   //
// forcing this stage's own would-be WB-forward mux to always miss still     //
// passed, because rdata_a was already correct by the time it got here.      //
// Keeping a second, provably redundant mux here would be dead logic; see    //
// ap040_pipe_core.v for the write-through wiring.                           //
//                                                                          //
// id_is_branch/id_is_scc/id_is_dbcc/id_cond/id_next_pc ride through          //
// unchanged -- this stage has nothing to compute for any of them;            //
// ap040_execute.v is where the condition is actually checked, the Scc merge  //
// actually happens, the DBcc decrement/compare happens, and eaf_next_pc is   //
// finally consumed (as ex_recovery_pc), per its header comment on why.       //
//                                                                          //
// DBcc (milestone 8) needs no new mux shape here either: its loop-counter    //
// register is both eac_src_reg and eac_dest_reg (ap040_decode.v sets both     //
// to the same Dn), so operand_a already resolves to Dn's current,             //
// correctly-forwarded value via the existing port-A priority mux -- exactly   //
// the read a decrement needs, with no dedicated DBcc case added to either     //
// port's select logic.                                                      //
//                                                                          //
// flush: when ap040_execute.v detects a mispredicted branch, everything     //
// speculatively fetched behind it -- including whatever is sitting here -- //
// must be discarded. Same shape as stall_in but forces a bubble instead     //
// of holding.                                                              //
//                                                                          //
// MOVE.L (An),Dn (milestone 9b, new) is this pipeline's first memory read,   //
// and its first genuine stall. The address itself needs NO new logic: An's  //
// value is already what `operand_a` resolves to (decode set eac_src_reg to   //
// An's unified regfile index, see ap040_decode.v's header) -- the SAME       //
// priority mux, with the SAME EX-forward source, that every register-direct  //
// operand has used since milestone 2. What's new is turning that value into  //
// an actual ap040_pipe_l1.v port-B access:                                   //
//                                                                            //
//   mem_issue    (eac_valid && eac_is_mem_src && !mem_pending) -- the cycle  //
//                this stage first sees an unserviced memory instruction.     //
//                l1_addr_b is driven combinationally off operand_a THIS      //
//                cycle (ap040_pipe_l1.v registers l1_q_b from it, valid      //
//                next cycle); eaf_valid goes to 0 (nothing for EX yet); and  //
//                eaf_stall's mem_issue term freezes ap040_ea_calc.v's        //
//                OUTPUT for exactly this one cycle -- not a separate pending-//
//                instruction latch in THIS stage, just reusing the stall     //
//                chain's existing freeze-on-stall_in behavior (every stage    //
//                has had it since milestone 1). That's why eac_* is         //
//                guaranteed to still describe the SAME instruction next      //
//                cycle with no extra bookkeeping here.                       //
//   mem_complete (mem_pending, i.e. the cycle after issuing) -- eac_* is      //
//                still frozen on this same instruction, l1_q_b now holds the  //
//                fetched longword. Finishes exactly like the normal path      //
//                below, except eaf_operand_a comes from l1_q_b instead of     //
//                operand_a/regfile/forwarding. eaf_stall drops (mem_issue is  //
//                false once mem_pending is set), so ap040_ea_calc.v is free   //
//                to advance a NEW instruction into its output starting THIS   //
//                cycle -- the memory latency is hidden from whatever comes    //
//                next, not paid twice.                                       //
//                                                                            //
// L1 addressing: ap040_pipe_l1.v is addressed PC_RESET-relative (see          //
// ap040_inst_fetch.v's header) -- PC_RESET is the L1 window's BASE address,   //
// not "where execution starts" in isolation, and both ports must agree on    //
// it or the "unified" memory silently stops being unified. This stage takes  //
// the SAME PC_RESET parameter ap040_inst_fetch.v does (ap040_pipe_core.v     //
// passes the identical value to both) rather than assuming address 0.        //
//                                                                            //
// MOVE.L (d16,An),Dn (milestone 10, new): needed NO changes to the mem_issue/ //
// mem_complete FSM above -- ap040_decode.v now reuses id_imm to carry the      //
// sign-extended displacement (0 for the plain (An) case), and this stage's     //
// address computation became `operand_a + eac_imm` unconditionally (see        //
// where l1_addr_b is assigned below) rather than a per-mode branch. The         //
// gather that assembles the extension word happens entirely in                  //
// ap040_decode.v, same as Bcc.W/DBcc -- this stage never knows a gather           //
// happened, exactly the same "looks identical from EA-calc onward" property       //
// those two established.                                                          //
//                                                                            //
// flush: when ap040_execute.v detects a mispredicted branch, everything     //
// speculatively fetched behind it -- including whatever is sitting here -- //
// must be discarded. An outstanding L1 request (mem_pending) is simply       //
// abandoned, not unwound -- a read has no side effect, so its eventual        //
// l1_q_b is just never consumed once mem_pending clears.                     //
//                                                                            //
// JMP (An) / JMP (d16,An) (milestone 11, new): the EA (operand_a + eac_imm,   //
// the exact same expression l1_addr_b already computes) is what JMP needs     //
// as its RESULT, not an address to dereference -- eac_is_mem_src stays 0 for  //
// JMP (ap040_decode.v never sets it), so this stage's mem_issue/mem_complete   //
// FSM never triggers for it and JMP rides the plain, non-stalling "else"       //
// path below like any register-direct instruction. The only change there is    //
// eaf_operand_a: `eac_is_jmp ? (operand_a + eac_imm) : operand_a` -- routing     //
// the computed target into the SAME field ap040_execute.v already reads for      //
// its EX-forward tap and (via a new case there) ex_recovery_pc. See its           //
// header for why JMP is modeled as an unconditional misprediction rather than      //
// a new redirect mechanism.                                                        //
//                                                                            //
// BSR / JSR (milestone 13, new): this pipeline's first genuine memory WRITE,   //
// and its second kind of stall (mem_issue's read-side FSM has existed since     //
// milestone 9b; this is the write-side counterpart). Both push the return        //
// address (eac_next_pc -- already exactly right, no new arithmetic: it's the      //
// address of whatever comes after the whole BSR/JSR, extension words              //
// included, the same field id_next_pc has threaded since milestone 7) onto        //
// -(A7), then decrement A7 by 4. Both quantities the push needs -- the WRITE       //
// ADDRESS (A7-4) and the NEW A7 VALUE to commit -- are the SAME expression,         //
// and ap040_decode.v set eac_dest_reg to A7's unified index (4'd15) for both        //
// instructions specifically so operand_b (port B, "the destination register's       //
// current value", already resolved with full EX-forwarding) gives us A7's            //
// CURRENT value for free -- no new regfile read, no new forwarding mux. JSR's         //
// own EA target (An + eac_imm, via operand_a, exactly like JMP) and BSR/JSR's          //
// push address (operand_b - 4) are simply two DIFFERENT fields, resolved from           //
// two DIFFERENT ports, with no conflict.                                                //
//                                                                            //
//   l1_addr_b (extended): now selects between ea_target (a memory-source read     //
//   or JMP/JSR's redirect target) and push_addr (operand_b - 4) depending on        //
//   which this cycle's instruction actually is -- mutually exclusive by              //
//   construction, an instruction is never more than one of these at once.             //
//   l1_wren_b/l1_data_b: asserted whenever eac_is_bsr||eac_is_jsr, driving the          //
//   SAME push_addr and eac_next_pc as the data -- held stable (not gated by any          //
//   "have we issued yet" latch) for as long as eac_* itself stays stable, which           //
//   the stall below guarantees.                                                            //
//   wr_stall (`eac_valid && (eac_is_bsr||eac_is_jsr) && l1_wr_busy`): unlike a               //
//   read, a write needs no "wait for the response" phase -- ap040_pipe_l1.v's                //
//   OWN contract (see its header) is that asserting wren_b while wr_busy is LOW               //
//   always succeeds THIS cycle, so the only thing to wait for is the port being                //
//   free at all. When wr_stall is false (either this isn't a push, or it is and                 //
//   l1_wr_busy just read low), the push -- if any -- is being accepted THIS SAME                 //
//   cycle, and the instruction proceeds to EX in that SAME cycle carrying                         //
//   eaf_operand_b = operand_b - 4 as its result (ap040_execute.v routes this into                  //
//   the regfile commit for A7, exactly the same "dedicated combinational result,                    //
//   not the generic ALU" pattern DBcc's decrement and Scc's byte-merge already use;                  //
//   see its header for why the generic ALU wasn't used here either -- eaf_operand_a                   //
//   is busy carrying JSR's redirect target and can't also carry a "4" operand).                        //
//--------------------------------------------------------------------------//

module ap040_ea_fetch
#(
	parameter [31:0] PC_RESET = 32'h0000_0400,   // must match ap040_inst_fetch.v's -- see header
	parameter         L1_AW    = 12               // must match the ap040_pipe_l1.v instance's AW
)
(
	input             clk,
	input             nreset,
	input             ce,
	input             stall_in,   // EX cannot accept this cycle
	input             flush,      // EX detected a misprediction: force a bubble

	input             eac_valid,
	input      [31:0] eac_pc,
	input      [31:0] eac_next_pc,
	input       [3:0] eac_dest_reg,
	input       [3:0] eac_src_reg,
	input      [31:0] eac_imm,
	input       [5:0] eac_alu_op,
	input             eac_src_a_is_imm,
	input             eac_writes_reg,
	input             eac_writes_ccr,
	input             eac_is_branch,
	input             eac_is_scc,
	input             eac_is_dbcc,
	input             eac_is_mem_src,
	input             eac_is_jmp,
	input             eac_is_bsr,
	input             eac_is_jsr,
	input       [3:0] eac_cond,

	// regfile operand read ports (driven combinationally by this stage;
	// ap040_pipe_core.v wires raddr_a/b <-> rdata_a/b straight to the
	// ap040_pipe_regfile.v instance)
	output     [3:0]  raddr_a,
	input      [31:0] rdata_a,
	output     [3:0]  raddr_b,
	input      [31:0] rdata_b,

	// EX-forward source, see header comment
	input             ex_fwd_valid,
	input       [3:0] ex_fwd_dest,
	input      [31:0] ex_fwd_data,

	// ap040_pipe_l1.v port B -- read for a memory-source instruction or
	// JMP/JSR's redirect target; write for BSR/JSR's push -- see header.
	output [L1_AW-1:0] l1_addr_b,
	input        [31:0] l1_q_b,
	output              l1_wren_b,
	output       [31:0] l1_data_b,
	input               l1_wr_busy,

	output            eaf_stall,  // to EA-calc

	output reg        eaf_valid,
	output reg [31:0] eaf_pc,
	output reg [31:0] eaf_next_pc,
	output reg  [3:0] eaf_dest_reg,
	output reg [31:0] eaf_operand_a,
	output reg [31:0] eaf_operand_b,
	output reg  [5:0] eaf_alu_op,
	output reg        eaf_writes_reg,
	output reg        eaf_writes_ccr,
	output reg        eaf_is_branch,
	output reg        eaf_is_scc,
	output reg        eaf_is_dbcc,
	output reg        eaf_is_jmp,
	output reg        eaf_is_bsr,
	output reg        eaf_is_jsr,
	output reg  [3:0] eaf_cond
);

reg mem_pending;   // an L1 port-B request is in flight for the CURRENT eac_*
                    // instruction; its result becomes valid next cycle -- see
                    // header.

wire mem_issue    = eac_valid && eac_is_mem_src && !mem_pending;
wire mem_complete = mem_pending;
// BSR/JSR's push -- no "pending" latch needed, see header: a write either
// succeeds immediately (l1_wr_busy low) or must wait for the port, but
// never needs a separate multi-cycle completion phase the way a read does.
wire eac_is_push  = eac_is_bsr || eac_is_jsr;
wire wr_stall     = eac_valid && eac_is_push && l1_wr_busy;

assign eaf_stall = stall_in || mem_issue || wr_stall;
assign raddr_a    = eac_src_reg;
assign raddr_b    = eac_dest_reg;

wire fwd_a_from_ex = ex_fwd_valid && (ex_fwd_dest == eac_src_reg);
wire fwd_b_from_ex = ex_fwd_valid && (ex_fwd_dest == eac_dest_reg);

// Flat 3-way select on port A: immediate, else forwarded, else the
// regfile's own (possibly write-through-bypassed) read. eac_src_a_is_imm
// and fwd_a_from_ex are mutually exclusive in practice (MOVEQ never has a
// meaningful eac_src_reg), so this is one priority mux, not two stacked.
// For a memory-source instruction this IS the effective address (An's
// value, decode having set eac_src_reg to An's unified index) -- see header.
wire [31:0] operand_a = eac_src_a_is_imm ? eac_imm :
                         fwd_a_from_ex   ? ex_fwd_data :
                                           rdata_a;

// Port B has no immediate case -- it's always "the destination register's
// current value" -- so it's a flat 2-way select.
wire [31:0] operand_b = fwd_b_from_ex ? ex_fwd_data : rdata_b;

// The effective address: operand_a (An's value, resolved by the mux above)
// PLUS eac_imm (the sign-extended displacement for a (d16,An) form, or
// exactly 0 for the plain (An) form -- see ap040_decode.v's header for why
// id_imm is zeroed for the plain case). One formula for both EA modes and
// both consumers (a memory-source MOVE dereferences it below; JMP uses it
// directly as its result) -- adding a future EA mode that also produces an
// address+offset shape (indexed, etc.) needs no new logic here, just
// decode emitting the right eac_imm.
wire [31:0] ea_target = operand_a + eac_imm;

// BSR/JSR's push address AND the new A7 value to commit are the SAME
// expression, from operand_b (A7's current value via port B, decode having
// set eac_dest_reg to A7's unified index for both -- see header).
wire [31:0] push_addr = operand_b - 32'd4;

// Driven unconditionally, same "compute always, gate consumption" precedent
// as raddr_b -- harmless when neither eac_is_mem_src/eac_is_jmp nor
// eac_is_push is set, nothing reads l1_q_b or l1_wr_busy that cycle.
// PC_RESET-relative to match ap040_inst_fetch.v's own L1 addressing -- see
// header. Selects between a READ target (memory-source or JMP/JSR's
// redirect) and the PUSH address -- mutually exclusive by construction.
wire [31:0] l1_addr_word = eac_is_push ? push_addr : ea_target;
assign l1_addr_b = (l1_addr_word - PC_RESET) >> 1;
assign l1_wren_b = eac_valid && eac_is_push;
assign l1_data_b = eac_next_pc;

always @(posedge clk) begin
	if (!nreset) begin
		eaf_valid      <= 1'b0;
		eaf_pc         <= 32'h0;
		eaf_next_pc    <= 32'h0;
		eaf_dest_reg   <= 4'h0;
		eaf_operand_a  <= 32'h0;
		eaf_operand_b  <= 32'h0;
		eaf_alu_op     <= 6'h0;
		eaf_writes_reg <= 1'b0;
		eaf_writes_ccr <= 1'b0;
		eaf_is_branch  <= 1'b0;
		eaf_is_scc     <= 1'b0;
		eaf_is_dbcc    <= 1'b0;
		eaf_is_jmp     <= 1'b0;
		eaf_is_bsr     <= 1'b0;
		eaf_is_jsr     <= 1'b0;
		eaf_cond       <= 4'h0;
		mem_pending    <= 1'b0;
	end else if (ce) begin
		if (flush) begin
			eaf_valid   <= 1'b0;
			mem_pending <= 1'b0;
		end else if (!stall_in) begin
			if (mem_issue) begin
				eaf_valid   <= 1'b0;
				mem_pending <= 1'b1;
			end else if (mem_complete) begin
				eaf_valid      <= eac_valid;
				eaf_pc         <= eac_pc;
				eaf_next_pc    <= eac_next_pc;
				eaf_dest_reg   <= eac_dest_reg;
				eaf_operand_a  <= l1_q_b;
				eaf_operand_b  <= operand_b;
				eaf_alu_op     <= eac_alu_op;
				eaf_writes_reg <= eac_writes_reg;
				eaf_writes_ccr <= eac_writes_ccr;
				eaf_is_branch  <= eac_is_branch;
				eaf_is_scc     <= eac_is_scc;
				eaf_is_dbcc    <= eac_is_dbcc;
				eaf_is_jmp     <= 1'b0;   // a mem-source instruction is never also a JMP/BSR/JSR
				eaf_is_bsr     <= 1'b0;
				eaf_is_jsr     <= 1'b0;
				eaf_cond       <= eac_cond;
				mem_pending    <= 1'b0;
			end else if (wr_stall) begin
				// Waiting for l1_wr_busy to clear -- see header. eac_* stays
				// frozen (eaf_stall propagates backward), so this re-evaluates
				// identically next cycle with the SAME push request still
				// asserted, until the port is free.
				eaf_valid <= 1'b0;
			end else begin
				eaf_valid      <= eac_valid;
				eaf_pc         <= eac_pc;
				eaf_next_pc    <= eac_next_pc;
				eaf_dest_reg   <= eac_dest_reg;
				// JMP/JSR: route the computed EA itself, not the register
				// value alone -- see header. BSR has no source read at all
				// (operand_a is simply unused for it), so it falls through
				// to the same default every register-direct instruction
				// already used.
				eaf_operand_a  <= (eac_is_jmp || eac_is_jsr) ? ea_target : operand_a;
				// BSR/JSR: the NEW A7 value (== push_addr, the same
				// expression already used for the write address) -- see
				// header for why the decrement happens HERE, once, rather
				// than being recomputed in ap040_execute.v.
				eaf_operand_b  <= (eac_is_bsr || eac_is_jsr) ? push_addr : operand_b;
				eaf_alu_op     <= eac_alu_op;
				eaf_writes_reg <= eac_writes_reg;
				eaf_writes_ccr <= eac_writes_ccr;
				eaf_is_branch  <= eac_is_branch;
				eaf_is_scc     <= eac_is_scc;
				eaf_is_dbcc    <= eac_is_dbcc;
				eaf_is_jmp     <= eac_is_jmp;
				eaf_is_bsr     <= eac_is_bsr;
				eaf_is_jsr     <= eac_is_jsr;
				eaf_cond       <= eac_cond;
			end
		end
	end
end

endmodule

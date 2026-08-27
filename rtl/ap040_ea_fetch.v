//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 14: exceptions)       //
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
//                                                                            //
// TRAP #n / illegal instruction (milestone 14, new): this pipeline's first     //
// exception entry, and its first multi-beat memory sequence -- BSR/JSR's push    //
// was exactly one write; a format-$0 frame is TWO (SR:PC_hi, then PC_lo:         //
// FmtVec), followed by a THIRD access, a plain read, to fetch the handler          //
// address out of the vector table. A small state register (exc_ph, BEAT0/          //
// BEAT1/VECRD) sequences these three L1 accesses one at a time; eac_* stays          //
// frozen throughout via exc_stall feeding eaf_stall, the SAME freeze-on-stall         //
// mechanism every multi-cycle operation in this stage already uses (mem_issue/        //
// wr_stall above). Each beat reuses wr_stall's own accept-when-!l1_wr_busy idiom       //
// independently (retried every cycle it's needed, exactly like BSR/JSR's single        //
// write already does) rather than a new handshake shape.                                //
//                                                                            //
// Frame contents (format $0 only this milestone -- see ap040_decode.v's header    //
// for why address-error's format $2 is deferred to its own milestone):             //
//   SR word:  this pipeline has no real T1/T0/M/IPL state (sr_s/sr_m are            //
//             hardwired in ap040_pipe_core.v), so the system byte is a fixed        //
//             8'b0010_0000 (S=1, everything else 0, matching AP040_SR_RESET's        //
//             S=1/M=0) -- ccr_in supplies the real, live low byte (this stage's       //
//             CCR forwarding source, same signal ap040_execute.v's Bcc/Scc            //
//             condition check already uses -- see ap040_pipe_core.v).                  //
//   PC field: id_pc (the faulting instruction's OWN address) for illegal --             //
//             you can't "return past" an illegal opcode -- id_next_pc (the               //
//             FOLLOWING instruction, TRAP's actual return address) for TRAP,              //
//             the same subroutine-call semantics BSR/JSR's return-address push             //
//             already established. Verified against ap040_core.v's go_illegal              //
//             (spc=pc_i, its own convention for "current instruction's address")             //
//             vs. its vector-32+n TRAP dispatch (spc=pc, its convention for "the              //
//             next instruction" -- see that file for the pc/pc_i distinction).                //
//   Format/vector-offset word: {format(4)=0, 00, vector(8), 00} -- vector*4 falls               //
//             naturally out of this bit placement, no explicit shift needed.                     //
//                                                                            //
// Vector fetch: this substrate has no real VBR register (no MOVEC, no control        //
// registers at all yet) and, unlike a real 68040, no separate low-memory region       //
// distinct from ap040_pipe_l1.v's PC_RESET-relative window -- vector*4 is used         //
// as an ABSOLUTE address, fed through the exact SAME `l1_addr_word - PC_RESET) >>       //
// 1` conversion every other address on this port already goes through. This is          //
// not a special case: PC_RESET's own value (0x400 in every testbench so far,             //
// deliberately -- exactly the byte size of a full 256-entry 68k vector table) was          //
// already chosen with headroom for the vector table to occupy the address range              //
// BELOW it, and the unsigned wraparound in that subtraction lands vector*4 in the              //
// mathematically-correct modular slot of the SAME L1 array with zero new address               //
// logic -- confirmed arithmetically, not assumed, before relying on it.                          //
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
	input             eac_is_trap,
	input             eac_is_illegal,
	input       [3:0] eac_cond,

	// Architectural CCR, write-through forwarded -- see
	// ap040_pipe_core.v's ccr_resolved. Needed only to synthesize the SR
	// word an exception frame pushes -- see header.
	input       [4:0] ccr_in,

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
	output reg        eaf_is_trap,
	output reg        eaf_is_illegal,
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

// TRAP #n / illegal instruction exception entry -- see header. exc_ph
// sequences the frame's two writes and the vector-table read one at a time;
// exc_vec_pending mirrors mem_pending's own shape for the read's one-cycle
// latency.
localparam EXC_BEAT0 = 2'd0, EXC_BEAT1 = 2'd1, EXC_VECRD = 2'd2;
reg [1:0] exc_ph;
reg       exc_vec_pending;

wire eac_is_exc    = eac_is_trap || eac_is_illegal;
wire exc_active    = eac_valid && eac_is_exc;
wire exc_writing   = exc_active && !exc_vec_pending &&
                      (exc_ph == EXC_BEAT0 || exc_ph == EXC_BEAT1);
wire exc_beat_ack  = exc_writing && !l1_wr_busy;    // this beat accepted THIS cycle
wire exc_vec_issue = exc_active && !exc_vec_pending && (exc_ph == EXC_VECRD);
wire exc_vec_done  = exc_active && exc_vec_pending;
wire exc_stall     = exc_active && !exc_vec_done;

assign eaf_stall = stall_in || mem_issue || wr_stall || exc_stall;
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

// TRAP #n / illegal instruction: A7's NEW value after a format-$0 (8-byte)
// frame -- same "operand_b via port B" trick, decode having pointed
// eac_dest_reg at A7 for these too. Recomputed live off eac_/ccr_in every
// cycle of the sequence rather than latched once: eac_* is frozen by
// exc_stall the whole time anyway (nothing downstream can change it), so a
// latch would just be a redundant copy -- see header.
wire [31:0] exc_new_sp     = operand_b - 32'd8;
wire [15:0] exc_sr_word    = {8'b0010_0000, 3'b000, ccr_in};
wire [31:0] exc_pc_field   = eac_is_illegal ? eac_pc : eac_next_pc;
wire  [7:0] exc_vec_num    = eac_is_illegal ? 8'd4 : eac_imm[7:0];
wire [15:0] exc_vecoff_word = {4'd0, 2'b00, exc_vec_num, 2'b00};

// Beat0 @ exc_new_sp: SR, then PC's high word. Beat1 @ exc_new_sp+4: PC's
// low word, then the format/vector-offset word -- four 16-bit frame words
// packed into two 32-bit L1 writes, see header.
wire [31:0] exc_beat_addr = (exc_ph == EXC_BEAT1) ? (exc_new_sp + 32'd4) : exc_new_sp;
wire [31:0] exc_wdata     = (exc_ph == EXC_BEAT0)
                             ? {exc_sr_word, exc_pc_field[31:16]}
                             : {exc_pc_field[15:0], exc_vecoff_word};

// Vector table address: vector*4, used as an ABSOLUTE address fed through
// the SAME PC_RESET-relative conversion below -- see header for why no
// special-casing (a real VBR, a separate low-memory region) is needed.
wire [31:0] exc_vec_addr = {22'd0, exc_vec_num, 2'b00};

// Driven unconditionally, same "compute always, gate consumption" precedent
// as raddr_b -- harmless when none of eac_is_mem_src/eac_is_jmp/eac_is_push/
// eac_is_exc is set, nothing reads l1_q_b or l1_wr_busy that cycle.
// PC_RESET-relative to match ap040_inst_fetch.v's own L1 addressing -- see
// header. Four-way select: a READ target (memory-source or JMP/JSR's
// redirect), the BSR/JSR PUSH address, an exception frame WRITE beat, or the
// exception's own vector-table READ -- mutually exclusive by construction
// (an instruction is never more than one of these at once).
wire [31:0] l1_addr_word = eac_is_push  ? push_addr :
                            exc_writing ? exc_beat_addr :
                            (exc_vec_issue || exc_vec_pending) ? exc_vec_addr :
                                                                  ea_target;
assign l1_addr_b = (l1_addr_word - PC_RESET) >> 1;
assign l1_wren_b = (eac_valid && eac_is_push) || exc_writing;
assign l1_data_b = exc_writing ? exc_wdata : eac_next_pc;

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
		eaf_is_trap    <= 1'b0;
		eaf_is_illegal <= 1'b0;
		eaf_cond       <= 4'h0;
		mem_pending    <= 1'b0;
		exc_ph          <= EXC_BEAT0;
		exc_vec_pending <= 1'b0;
	end else if (ce) begin
		if (flush) begin
			eaf_valid       <= 1'b0;
			mem_pending     <= 1'b0;
			// Abandon a mid-flight exception sequence the same way an
			// abandoned mem_pending read is: nothing downstream of a flush
			// consumes what was in progress, but exc_ph/exc_vec_pending
			// MUST reset here too, or the next (unrelated) exception
			// instruction would resume mid-sequence instead of starting at
			// beat0.
			exc_ph          <= EXC_BEAT0;
			exc_vec_pending <= 1'b0;
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
				eaf_is_jmp     <= 1'b0;   // a mem-source instruction is never also a JMP/BSR/JSR/TRAP/illegal
				eaf_is_bsr     <= 1'b0;
				eaf_is_jsr     <= 1'b0;
				eaf_is_trap    <= 1'b0;
				eaf_is_illegal <= 1'b0;
				eaf_cond       <= eac_cond;
				mem_pending    <= 1'b0;
			end else if (wr_stall) begin
				// Waiting for l1_wr_busy to clear -- see header. eac_* stays
				// frozen (eaf_stall propagates backward), so this re-evaluates
				// identically next cycle with the SAME push request still
				// asserted, until the port is free.
				eaf_valid <= 1'b0;
			end else if (exc_writing) begin
				// Posting one beat of the exception frame -- see header.
				// exc_beat_ack means l1_wr_busy read low THIS cycle, so the
				// combinational wren_b above is being accepted right now;
				// advance to the next beat. Otherwise the port's still
				// busy: hold exc_ph, retry the SAME beat next cycle (eac_*
				// stays frozen via exc_stall, exactly like wr_stall above).
				eaf_valid <= 1'b0;
				if (exc_beat_ack)
					exc_ph <= (exc_ph == EXC_BEAT0) ? EXC_BEAT1 : EXC_VECRD;
			end else if (exc_vec_issue) begin
				// Both frame writes are posted; the vector-table address is
				// being driven THIS cycle (l1_addr_b, see the combinational
				// block above) -- l1_q_b registers its data by next cycle,
				// same one-cycle latency mem_issue/mem_complete already
				// relies on.
				eaf_valid       <= 1'b0;
				exc_vec_pending <= 1'b1;
			end else if (exc_vec_done) begin
				// l1_q_b now holds the handler address fetched last cycle.
				// eaf_operand_a carries it into ap040_execute.v's
				// ex_recovery_pc exactly like JMP/JSR's redirect target;
				// eaf_operand_b carries A7's new (post-push) value into the
				// regfile commit, exactly like BSR/JSR's decrement -- zero
				// new mux shapes in ap040_execute.v beyond adding these two
				// flags to existing OR-chains, see its header.
				eaf_valid      <= eac_valid;
				eaf_pc         <= eac_pc;
				eaf_next_pc    <= eac_next_pc;
				eaf_dest_reg   <= eac_dest_reg;
				eaf_operand_a  <= l1_q_b;
				eaf_operand_b  <= exc_new_sp;
				eaf_alu_op     <= eac_alu_op;
				eaf_writes_reg <= eac_writes_reg;
				eaf_writes_ccr <= eac_writes_ccr;
				eaf_is_branch  <= 1'b0;
				eaf_is_scc     <= 1'b0;
				eaf_is_dbcc    <= 1'b0;
				eaf_is_jmp     <= 1'b0;
				eaf_is_bsr     <= 1'b0;
				eaf_is_jsr     <= 1'b0;
				eaf_is_trap    <= eac_is_trap;
				eaf_is_illegal <= eac_is_illegal;
				eaf_cond       <= eac_cond;
				exc_ph          <= EXC_BEAT0;
				exc_vec_pending <= 1'b0;
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
				eaf_is_trap    <= eac_is_trap;
				eaf_is_illegal <= eac_is_illegal;
				eaf_cond       <= eac_cond;
			end
		end
	end
end

endmodule

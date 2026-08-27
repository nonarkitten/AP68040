//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 10: MOVE.L          //
// (d16,An),Dn)                                                             //
//                                                                          //
// ap040_decode.v - ID stage                                               //
//                                                                          //
// Recognizes NOP, MOVEQ, register-direct-to-register-direct MOVE.L,        //
// register-direct ADD.L Dn,Dm, register-direct Scc.B Dn, DBcc Dn,<label>,  //
// MOVE.L (An),Dn and MOVE.L (d16,An),Dn, and the whole Bcc family           //
// including its 16-/32-bit displacement forms (BRA included, as its        //
// "always true" special case -- see below). Anything else classifies as    //
// "unimplemented" and rides through the rest of the pipeline as a safe     //
// bubble (id_writes_reg stays 0) rather than an error; a later milestone   //
// replaces this with the real instruction-field decode extracted from      //
// ap040_core.v's S_DECODE.                                                 //
//                                                                          //
// MOVE.L (d16,An),Dn (milestone 10, new): a THIRD trigger onto the same     //
// gather state machine DBcc added a second one to -- (d16,An) always         //
// carries exactly one 16-bit extension word (the displacement), same        //
// word-form shape as DBcc, so held_is_move_disp/held_dest_reg join            //
// held_is_dbcc/held_reg rather than a parallel mechanism. Two things this     //
// mode needed that neither Bcc nor DBcc did:                                 //
//  - TWO different held registers, not one: dest (Dn) and src (An) are        //
//    different registers here, unlike DBcc where the loop counter is both.    //
//    held_dest_reg is new; held_reg (renamed in spirit, not in code, from      //
//    "DBcc's loop counter") now generically means "the extra register field    //
//    -- meaning depends on held_is_dbcc/held_is_move_disp".                    //
//  - This gather must NOT trigger IF's speculative redirect. Every prior        //
//    gather user (Bcc.W/L, DBcc) is something that MIGHT branch, so             //
//    redirect_from_gather fired unconditionally on any gather completion         //
//    through milestone 9. (d16,An)'s displacement is a MEMORY offset, not an     //
//    instruction address -- redirecting IF there would be a real, silent         //
//    correctness bug (confirmed by mutation-testing the guard back out; see       //
//    ap040_execute.v-adjacent test notes / AP040_IMPLEMENTATION_PLAN.md).          //
//    redirect_from_gather is now gated `&& !held_is_move_disp`.                    //
//                                                                          //
// The effective address itself needs no new machinery in ap040_ea_fetch.v:      //
// EA = An + displacement, and id_imm already exists as a general-purpose         //
// "extra 32-bit value" field (MOVEQ's immediate today) -- reused here to          //
// carry the sign-extended displacement (gather_disp, the SAME wire Bcc/DBcc       //
// already compute the identical way). ap040_ea_fetch.v's address computation       //
// becomes `operand_a + eac_imm` unconditionally for every memory-source            //
// instruction: for MOVE.L (An),Dn, eac_imm is 0 (see the id_imm fix below),         //
// so the addition is a no-op; for (d16,An) it's the real displacement. One          //
// formula, not a per-mode branch -- see ap040_ea_fetch.v's header.                  //
//                                                                          //
// id_imm fix (this milestone, affects the PLAIN (An) case too): the single-       //
// word decode branch previously set id_imm unconditionally to the sign-             //
// extended opcode LOW BYTE for every instruction, correct for MOVEQ and             //
// harmless-because-unused for everything else -- until ap040_ea_fetch.v             //
// started actually ADDING eac_imm to the address this milestone. MOVE.L             //
// (An),Dn's own opcode low byte (e.g. 0x10 for (A0),D0) would have been              //
// silently added to every plain-(An) address as a phantom displacement.              //
// Fixed by explicitly zeroing id_imm for is_move_mem_l in that branch; caught         //
// by re-running tb_ap040_pipe_move_mem.v (unaffected, since eac_imm is 0             //
// either way for its address -- gap closed by mutation-testing the fix back           //
// out, see the plan doc).                                                            //
//                                                                          //
// DBcc Dn,<label> (milestone 8, new): reuses the word-form gather this      //
// module already built for Bcc.W -- DBcc always carries exactly one 16-bit //
// extension word (the branch displacement), never byte or long forms, so   //
// it is added as a second trigger onto the SAME gather state machine        //
// (held_is_dbcc/held_reg alongside held_cond/held_pc), not a parallel one.  //
// Verified bit-for-bit against ap040_core.v's own decode                   //
// (ap040_core.v:5415-5421's `d_mode == 3'b001` -> DBcc arm of its 0x5       //
// ADDQ/SUBQ/Scc/DBcc group, immf(2'd1, S_DBCC1) confirming the single-word  //
// extension) and the same base-address convention as every other branch     //
// form (br_base <= pc, captured before the extension word is fetched --     //
// ap040_core.v:5419-5420 -- i.e. opcode_pc + 2, the formula id_redirect_pc  //
// already computes for the word-gather case, so DBcc needs no new base       //
// arithmetic here at all).                                                 //
//                                                                          //
// Decode still only speculatively redirects IF to the branch target,        //
// unconditionally assuming the loop continues -- the real decision (does    //
// the condition hold, and if not, does the decremented counter reach -1)    //
// needs a register value that isn't available until EA-fetch/EX, exactly    //
// the same timing argument milestone 5 made for Bcc's condition check; see  //
// ap040_execute.v's header for the decrement/compare logic and why DBcc      //
// reuses Bcc's mispredict/recovery wiring (ex_mispredict/ex_recovery_pc)     //
// completely unchanged -- both of DBcc's "don't branch" outcomes (condition //
// true, or counter expired) are, from IF's perspective, the identical         //
// fall-through-to-eaf_next_pc recovery Bcc already has.                     //
//                                                                          //
// The one thing DBcc needs that Bcc never did: it both reads AND writes Dn, //
// and unlike every register-writing instruction so far, whether it writes   //
// at all depends on a runtime value (the condition), not a static decode     //
// bit -- so id_writes_reg stays 0 here (like Bcc) and ap040_execute.v gates  //
// the real write dynamically off cond_result. id_dest_reg/id_src_reg are     //
// still threaded as Dn's own number (same field position as Scc's dest,      //
// if_opcode[2:0]) so EA-fetch reads Dn's current value into operand_a the     //
// normal way -- no new regfile port shape needed.                           //
//                                                                          //
// Multi-word gather (new this milestone): ap040_inst_fetch.v still hands   //
// over exactly one word per cycle, oblivious to instruction boundaries --  //
// deliberately NOT the real 68040's wide-prefetch-buffer approach (doc/    //
// The_68040_processor_I_Design_and_impleme.pdf describes an 8-word buffer  //
// and a 3-word decode window; disproportionate to what's needed here and   //
// not what section 16's "architecturally-equivalent, not cycle-exact"      //
// non-goal calls for). Instead, THIS stage recognizes when it needs more   //
// words than the one it's looking at, and spends the next 1-2 cycles       //
// treating ap040_inst_fetch.v's incoming words as extension DATA rather    //
// than fresh opcodes, emitting exactly one complete id_* entry once        //
// assembled. ext_pending/held_is_long/held_pc/held_cond/disp_acc are the    //
// gather state; none of it is visible outside this module -- once a        //
// Bcc.W/Bcc.L instruction is fully assembled it looks identical to Bcc.B    //
// from EA-calc onward, so nothing downstream needs to know gathering ever   //
// happened.                                                                //
//                                                                          //
// Two facts this relies on, verified against ap040_core.v's own            //
// implementation rather than assumed:                                      //
//  - the branch-displacement base is ALWAYS opcode_pc + 2 regardless of     //
//    form (ap040_core.v's br_base <= pc is captured at the same decode      //
//    instant for all three forms, before any extension word is fetched) --  //
//    so only the displacement's width/source changes between forms, not     //
//    the base formula Bcc.B already uses.                                   //
//  - word order for the 32-bit form is high-word-first: ap040_core.v:1817's //
//    imm <= {imm[15:0], epf_data[...]} shifts each new word in as the new   //
//    low half, so the first-fetched word becomes the high half once the     //
//    second arrives.                                                       //
//                                                                          //
// id_next_pc (new this milestone, every instruction not just branches):     //
// every instruction implemented before this one was exactly one word, so    //
// ap040_execute.v's fall-through/recovery address (eaf_pc + 2) was correct  //
// only by coincidence of scope. A 2-/3-word Bcc breaks that. The general    //
// fix -- needed for every future variable-length instruction, not just      //
// this one -- is for decode to compute the ACTUAL next-instruction address  //
// itself and thread it as its own field, rather than leave "+2" arithmetic  //
// at EX guessing at instruction length. id_pc stays "this instruction's     //
// own opcode address" (debug taps, the branch-target base above);           //
// id_next_pc is the new, separate "address of whatever comes after".        //
//                                                                          //
// flush during a gather (new correctness case): if a mispredict from an     //
// OLDER, already-in-flight branch arrives while this stage is mid-gather,   //
// the instruction being assembled is on the wrong path too (younger than    //
// the mispredicted branch) and must be abandoned, not just have its         //
// (already-0) id_valid re-zeroed -- flush resets ext_pending as well, so    //
// the next incoming word (from the recovery-redirected fetch stream) is     //
// correctly treated as a fresh opcode, not leftover extension data from a   //
// discarded instruction.                                                    //
//                                                                          //
// Bcc/BRA redirect: ap040_core.v itself treats BRA as nothing but the      //
// always-true case of the SAME Bcc mechanism -- ap040_core.v:4802-4819's   //
// 4'h6 decode calls one shared finish_bcc(target, cond_true(ir[11:8])) for //
// every condition code including T (ir[11:8]==0). This decoder does the    //
// same: id_redirect_valid/id_redirect_pc fire COMBINATIONALLY the instant   //
// the full displacement is known -- immediately, from if_opcode alone, for  //
// the byte form (same cycle it's fetched, before it's even latched into     //
// id_*); on the gather-completing cycle for the word/long forms -- for      //
// EVERY branch regardless of its actual condition, the "always assume       //
// taken" policy the real 68040 uses. For BRA (cond_true(0)==1 always) the   //
// guess is always right and nothing downstream ever needs to correct it.    //
// For a real Bcc, the guess can be wrong; id_is_branch/id_cond are threaded //
// onward (through ap040_ea_calc.v/ap040_ea_fetch.v unchanged) so            //
// ap040_execute.v can check the real condition once CCR forwarding actually //
// has something to forward (see its header comment for why EX, not here,   //
// is where that check has to happen) and signal a flush back through this  //
// stage's `flush` input if the guess was wrong.                            //
//                                                                          //
// Scc.B Dn shares id_cond's field position with Bcc (both put the 4-bit     //
// condition at if_opcode[11:8]), so it reuses id_cond directly -- no new    //
// condition field, and ap040_execute.v's cond_true() (all 16 encodings,     //
// not a subset) is reused by both consumers as-is. Scc's dest register is   //
// at if_opcode[2:0], NOT if_opcode[11:9] like MOVEQ/MOVE.L/ADD.L's dest --  //
// for Scc that upper field is the condition code instead -- so             //
// id_dest_reg's source is a per-opcode mux, the same shape id_alu_op        //
// already uses.                                                            //
//--------------------------------------------------------------------------//

`include "ap040_pipe_defs.svh"

module ap040_decode
(
	input             clk,
	input             nreset,
	input             ce,
	input             stall_in,   // EA-calc cannot accept this cycle
	input             flush,      // EX detected a misprediction: force a bubble

	input             if_valid,
	input      [31:0] if_pc,
	input      [15:0] if_opcode,

	output            id_stall,   // to IF: no local stall of its own yet

	// Bcc/BRA redirect, combinational -- see header comment
	output            id_redirect_valid,
	output     [31:0] id_redirect_pc,

	output reg        id_valid,
	output reg [31:0] id_pc,
	output reg [31:0] id_next_pc,
	output reg  [3:0] id_dest_reg,
	output reg  [3:0] id_src_reg,
	output reg [31:0] id_imm,
	output reg  [5:0] id_alu_op,
	output reg        id_src_a_is_imm,
	output reg        id_writes_reg,
	output reg        id_writes_ccr,
	output reg        id_is_branch,
	output reg        id_is_scc,
	output reg        id_is_dbcc,
	output reg        id_is_mem_src,
	output reg  [3:0] id_cond,
	output reg        id_unimpl
);

assign id_stall = stall_in;

// ap040_core.v:667-670 equivalents. All the opcodes decoded so far happen
// to share these same two field positions, except Scc (see header comment).
wire [2:0] d_reg9 = if_opcode[11:9];   // MOVEQ/MOVE.L/ADD.L dest Dn
wire [2:0] d_rn   = if_opcode[2:0];    // MOVE.L/ADD.L src Dn; Scc dest Dn

// MOVEQ: 0111 rrr 0 iiiiiiii  (rrr = dest Dn, iiiiiiii = 8-bit immediate)
wire is_moveq = (if_opcode[15:12] == 4'b0111) && (if_opcode[8] == 1'b0);

// MOVE.L Dn,Dm: 00 10 RRR 000 000 rrr (RRR = dest Dn, rrr = src Dn,
// dest/src EA modes both 000 = data-register-direct; no other EA mode yet).
// The size field (if_opcode[13:12]) is matched against the raw '10' = Long
// encoding rather than ap040_core.v's move_size decode (ap040_core.v:673-
// 674) because byte/word MOVE aren't implemented yet; switch to that
// convention when they are.
wire is_move_rr = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] == 2'b10) &&
                   (if_opcode[8:6]  == 3'b000) && (if_opcode[5:3]  == 3'b000);

// ADD.L Dn,Dm: 1101 RRR 0 10 000 rrr (RRR = dest Dm, rrr = src Dn; ir[8]=0
// selects the "<ea>+Dn->Dn" direction, ir[7:6]=std_size=10=Long, ir[5:3]=000
// = ea is Dn direct). Verified bit-for-bit against ap040_core.v's own
// SUB/ADD decode (ap040_core.v:4928-4974, d_op8_6/std_size at :668,:675),
// not guessed. No other size/direction/EA mode yet.
wire is_add_rr = (if_opcode[15:12] == 4'b1101) && (if_opcode[8] == 1'b0) &&
                  (if_opcode[7:6]  == 2'b10)    && (if_opcode[5:3] == 3'b000);

// Bcc family (BRA included as cc==0000; cc==0001 is BSR, excluded -- needs
// a stack push, not this milestone). The displacement byte selects which
// form: a real byte value is the short form (this cycle is a complete
// instruction); 8'h00 selects one 16-bit extension word; 8'hFF selects two
// (a 32-bit displacement, 68020+).
wire is_branch_opcode = (if_opcode[15:12] == 4'b0110) && (if_opcode[11:8] != 4'h1);
wire is_branch_byte   = is_branch_opcode && (if_opcode[7:0] != 8'h00) && (if_opcode[7:0] != 8'hFF);
wire is_branch_word   = is_branch_opcode && (if_opcode[7:0] == 8'h00);
wire is_branch_long   = is_branch_opcode && (if_opcode[7:0] == 8'hFF);

// Scc.B Dn: 0101 cccc 11 000 rrr (cccc = condition, mode 000 = data-
// register-direct dest at rrr). Verified against ap040_core.v:4745-4776's
// own ADDQ/SUBQ/Scc/DBcc decode: ir[7:6]==11 with mode 000 selects Scc
// (mode 001 is DBcc, mode 111+d_rn 010-100 is TRAPcc -- both excluded here
// by construction, not by explicit exclusion). No other destination EA
// mode yet -- register-direct only, same scope discipline as every other
// instruction so far.
wire is_scc_rr = (if_opcode[15:12] == 4'b0101) && (if_opcode[7:6] == 2'b11) &&
                  (if_opcode[5:3]  == 3'b000);

// DBcc Dn: 0101 cccc 11 001 rrr (cccc = condition, mode 001 = DBcc -- mode
// 000 is Scc above, mode 111+d_rn 010-100 is TRAPcc, both excluded here by
// construction). Verified against the same ap040_core.v:5415-5421 decode
// as the header comment. rrr (d_rn) is the loop-counter register, both read
// and written -- unlike Scc's dest, there is no separate "src" field.
wire is_dbcc = (if_opcode[15:12] == 4'b0101) && (if_opcode[7:6] == 2'b11) &&
               (if_opcode[5:3]  == 3'b001);

// MOVE.L (An),Dn: 0010 DDD 000 010 aaa (DDD = dest Dn at ir[11:9], aaa =
// source An at ir[2:0]). ir[15:12]==0010 alone already selects size=Long
// within the MOVE class (ir[15:14]=00 is fixed for MOVE, ir[13:12]=10=Long
// is baked into 4'b0010 -- no separate size check needed, unlike
// is_move_rr's raw-'10' note above). Verified against ap040_core.v's own
// MOVE decode (ap040_core.v:5021-5052): d_op8_6==000 selects DK_REG
// (register-direct dest) there, d_mode==010 selects its SK_MEM/(An) source
// -- rtl_old routes (An) through its fully general src_mode_r/src_rn_r EA
// machinery, which covers every addressing mode uniformly. This decoder
// special-cases JUST register-indirect for now, since ITS effective address
// needs no arithmetic at all: EA = An's value, resolved through the exact
// same regfile-port-A / EX-forward path register-direct source operands
// already use (id_src_reg = An's UNIFIED index, 8+n, same 4-bit space
// ap040_pipe_regfile.v already uses for A0-A6) -- see ap040_ea_fetch.v's
// header for the mechanism that turns that resolved value into an actual
// memory access rather than an ALU operand.
wire is_move_mem_l = (if_opcode[15:12] == 4'b0010) &&
                      (if_opcode[8:6]  == 3'b000) &&
                      (if_opcode[5:3]  == 3'b010);

// MOVE.L (d16,An),Dn: 0010 DDD 000 101 aaa -- same shape as is_move_mem_l
// above, mode field 101 instead of 010 (verified against the same
// ap040_core.v:5021-5052 MOVE decode -- d_mode==101 is SK_MEM there too,
// just a different EA mode within the same general machinery). Always
// carries exactly one 16-bit extension word (the displacement) -- routed
// through the shared gather state machine below, not a parallel one.
wire is_move_disp = (if_opcode[15:12] == 4'b0010) &&
                     (if_opcode[8:6]  == 3'b000) &&
                     (if_opcode[5:3]  == 3'b101);

wire is_nop = (if_opcode == `AP040_OP_NOP);

// Gather state -- see header comment. 0 = idle/normal decode cycle;
// 1 = this cycle's if_opcode completes the gather; 2 = one more word
// needed after this one (only ever set to 2 at gather start, long form).
// held_is_dbcc/held_reg (milestone 8) distinguish a DBcc gather from a
// Bcc.W gather sharing this same state machine; held_is_move_disp/
// held_dest_reg (milestone 10) add a third kind, MOVE.L (d16,An),Dn. held_reg
// is "the extra register field" generically -- An for move-disp, the loop
// counter for DBcc, meaningless when neither flag is set; held_dest_reg is
// move-disp-only (Dn), meaningless otherwise.
reg  [1:0]  ext_pending;
reg         held_is_long;
reg         held_is_dbcc;
reg         held_is_move_disp;
reg  [2:0]  held_reg;
reg  [2:0]  held_dest_reg;
reg  [31:0] held_pc;
reg  [3:0]  held_cond;
reg  [31:0] disp_acc;

wire completing_gather = (ext_pending == 2'd1);

// The full displacement as of the completing cycle: word form sign-extends
// if_opcode alone (nothing was usefully shifted into disp_acc for a 1-word
// gather); long form combines the word shifted in last cycle with this
// cycle's word, high-word-first (see header comment).
wire [31:0] gather_disp = held_is_long ? {disp_acc[15:0], if_opcode}
                                       : {{16{if_opcode[15]}}, if_opcode};

// Combinational redirect: fires immediately for the byte form (same cycle
// it's fetched, gated to only when not already mid-gather -- if_opcode
// during a gather cycle is data, not an opcode, and could coincidentally
// bit-match the byte-form pattern), or on the exact cycle a word/long
// gather completes -- EXCEPT a move-disp gather, whose displacement is a
// memory offset, not a branch target (see header comment).
wire redirect_from_byte   = if_valid && is_branch_byte && (ext_pending == 2'd0);
wire redirect_from_gather = completing_gather && !held_is_move_disp;

assign id_redirect_valid = redirect_from_byte || redirect_from_gather;
assign id_redirect_pc    = redirect_from_gather
                          ? (held_pc + 32'd2 + gather_disp)
                          : (if_pc   + 32'd2 + {{24{if_opcode[7]}}, if_opcode[7:0]});

always @(posedge clk) begin
	if (!nreset) begin
		id_valid        <= 1'b0;
		id_pc           <= 32'h0;
		id_next_pc      <= 32'h0;
		id_dest_reg     <= 4'h0;
		id_src_reg      <= 4'h0;
		id_imm          <= 32'h0;
		id_alu_op       <= 6'h0;
		id_src_a_is_imm <= 1'b0;
		id_writes_reg   <= 1'b0;
		id_writes_ccr   <= 1'b0;
		id_is_branch    <= 1'b0;
		id_is_scc       <= 1'b0;
		id_is_dbcc      <= 1'b0;
		id_is_mem_src   <= 1'b0;
		id_cond         <= 4'h0;
		id_unimpl       <= 1'b0;
		ext_pending     <= 2'd0;
		held_is_long    <= 1'b0;
		held_is_dbcc    <= 1'b0;
		held_is_move_disp <= 1'b0;
		held_reg        <= 3'h0;
		held_dest_reg   <= 3'h0;
		held_pc         <= 32'h0;
		held_cond       <= 4'h0;
		disp_acc        <= 32'h0;
	end else if (ce) begin
		if (flush) begin
			id_valid    <= 1'b0;
			ext_pending <= 2'd0;   // abandon any in-progress gather too
		end else if (!stall_in) begin
			if (ext_pending != 2'd0) begin
				// Gathering: if_opcode is extension-word data, never a
				// fresh opcode.
				disp_acc <= {disp_acc[15:0], if_opcode};
				if (completing_gather) begin
					id_valid        <= 1'b1;
					id_pc           <= held_pc;
					id_next_pc      <= held_pc + 32'd2 + (held_is_long ? 32'd4 : 32'd2);
					id_dest_reg     <= held_is_dbcc ? {1'b0, held_reg} :
					                    held_is_move_disp ? {1'b0, held_dest_reg} : 4'h0;
					id_src_reg      <= held_is_dbcc ? {1'b0, held_reg} :
					                    held_is_move_disp ? {1'b1, held_reg} : 4'h0;
					// gather_disp is already the sign-extended displacement
					// word (same wire Bcc/DBcc use for their target math) --
					// move-disp reuses it verbatim as id_imm, no new
					// arithmetic. Zero for every other gather kind, matching
					// the plain-(An) id_imm fix below (this field is a real
					// address offset now, not just MOVEQ's unused-elsewhere
					// immediate -- see header).
					id_imm          <= held_is_move_disp ? gather_disp : 32'h0;
					id_alu_op       <= `AP040_ALU_MOVE;
					id_src_a_is_imm <= 1'b0;
					id_writes_reg   <= held_is_move_disp;   // DBcc's write is dynamic -- see header
					id_writes_ccr   <= held_is_move_disp;
					id_is_branch    <= !held_is_dbcc && !held_is_move_disp;
					id_is_scc       <= 1'b0;
					id_is_dbcc      <= held_is_dbcc;
					id_is_mem_src   <= held_is_move_disp;
					id_cond         <= held_cond;
					id_unimpl       <= 1'b0;
					ext_pending     <= 2'd0;
				end else begin
					id_valid    <= 1'b0;
					ext_pending <= ext_pending - 2'd1;
				end
			end else if (is_branch_word || is_branch_long || is_dbcc || is_move_disp) begin
				// Opcode word of a word/long-form branch, a DBcc, or a
				// MOVE.L (d16,An),Dn (all word-form except long-branch): not
				// a complete instruction yet -- hold what we know, start
				// gathering.
				id_valid      <= 1'b0;
				held_pc       <= if_pc;
				held_cond     <= if_opcode[11:8];
				held_is_long  <= is_branch_long;
				held_is_dbcc  <= is_dbcc;
				held_is_move_disp <= is_move_disp;
				held_reg      <= if_opcode[2:0];
				held_dest_reg <= if_opcode[11:9];
				ext_pending   <= is_branch_long ? 2'd2 : 2'd1;
			end else begin
				id_valid        <= if_valid;
				id_pc           <= if_pc;
				id_next_pc      <= if_pc + 32'd2;
				id_dest_reg     <= is_scc_rr ? {1'b0, d_rn} : {1'b0, d_reg9};
				// is_move_mem_l's src_reg is An, not Dn -- the unified index's
				// top bit (8+n vs 0+n) is the ONLY thing that distinguishes
				// "read this register's value as an operand" (every other
				// instruction so far) from "read this register's value as an
				// ADDRESS" here; ap040_ea_fetch.v's existing operand_a mux
				// (regfile read + EX-forward) doesn't need to know the
				// difference, it just resolves whichever register this is.
				id_src_reg      <= is_move_mem_l ? {1'b1, d_rn} : {1'b0, d_rn};
				// Zeroed for is_move_mem_l (was the sign-extended opcode low
				// byte for EVERY instruction here, harmless until
				// ap040_ea_fetch.v started adding eac_imm to the address
				// this milestone -- see header). Still meaningful for MOVEQ.
				id_imm          <= is_move_mem_l ? 32'h0 : {{24{if_opcode[7]}}, if_opcode[7:0]};
				id_alu_op       <= is_add_rr ? `AP040_ALU_ADD : `AP040_ALU_MOVE;
				id_src_a_is_imm <= if_valid && is_moveq;
				id_writes_reg   <= if_valid && (is_moveq || is_move_rr || is_add_rr || is_scc_rr || is_move_mem_l);
				id_writes_ccr   <= if_valid && (is_moveq || is_move_rr || is_add_rr || is_move_mem_l);
				id_is_branch    <= if_valid && is_branch_byte;
				id_is_scc       <= if_valid && is_scc_rr;
				id_is_dbcc      <= 1'b0;
				id_is_mem_src   <= if_valid && is_move_mem_l;
				id_cond         <= if_opcode[11:8];
				id_unimpl       <= if_valid && !is_nop && !is_moveq && !is_move_rr && !is_add_rr && !is_branch_byte && !is_scc_rr && !is_move_mem_l;
			end
		end
	end
end

endmodule

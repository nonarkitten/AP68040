//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 4: BRA.B)           //
//                                                                          //
// ap040_inst_fetch.v - IF stage                                           //
//                                                                          //
// Still stands in for the real icache/bus port with a tiny inline ROM (all //
// NOPs by default): the goal so far is to prove the pipeline mechanism     //
// itself, not to wire up memory yet. A later milestone replaces the ROM    //
// with a request to ap040_cache.v / ap040_bus16_adapter.v.                 //
//                                                                          //
// Milestone 4 turns this from a pure linear-index walker into a real PC    //
// register: redirect_valid/redirect_pc (driven combinationally by          //
// ap040_decode.v the same cycle a branch is recognized -- see its header   //
// comment) select the next PC instead of a plain +2 sequential advance.    //
// The ROM is indexed by PC directly ((pc-PC_RESET)>>1) rather than a       //
// separate counter, so a redirect within the ROM's address range just      //
// works. "Stop after PROG_WORDS instructions" -- which every existing      //
// testbench's drain-check relies on -- is now tracked by a separate        //
// issued counter, decoupled from the PC value itself, so a program that    //
// branches still issues exactly PROG_WORDS instructions total rather than  //
// however many words a purely linear walk to PROG_WORDS would have         //
// covered.                                                                 //
//--------------------------------------------------------------------------//

`include "ap040_pipe_defs.svh"

module ap040_inst_fetch
#(
	parameter [31:0] PC_RESET   = 32'h0000_0400,
	parameter         PROG_WORDS = 10
)
(
	input             clk,
	input             nreset,
	input             ce,
	input             stall_in,     // ID cannot accept a new word this cycle

	input             redirect_valid,
	input      [31:0] redirect_pc,

	output reg        if_valid,
	output reg [31:0] if_pc,
	output reg [15:0] if_opcode
);

reg [15:0] rom [0:PROG_WORDS-1];
integer w;
initial for (w = 0; w < PROG_WORDS; w = w + 1) rom[w] = `AP040_OP_NOP;

reg [31:0] pc;                       // next word's address, absent a redirect
reg [31:0] issued;                   // instructions issued so far, 0..PROG_WORDS
wire       have_more = (issued < PROG_WORDS);

// The redirect must land on THIS fetch, not merely be scheduled for the
// following one -- otherwise the word at the old (sequential) pc still
// gets fetched first, one cycle late, which is exactly the "poison word
// executes anyway" bug this shape avoids. fetch_pc is what actually gets
// fetched and stored into if_pc/if_opcode this edge; pc (registered) only
// tracks the sequential fallback for cycles with no redirect.
wire [31:0] fetch_pc  = redirect_valid ? redirect_pc : pc;
wire [31:0] rom_idx   = (fetch_pc - PC_RESET) >> 1;   // assumes fetch_pc stays in-ROM

always @(posedge clk) begin
	if (!nreset) begin
		pc        <= PC_RESET;
		issued    <= 32'd0;
		if_valid  <= 1'b0;
		if_pc     <= PC_RESET;
		if_opcode <= 16'h0;
	end else if (ce && !stall_in) begin
		if_valid <= have_more;
		if (have_more) begin
			if_pc     <= fetch_pc;
			if_opcode <= rom[rom_idx];
			pc        <= fetch_pc + 32'd2;
			issued    <= issued + 32'd1;
		end
	end
end

endmodule

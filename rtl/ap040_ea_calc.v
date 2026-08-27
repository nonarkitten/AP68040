//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 11: JMP (An),       //
// JMP (d16,An))                                                            //
//                                                                          //
// ap040_ea_calc.v - EA-calc stage                                         //
//                                                                          //
// MOVEQ, register-direct MOVE.L/ADD.L, Bcc/BRA (any displacement width),   //
// Scc, DBcc, and JMP all have no memory effective address to compute, so   //
// this is still a pure pass-through -- it exists to occupy the pipeline-   //
// register slot a real address calculation will use once an EA mode that   //
// actually needs arithmetic is added, rather than being spliced in later   //
// and reshaping the pipeline. id_is_branch/id_is_scc/id_cond/              //
// id_writes_ccr/id_next_pc ride through unchanged for the same reason --   //
// this stage has nothing to compute for any of them; only                  //
// ap040_execute.v actually evaluates a condition or merges a byte, per its //
// header comment. id_next_pc is just one more field in the same            //
// pass-through shape.                                                     //
//                                                                          //
// MOVE.L (An),Dn (milestone 9b) and JMP (An)/(d16,An) (milestone 11) are    //
// ALSO pure pass-throughs here, deliberately: their effective addresses     //
// are An's raw value (plus id_imm's displacement for the (d16,An) forms),   //
// no arithmetic -- id_is_mem_src/id_is_jmp thread through unchanged, same   //
// shape as every other flag above. This is why the EA-calc/EA-fetch split   //
// exists as two stages already (per the original plan): the day an EA mode  //
// needs real arithmetic (indexed modes, etc.), it lands HERE, and EA-       //
// fetch's memory-access/stall logic (see its header) doesn't need to        //
// change at all -- it already just consumes whatever address EA-calc        //
// resolved.                                                                 //
//                                                                          //
// flush: when ap040_execute.v detects a mispredicted branch, everything     //
// speculatively fetched behind it -- including whatever is sitting here -- //
// must be discarded. Same shape as stall_in but forces a bubble instead     //
// of holding.                                                              //
//--------------------------------------------------------------------------//

module ap040_ea_calc
(
	input             clk,
	input             nreset,
	input             ce,
	input             stall_in,   // EA-fetch cannot accept this cycle
	input             flush,      // EX detected a misprediction: force a bubble

	input             id_valid,
	input      [31:0] id_pc,
	input      [31:0] id_next_pc,
	input       [3:0] id_dest_reg,
	input       [3:0] id_src_reg,
	input      [31:0] id_imm,
	input       [5:0] id_alu_op,
	input             id_src_a_is_imm,
	input             id_writes_reg,
	input             id_writes_ccr,
	input             id_is_branch,
	input             id_is_scc,
	input             id_is_dbcc,
	input             id_is_mem_src,
	input             id_is_jmp,
	input       [3:0] id_cond,

	output            ea_stall,   // to ID: no local stall of its own yet

	output reg        eac_valid,
	output reg [31:0] eac_pc,
	output reg [31:0] eac_next_pc,
	output reg  [3:0] eac_dest_reg,
	output reg  [3:0] eac_src_reg,
	output reg [31:0] eac_imm,
	output reg  [5:0] eac_alu_op,
	output reg        eac_src_a_is_imm,
	output reg        eac_writes_reg,
	output reg        eac_writes_ccr,
	output reg        eac_is_branch,
	output reg        eac_is_scc,
	output reg        eac_is_dbcc,
	output reg        eac_is_mem_src,
	output reg        eac_is_jmp,
	output reg  [3:0] eac_cond
);

assign ea_stall = stall_in;

always @(posedge clk) begin
	if (!nreset) begin
		eac_valid        <= 1'b0;
		eac_pc           <= 32'h0;
		eac_next_pc      <= 32'h0;
		eac_dest_reg     <= 4'h0;
		eac_src_reg      <= 4'h0;
		eac_imm          <= 32'h0;
		eac_alu_op       <= 6'h0;
		eac_src_a_is_imm <= 1'b0;
		eac_writes_reg   <= 1'b0;
		eac_writes_ccr   <= 1'b0;
		eac_is_branch    <= 1'b0;
		eac_is_scc       <= 1'b0;
		eac_is_dbcc      <= 1'b0;
		eac_is_mem_src   <= 1'b0;
		eac_is_jmp       <= 1'b0;
		eac_cond         <= 4'h0;
	end else if (ce) begin
		if (flush) begin
			eac_valid <= 1'b0;
		end else if (!stall_in) begin
			eac_valid        <= id_valid;
			eac_pc           <= id_pc;
			eac_next_pc      <= id_next_pc;
			eac_dest_reg     <= id_dest_reg;
			eac_src_reg      <= id_src_reg;
			eac_imm          <= id_imm;
			eac_alu_op       <= id_alu_op;
			eac_src_a_is_imm <= id_src_a_is_imm;
			eac_writes_reg   <= id_writes_reg;
			eac_writes_ccr   <= id_writes_ccr;
			eac_is_branch    <= id_is_branch;
			eac_is_scc       <= id_is_scc;
			eac_is_dbcc      <= id_is_dbcc;
			eac_is_mem_src   <= id_is_mem_src;
			eac_is_jmp       <= id_is_jmp;
			eac_cond         <= id_cond;
		end
	end
end

endmodule

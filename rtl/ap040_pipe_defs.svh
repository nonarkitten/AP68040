//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 6: folder           //
// independence + Scc.B)                                                    //
//                                                                          //
// ap040_pipe_defs.svh - constants shared by the ap040_pipe_* modules.      //
//                                                                          //
// Fully self-contained as of this milestone: earlier milestones            //
// `include`d rtl/ap040/ap040_defs.svh directly; the user decided           //
// rtl/ap040/ should stay completely untouched and rtl/ap040_pipe/ should   //
// be independently deletable, so the constants ap040_pipe_alu.v/           //
// ap040_pipe_regfile.v/ap040_decode.v/ap040_execute.v actually need are    //
// copied in below instead of shared. Values match rtl/ap040/ap040_defs.svh //
// exactly (same architectural encodings -- MC68040 opcode/size fields      //
// don't change), but this is a separate copy, not a shared definition.     //
//--------------------------------------------------------------------------//

`ifndef AP040_PIPE_DEFS_SVH
`define AP040_PIPE_DEFS_SVH

// Internal transfer sizes
`define AP040_SZ_B        2'd0
`define AP040_SZ_W        2'd1
`define AP040_SZ_L        2'd2

// The only opcode literal ap040_decode.v/ap040_inst_fetch.v match directly
// rather than by field decode.
`define AP040_OP_NOP      16'h4E71

// ALU operations. ap040_pipe_alu.v's case statement needs every entry to
// compile even though only MOVE/ADD are driven by any decoder yet.
`define AP040_ALU_MOVE    6'd0
`define AP040_ALU_ADD     6'd1
`define AP040_ALU_ADDX    6'd2
`define AP040_ALU_SUB     6'd3
`define AP040_ALU_SUBX    6'd4
`define AP040_ALU_CMP     6'd5
`define AP040_ALU_AND     6'd6
`define AP040_ALU_OR      6'd7
`define AP040_ALU_EOR     6'd8
`define AP040_ALU_NOT     6'd9
`define AP040_ALU_NEG     6'd10
`define AP040_ALU_NEGX    6'd11
`define AP040_ALU_CLR     6'd12
`define AP040_ALU_TST     6'd13
`define AP040_ALU_EXT     6'd14
`define AP040_ALU_EXTB    6'd15
`define AP040_ALU_SWAP    6'd16
`define AP040_ALU_TAS     6'd17
`define AP040_ALU_ABCD    6'd18
`define AP040_ALU_SBCD    6'd19
`define AP040_ALU_NBCD    6'd20
`define AP040_ALU_ASL1    6'd21
`define AP040_ALU_ASR1    6'd22
`define AP040_ALU_LSL1    6'd23
`define AP040_ALU_LSR1    6'd24
`define AP040_ALU_ROL1    6'd25
`define AP040_ALU_ROR1    6'd26
`define AP040_ALU_ROXL1   6'd27
`define AP040_ALU_ROXR1   6'd28
`define AP040_ALU_BTST    6'd29
`define AP040_ALU_BCHG    6'd30
`define AP040_ALU_BCLR    6'd31
`define AP040_ALU_BSET    6'd32

`endif // AP040_PIPE_DEFS_SVH

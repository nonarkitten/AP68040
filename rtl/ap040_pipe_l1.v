//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 9b: MOVE.L (An),Dn) //
//                                                                          //
// ap040_pipe_l1.v - unified instruction/data L1, backing-store-less        //
//                                                                          //
// A true dual-port memory (same port shape/naming convention as            //
// rtl_old/primitives/dpram.v -- forked, not shared, per folder             //
// independence: see ap040_pipe_core.v's header), used as BOTH instruction  //
// and data storage. Port A is ap040_inst_fetch.v's read (unchanged since    //
// milestone 9a). Port B (milestone 9b, new) is ap040_ea_fetch.v's read for  //
// the first memory-referencing instruction, MOVE.L (An),Dn -- see its       //
// header for the stall/timing mechanism that actually consumes it.          //
//                                                                          //
// Port B is 32-bit, not 16-bit like port A: this core's internal memory     //
// interface is meant to be 32-bit-native (the original plan's section 1: a  //
// clean 32-bit request/response internally, with any 16-bit-bus splitting   //
// left to a future host adapter, not built into the core itself).           //
// address_b addresses the HIGH word of a longword; the low word is          //
// implicitly address_b+1, big-endian/high-word-first, the same word order   //
// ap040_decode.v's Bcc.L gather and rtl_old's bus16 adapter both already     //
// use. wren_b/data_b are wired for a future store instruction (MOVE.L       //
// Dn,(An) etc.) -- unused by milestone 9b's read-only MOVE.L (An),Dn, tied   //
// off at the ap040_pipe_core.v instantiation -- so the port shape doesn't    //
// need reshaping again when a store instruction lands.                      //
//                                                                          //
// Calling this a "cache" is aspirational today: there is no larger backing //
// store behind it yet for this repo to miss into (no SDRAM/DDR model the   //
// way the Minimig-AGA tree this core was extracted from has), so right now //
// this module simply IS the memory, sized generously (AW=12 -> 4096 words) //
// rather than to any real cache-line/set budget. What DOES carry over      //
// architecturally, and is the actual point of unifying it now rather than  //
// giving each future memory-referencing instruction its own private test   //
// stub: a write through port B is visible to a port-A read on any LATER    //
// cycle for free, just from both ports addressing the same `mem` array --  //
// no explicit invalidation path is needed for self-modifying code to work  //
// correctly, the way a split Harvard I/D cache would need one. Tags,       //
// per-line valid bits, replacement, and a real miss/fill path to an actual //
// backing store are deferred until this repo has a backing store worth     //
// missing into; the port shape below (one address/data/write-enable per    //
// side, registered read) is chosen so that adding those later doesn't      //
// require reshaping the ports IF/EA-fetch already depend on.               //
//                                                                          //
// One real semantic gap this shortcut creates, worth remembering rather    //
// than rediscovering: CINV/CPUSH exist on real hardware because a split    //
// I/D cache can go stale and needs explicit maintenance; on THIS substrate //
// there is no staleness to invalidate, so those instructions can only ever //
// be decoded/accepted as legal no-ops here, never exercised for their      //
// actual observable effect (stale-until-flush) the way rtl_old's t_cache   //
// suite does against the split cache in rtl_old/ap040_cache.v. That's an   //
// accepted trade for this pipeline (non-goal: architecturally-equivalent,  //
// not a transistor-level replica -- AP040_IMPLEMENTATION_PLAN.md section   //
// 2), not an oversight to fix later.                                      //
//                                                                          //
// Default fill is AP040_OP_NOP, not zero: every existing pipe testbench    //
// pokes a handful of program words via hierarchical reference             //
// (`dut.u_l1.mem[N] = ...`, moved here from `dut.u_if.rom[N]` when this    //
// milestone lifted the array out of ap040_inst_fetch.v) and relies on      //
// every OTHER word fetched within the program's PROG_WORDS budget          //
// draining as a harmless NOP. Nothing currently reads port B as data, so   //
// this default has no data-side consequence yet; revisit if/when a test    //
// cares about a genuine zero-fill default for data.                       //
//                                                                          //
// Timing note (why this is a drop-in, not a stall-infrastructure change):  //
// ap040_inst_fetch.v's own array read, before this milestone, was already  //
// a registered (1-cycle-latency) read -- functionally identical to this    //
// module's `q_a <= mem[address_a]`. Moving the storage out to a shared      //
// module and having IF drive address_a combinationally (see its own        //
// header) reproduces the EXACT same address-to-data latency IF already     //
// had, so this is a like-for-like swap: every existing pipe testbench       //
// passes unmodified in behavior (only the hierarchical poke path changed). //
// The real timing decision -- combinational vs. a genuine stalled read for //
// port B once EA-fetch actually consumes it -- is still open; see          //
// AP040_IMPLEMENTATION_PLAN.md section 6.                                  //
//--------------------------------------------------------------------------//

`include "ap040_pipe_defs.svh"

module ap040_pipe_l1
#(
	parameter AW = 12,   // word address width -> 2**AW words of storage
	parameter DW = 16    // word width
)
(
	input                clock,

	input      [AW-1:0]  address_a,
	input      [DW-1:0]  data_a,
	input                wren_a,
	output reg [DW-1:0]  q_a,

	// port B: 32-bit, addresses the HIGH word; low word is address_b+1 -- see
	// header comment.
	input      [AW-1:0]  address_b,
	input       [31:0]   data_b,
	input                wren_b,
	output reg  [31:0]   q_b
);

reg [DW-1:0] mem [0:(1<<AW)-1];

integer i;
initial for (i = 0; i < (1<<AW); i = i + 1) mem[i] = `AP040_OP_NOP;

wire [AW-1:0] address_b_lo = address_b + {{(AW-1){1'b0}}, 1'b1};

always @(posedge clock) begin
	if (wren_a) mem[address_a] <= data_a;
	if (wren_b) begin
		mem[address_b]    <= data_b[31:16];
		mem[address_b_lo] <= data_b[15:0];
	end
	q_a <= mem[address_a];
	q_b <= {mem[address_b], mem[address_b_lo]};
end

endmodule

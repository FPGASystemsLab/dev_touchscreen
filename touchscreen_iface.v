//=============================================================================
// Main contributors
//  - Jakub  Siast        <mailto:jsiast@multimedia.edu.pl>
//  - Maciej Kurc         <mailto:mkurc@multimedia.edu.pl>
//=============================================================================
`default_nettype none
`timescale 1 ns / 1 ps
//-----------------------------------------------------------------------------
module touchscreen_iface
(
// Clocking
input wire              CLK,
input wire              RST,

// Input
input  wire             TS_STB,     // Do modulu touchscreen_box
output wire             TS_ACK,
input  wire [11:0]      TS_DAT_X,
input  wire [11:0]      TS_DAT_Y,

// Output
output wire             O_STB,
input  wire             O_ACK,
output wire [31:0]      O_DAT       // 0000 XXXX XXXX XXXX 0000 YYYY YYYY YYYY
);
//-----------------------------------------------------------------------------
// Variables
reg             ts_ack_;
//-----------------------------------------------------------------------------
reg             o_stb_;
reg  [31:0]     o_dat_;
//-----------------------------------------------------------------------------
// Input
// ts_ack
always @(negedge CLK or posedge RST)
    if(RST)         ts_ack_ <= 0;
    else            ts_ack_ <= TS_STB;
//-----------------------------------------------------------------------------
// Output
assign TS_ACK = ts_ack_;
//-----------------------------------------------------------------------------
// Output
// o_stb
always @(posedge CLK or posedge RST)
    if(RST)                     o_stb_ <= 0;
    else if(         TS_STB)    o_stb_ <= 1;
    else if( o_stb_ & O_ACK)    o_stb_ <= 0;
//-----------------------------------------------------------------------------
// o_dat
always @(posedge CLK or posedge RST)
    if(RST)                     o_dat_ <= 0;
    else if(TS_STB)             o_dat_ <= {4'd8, TS_DAT_X, 4'd0, TS_DAT_Y};
//-----------------------------------------------------------------------------
// Output
assign O_STB = o_stb_;
assign O_DAT = o_dat_;
//-----------------------------------------------------------------------------
endmodule

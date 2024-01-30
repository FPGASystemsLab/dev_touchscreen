//=============================================================================
// Main contributors
//  - Jakub  Siast        <mailto:jsiast@multimedia.edu.pl>
//=============================================================================
`default_nettype none
//---------------------------------------------------------------------------------------------
`timescale 1ns / 1ns                            
//============================================================================================================================
module touchscreen_box #
(
// PRESCALER SETTING:
// In the TS controller chip specification, the acquisition time is:
// Tacq = 8.4 * (Rin + 100 Ohm) * 37 pF
// where 100 Ohm and 37 pF are the input parameters of the controller/converter,
// and Rin is the impedance of the screen connector. For Rin = 0, the system can operate with a clock up to 2MHz
// The actual screen has a maximum resistance value of 1300 Ohms.
// This is 13x more than the 100 Ohms input of the controller, so the system should operate
// with a clock at least 14 times slower. To be sure, the clock is set to
// 20 times slower: 2Mhz / 20 = 100kHz.
// PRESCALER = CLK / 100kHz = 100MHz / 100kHz = 1000
parameter PRESCALER = 1000, // SPI Prescaler (>= 2)
parameter INTERVAL = 5000 // The system can operate every this many cycles
)
(                             
 input  wire            clk,
 input  wire            rst, 
                                               
 input  wire            i_stb,
 input  wire    [14:0]  i_addr,
 input  wire    [31:0]  i_data,
 output wire            i_ack,
                                                       
 output wire            o_stb,                     
 output wire            o_wr,
 output wire    [14:0]  o_addr,
 output wire    [31:0]  o_data,
 output wire    [ 3:0]  o_mask,
 input  wire            o_ack,
                               
// Device (AD7843)
output wire             D_SCK,
output wire             D_DO,
input  wire             D_DI,
output wire             D_CS,
input  wire             D_BSY,      // Ten jest nie uzywany !!
input  wire             D_IRQ,

output wire [7:0]       DBG0,
output wire [7:0]       DBG1
); 
//===============================================================================
// Dummy soc side input   
//===============================================================================
assign i_ack = i_stb;
//===============================================================================
// Touchscreen physical interface   
//===============================================================================
wire             s0_stb;     // Do modulu touchscreen_box
wire             s0_ack;
wire [11:0]      s0_dat_x;
wire [11:0]      s0_dat_y;
wire             s0_irq;
//=============================================================================== 
touchscreen_phy #
(
.PRESCALER (PRESCALER), // SPI Prescaler (>= 2)
.INTERVAL (INTERVAL) // The system can operate every this many cycles
)
(
// Clocking
.CLK (clk),
.RST (rst),

// Input
.I_RST (1'b0), // Controller Reset (Actually, this is not needed)
.I_CONV (s0_irq), // Conversion command (connect to O_IRQ)
.I_ACK (),
.I_BSY (),

// Output
.O_STB (s0_stb),
.O_ACK (s0_ack),
.O_DAT_X (s0_dat_x), // X Position
.O_DAT_Y (s0_dat_y), // Y Position
.O_IRQ (s0_irq), // Interrupt (independent of O_STB, active 1)

// Device (AD7843)
.D_SCK (D_SCK),
.D_DO (D_DO),
.D_DI (D_DI),
.D_CS (D_CS),
.D_BSY (D_BSY), // This one is not used!!
.D_IRQ (D_IRQ),

.DBG0 (DBG0),
.DBG1 (DBG1)
);
//===============================================================================
// Data formating and output   
//===============================================================================
touchscreen_iface 
(
// Clocking
.CLK         (clk),
.RST         (rst),

// Input
.TS_STB      (s0_stb),
.TS_ACK      (s0_ack),
.TS_DAT_X    (s0_dat_x), // position X
.TS_DAT_Y    (s0_dat_y), // position Y

// Output
.O_STB       (o_stb),
.O_ACK       (o_ack),
.O_DAT       (o_data)          // 8000 XXXX XXXX XXXX 0000 YYYY YYYY YYYY
);
//-------------------------------------------------------------------------------              
assign  o_wr   = 1'b1;
assign  o_addr = 15'd0;
assign  o_mask = 4'hF;
//===============================================================================
endmodule
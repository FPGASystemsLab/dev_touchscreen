//=============================================================================
// Main contributors
//  - Maciej Kurc         <mailto:mkurc@multimedia.edu.pl>
//  - Jakub  Siast        <mailto:jsiast@multimedia.edu.pl>
//=============================================================================
`default_nettype none
`timescale 1 ns / 1 ps

//-----------------------------------------------------------------------------
module touchscreen_phy #
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
// Clocking
input wire              CLK,
input wire              RST,

// Input
input  wire             I_RST,      // Reset kontrolera (W zasadzie to nie jest potrzebny)
input  wire             I_CONV,     // Polecenie konwersji (podpiac do O_IRQ)
output wire             I_ACK,
output wire             I_BSY,

// Output
output wire             O_STB,
input  wire                O_ACK,
output wire [11:0]      O_DAT_X,    // Pozycja X
output wire [11:0]      O_DAT_Y,    // Pozycja Y
output wire             O_IRQ,      // Przerwanie (niezaleznie od O_STB, aktywne 1)

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

//-----------------------------------------------------------------------------
// FSM
localparam FSM_IDLE     = 8'd10;

localparam FSM_SYNC0x   = 8'd20;
localparam FSM_CONV0x   = 8'd21;
localparam FSM_CONV1x   = 8'd22;
localparam FSM_CONV2x   = 8'd23;
localparam FSM_SYNC1x   = 8'd24;

localparam FSM_SYNC0y   = 8'd30;
localparam FSM_CONV0y   = 8'd31;
localparam FSM_CONV1y   = 8'd32;
localparam FSM_CONV2y   = 8'd33;
localparam FSM_SYNC1y   = 8'd34;

localparam FSM_SYNC0r   = 8'd40;
localparam FSM_RESET0   = 8'd41;
localparam FSM_RESET1   = 8'd42;
localparam FSM_RESET2   = 8'd43;
localparam FSM_SYNC1r   = 8'd44;

localparam FSM_OUTa     = 8'd50;
localparam FSM_OUTb     = 8'd51;

localparam FSM_WAIT     = 8'd60;

localparam FSM_TRX      = 8'd99;

localparam FSM_BIT7a    = 8'd100;
localparam FSM_BIT7b    = 8'd101;
localparam FSM_BIT6a    = 8'd102;
localparam FSM_BIT6b    = 8'd103;
localparam FSM_BIT5a    = 8'd104;
localparam FSM_BIT5b    = 8'd105;
localparam FSM_BIT4a    = 8'd106;
localparam FSM_BIT4b    = 8'd107;
localparam FSM_BIT3a    = 8'd108;
localparam FSM_BIT3b    = 8'd109;
localparam FSM_BIT2a    = 8'd110;
localparam FSM_BIT2b    = 8'd111;
localparam FSM_BIT1a    = 8'd112;
localparam FSM_BIT1b    = 8'd113;
localparam FSM_BIT0a    = 8'd114;
localparam FSM_BIT0b    = 8'd115;

integer fsm_ctl;
integer fsm_spi;

assign DBG0 = fsm_ctl[7:0];
assign DBG1 = fsm_spi[7:0];
//-----------------------------------------------------------------------------
// Signals
reg         i_ack_;
reg         i_bsy_;

reg         o_stb_;
reg [11:0]  o_dat_x_;
reg [11:0]  o_dat_y_;

reg [31:0]  wait_cnt;
wire        wait_end;

reg         d_di_x;
reg         d_di_y;
reg         d_di_;

reg         d_bsy_x;
reg         d_bsy_y;
reg         d_bsy_;

reg         d_irq_x;
reg         d_irq_y;
reg         d_irq_;

reg         d_cs_;
reg         d_sck_;
reg         d_do_;

reg         spi_i_bsy;
reg         spi_i_stb;
reg [7:0]   spi_i_dat;

reg [15:0]  spi_prescaler;
reg         spi_tick;
reg [7:0]   spi_dat;

//-----------------------------------------------------------------------------
// Control FSM

// fsm_ctl
always @(posedge CLK or posedge RST)
    if(RST)                                               fsm_ctl <= FSM_SYNC0r;
        
    else if((fsm_ctl == FSM_IDLE  )&& ( I_RST & ~I_CONV)) fsm_ctl <= FSM_SYNC0r;
    else if((fsm_ctl == FSM_IDLE  )&& (~I_RST &  I_CONV)) fsm_ctl <= FSM_SYNC0x;
        
    else if((fsm_ctl == FSM_SYNC0r)&& (spi_tick)  )       fsm_ctl <= FSM_RESET0;
    else if((fsm_ctl == FSM_RESET0)&& (~spi_i_bsy))       fsm_ctl <= FSM_RESET1;
    else if((fsm_ctl == FSM_RESET1)&& (~spi_i_bsy))       fsm_ctl <= FSM_RESET2;
    else if((fsm_ctl == FSM_RESET2)&& (~spi_i_bsy))       fsm_ctl <= FSM_SYNC1r;
    else if((fsm_ctl == FSM_SYNC1r)&& (spi_tick)  )       fsm_ctl <= FSM_IDLE;

    else if((fsm_ctl == FSM_SYNC0x)&& (spi_tick)  )       fsm_ctl <= FSM_CONV0x;
    else if((fsm_ctl == FSM_CONV0x)&& (~spi_i_bsy))       fsm_ctl <= FSM_CONV1x;
    else if((fsm_ctl == FSM_CONV1x)&& (~spi_i_bsy))       fsm_ctl <= FSM_CONV2x;
    else if((fsm_ctl == FSM_CONV2x)&& (~spi_i_bsy))       fsm_ctl <= FSM_SYNC1x;
    else if((fsm_ctl == FSM_SYNC1x)&& (spi_tick)  )       fsm_ctl <= FSM_SYNC0y;

    else if((fsm_ctl == FSM_SYNC0y)&& (spi_tick)  )       fsm_ctl <= FSM_CONV0y;
    else if((fsm_ctl == FSM_CONV0y)&& (~spi_i_bsy))       fsm_ctl <= FSM_CONV1y;
    else if((fsm_ctl == FSM_CONV1y)&& (~spi_i_bsy))       fsm_ctl <= FSM_CONV2y;
    else if((fsm_ctl == FSM_CONV2y)&& (~spi_i_bsy))       fsm_ctl <= FSM_SYNC1y;
    else if((fsm_ctl == FSM_SYNC1y)&& (spi_tick)  )       fsm_ctl <= FSM_OUTa;
        
    else if((fsm_ctl == FSM_OUTa  )               )       fsm_ctl <= FSM_OUTb;
    else if((fsm_ctl == FSM_OUTb  )&& O_ACK       )       fsm_ctl <= FSM_WAIT;
        
    else if((fsm_ctl == FSM_WAIT  )&& wait_end    )       fsm_ctl <= FSM_IDLE;
    else                                                  fsm_ctl <= fsm_ctl;

//-----------------------------------------------------------------------------
// Waiting

// wait_cnt
always @(posedge CLK or posedge RST)
    if(RST)                                             wait_cnt <= 0;
    else if((fsm_ctl == FSM_IDLE)&&(I_RST | I_CONV))    wait_cnt <= (INTERVAL - 1);
    else if(wait_cnt == 0                          )    wait_cnt <=  wait_cnt;
     else                                               wait_cnt <=  wait_cnt - 1;
    
// wait_end
assign wait_end = (wait_cnt == 0);
    
//-----------------------------------------------------------------------------
// Input handshake

// i_ack
always @(negedge CLK or posedge RST)
    if(RST)                                             i_ack_ <= 1'b0;
    else if((fsm_ctl == FSM_IDLE)&& (I_RST | I_CONV))   i_ack_ <= 1'b1;
    else                                                i_ack_ <= 1'b0;

// i_bsy
always @(negedge CLK or posedge RST)    
    if(RST)                                             i_bsy_ <= 1'b0;
    else if((fsm_ctl == FSM_IDLE)&& (I_RST | I_CONV))   i_bsy_ <= 1'b1;
    else if((fsm_ctl == FSM_SYNC1r)&& spi_tick      )   i_bsy_ <= 1'b0;
    else if((fsm_ctl == FSM_WAIT  )&& wait_end      )   i_bsy_ <= 1'b0;
    else                                                i_bsy_ <= i_bsy_;
    
// Output
assign I_ACK = i_ack_;
assign I_BSY = i_bsy_;

//-----------------------------------------------------------------------------
// Output handshake & data

// o_stb
always @(posedge CLK or posedge RST)
    if(RST)                                           o_stb_   <= 1'b0;
    else if((fsm_ctl == FSM_OUTa)         )           o_stb_   <= 1'b1;
    else if((fsm_ctl == FSM_OUTb) && O_ACK)           o_stb_   <= 1'b0;
    else                                              o_stb_   <= o_stb_;
    
// o_dat_x
always @(posedge CLK or posedge RST)
    if(RST)                                           o_dat_x_ <= 12'd0;
    else if((fsm_ctl == FSM_CONV1x)&& (~spi_i_bsy))   o_dat_x_ <= {spi_dat[7:0]  , 4'd0};
    else if((fsm_ctl == FSM_CONV2x)&& (~spi_i_bsy))   o_dat_x_ <= {o_dat_x_[11:4], spi_dat[7:4]};
    else                                              o_dat_x_ <= o_dat_x_;
    
// o_dat_y
always @(posedge CLK or posedge RST)
    if(RST)                                           o_dat_y_ <= 12'd0;
    else if((fsm_ctl == FSM_CONV1y)&& (~spi_i_bsy))   o_dat_y_ <= {spi_dat[7:0]  , 4'd0};
    else if((fsm_ctl == FSM_CONV2y)&& (~spi_i_bsy))   o_dat_y_ <= {o_dat_y_[11:4], spi_dat[7:4]};
    else                                              o_dat_y_ <= o_dat_y_;

// Output
assign O_STB    = o_stb_;
assign O_DAT_X  = o_dat_x_;
assign O_DAT_Y  = o_dat_y_;
    
//-----------------------------------------------------------------------------
// Device

// d_cs
always @(posedge CLK or posedge RST)
    if(RST)                                      d_cs_ <= 1'b1;
    else if((fsm_ctl == FSM_SYNC0r) && spi_tick) d_cs_ <= 1'b0;
    else if((fsm_ctl == FSM_SYNC0x) && spi_tick) d_cs_ <= 1'b0;
    else if((fsm_ctl == FSM_SYNC0y) && spi_tick) d_cs_ <= 1'b0;
    else if((fsm_ctl == FSM_SYNC1r) && spi_tick) d_cs_ <= 1'b1;
    else if((fsm_ctl == FSM_SYNC1x) && spi_tick) d_cs_ <= 1'b1;
    else if((fsm_ctl == FSM_SYNC1y) && spi_tick) d_cs_ <= 1'b1;
    else                                         d_cs_ <= d_cs_;
    
// Output
assign D_CS = d_cs_;
    
//-----------------------------------------------------------------------------
// SPI Control
// spi_i_stb
always @(posedge CLK or posedge RST)
    if(RST)                                          spi_i_stb <= 1'b0;
    else if((fsm_ctl == FSM_SYNC0r) && (spi_tick  )) spi_i_stb <= 1'b1;
    else if((fsm_ctl == FSM_RESET0) && (~spi_i_bsy)) spi_i_stb <= 1'b1;
    else if((fsm_ctl == FSM_RESET1) && (~spi_i_bsy)) spi_i_stb <= 1'b1;

    else if((fsm_ctl == FSM_SYNC0x) && (spi_tick  )) spi_i_stb <= 1'b1;
    else if((fsm_ctl == FSM_CONV0x) && (~spi_i_bsy)) spi_i_stb <= 1'b1;
    else if((fsm_ctl == FSM_CONV1x) && (~spi_i_bsy)) spi_i_stb <= 1'b1;

    else if((fsm_ctl == FSM_SYNC0y) && (spi_tick  )) spi_i_stb <= 1'b1;
    else if((fsm_ctl == FSM_CONV0y) && (~spi_i_bsy)) spi_i_stb <= 1'b1;
    else if((fsm_ctl == FSM_CONV1y) && (~spi_i_bsy)) spi_i_stb <= 1'b1;
    
    else                                             spi_i_stb <= 1'b0;
    
// spi_i_dat
always @(posedge CLK or posedge RST)
    if(RST)                                          spi_i_dat <= 8'd0;
     
    else if((fsm_ctl == FSM_SYNC0r) && (spi_tick)  ) spi_i_dat <= 8'b11100000;   // Control byte
    else if((fsm_ctl == FSM_RESET0) && (~spi_i_bsy)) spi_i_dat <= 8'b0;
    else if((fsm_ctl == FSM_RESET1) && (~spi_i_bsy)) spi_i_dat <= 8'b0;

    else if((fsm_ctl == FSM_SYNC0x) && (spi_tick)  ) spi_i_dat <= 8'b10010000;   // Control byte (X)
    else if((fsm_ctl == FSM_CONV0x) && (~spi_i_bsy)) spi_i_dat <= 8'b0;
    else if((fsm_ctl == FSM_CONV1x) && (~spi_i_bsy)) spi_i_dat <= 8'b0;

    else if((fsm_ctl == FSM_SYNC0y) && (spi_tick)  ) spi_i_dat <= 8'b11010000;   // Control byte (Y)
    else if((fsm_ctl == FSM_CONV0y) && (~spi_i_bsy)) spi_i_dat <= 8'b0;
    else if((fsm_ctl == FSM_CONV1y) && (~spi_i_bsy)) spi_i_dat <= 8'b0;
     
     else                                            spi_i_dat <= spi_i_dat;   
    
//-----------------------------------------------------------------------------
// Input buffer (3x reg)
always @(posedge CLK or posedge RST)    d_di_x  <= (RST) ? 0 : D_DI;
always @(posedge CLK or posedge RST)    d_di_y  <= (RST) ? 0 : d_di_x;
always @(posedge CLK or posedge RST)    d_di_   <= (RST) ? 0 : d_di_y;

always @(posedge CLK or posedge RST)    d_bsy_x <= (RST) ? 1 : D_BSY;
always @(posedge CLK or posedge RST)    d_bsy_y <= (RST) ? 1 : d_bsy_x;
always @(posedge CLK or posedge RST)    d_bsy_  <= (RST) ? 1 : d_bsy_y;

always @(posedge CLK or posedge RST)    d_irq_x <= (RST) ? 1 : D_IRQ;
always @(posedge CLK or posedge RST)    d_irq_y <= (RST) ? 1 : d_irq_x;
always @(posedge CLK or posedge RST)    d_irq_  <= (RST) ? 1 : d_irq_y;

// IRQ
assign O_IRQ = ~d_irq_;
    
//-----------------------------------------------------------------------------
// SPI Prescaler

// spi_prescaler
always @(posedge CLK or posedge RST)
    if(RST)                                          spi_prescaler <= (PRESCALER/2-1);
     else if ( spi_prescaler == 16'd0           )    spi_prescaler <= (PRESCALER/2-1);
    else                                             spi_prescaler <= (spi_prescaler-1);

// spi_tick
always @(posedge CLK or posedge RST)
    if(RST)                                          spi_tick <= 1'b0;
    else if ( spi_prescaler == 16'd1            )    spi_tick <= 1'b1;
    else                                             spi_tick <= 1'b0;

//-----------------------------------------------------------------------------
// SPI FSM

// fsm_spi
always @(posedge CLK or posedge RST)
    if(RST) fsm_spi <= FSM_IDLE;
        
    else if ((fsm_spi == FSM_IDLE ) && (spi_i_stb))  fsm_spi <= FSM_TRX;
    
    else if ((fsm_spi == FSM_TRX  ) && (spi_tick))   fsm_spi <= FSM_BIT7a;
    else if ((fsm_spi == FSM_BIT7a) && (spi_tick))   fsm_spi <= FSM_BIT7b;
    else if ((fsm_spi == FSM_BIT7b) && (spi_tick))   fsm_spi <= FSM_BIT6a;
    else if ((fsm_spi == FSM_BIT6a) && (spi_tick))   fsm_spi <= FSM_BIT6b;
    else if ((fsm_spi == FSM_BIT6b) && (spi_tick))   fsm_spi <= FSM_BIT5a;
    else if ((fsm_spi == FSM_BIT5a) && (spi_tick))   fsm_spi <= FSM_BIT5b;
    else if ((fsm_spi == FSM_BIT5b) && (spi_tick))   fsm_spi <= FSM_BIT4a;
    else if ((fsm_spi == FSM_BIT4a) && (spi_tick))   fsm_spi <= FSM_BIT4b;
    else if ((fsm_spi == FSM_BIT4b) && (spi_tick))   fsm_spi <= FSM_BIT3a;
    else if ((fsm_spi == FSM_BIT3a) && (spi_tick))   fsm_spi <= FSM_BIT3b;
    else if ((fsm_spi == FSM_BIT3b) && (spi_tick))   fsm_spi <= FSM_BIT2a;
    else if ((fsm_spi == FSM_BIT2a) && (spi_tick))   fsm_spi <= FSM_BIT2b;
    else if ((fsm_spi == FSM_BIT2b) && (spi_tick))   fsm_spi <= FSM_BIT1a;
    else if ((fsm_spi == FSM_BIT1a) && (spi_tick))   fsm_spi <= FSM_BIT1b;
    else if ((fsm_spi == FSM_BIT1b) && (spi_tick))   fsm_spi <= FSM_BIT0a;
    else if ((fsm_spi == FSM_BIT0a) && (spi_tick))   fsm_spi <= FSM_BIT0b;
    else if ((fsm_spi == FSM_BIT0b) && (spi_tick))   fsm_spi <= FSM_IDLE;
    else                                             fsm_spi <= fsm_spi;
        
//-----------------------------------------------------------------------------
// SPI

// spi_i_bsy
wire spi_dat_enter = spi_i_stb && (fsm_spi == FSM_IDLE);
wire spi_dat_leave = spi_tick  && (fsm_spi == FSM_BIT0b);
always @(negedge CLK or posedge RST)
    if(RST)                             spi_i_bsy <= 0;
     else if(spi_dat_enter)             spi_i_bsy <= 1;
     else if(spi_dat_leave)             spi_i_bsy <= 0;
     else                               spi_i_bsy <= spi_i_bsy;

// spi_dat
wire spi_dat_shift =    spi_tick &&
                        ((fsm_spi == FSM_BIT7a)|
                         (fsm_spi == FSM_BIT6a)|
                         (fsm_spi == FSM_BIT5a)|
                         (fsm_spi == FSM_BIT4a)|
                         (fsm_spi == FSM_BIT3a)|
                         (fsm_spi == FSM_BIT2a)|
                         (fsm_spi == FSM_BIT1a)|
                         (fsm_spi == FSM_BIT0a));
                         
always @(posedge CLK or posedge RST)
    if(RST)                             spi_dat <= 8'd0;
     else if (spi_dat_enter)            spi_dat <= spi_i_dat[7:0];
     else if (spi_dat_shift)            spi_dat <={spi_dat[6:0], d_di_};
     else                               spi_dat <= spi_dat[7:0];

//-----------------------------------------------------------------------------
// SPI Output

// d_sck
always @(posedge CLK or posedge RST)
    if(RST)                                        d_sck_ <= 1'b0;
        
    else if ((fsm_spi == FSM_BIT7a) && (spi_tick)) d_sck_ <= 1'b1;
    else if ((fsm_spi == FSM_BIT7b) && (spi_tick)) d_sck_ <= 1'b0;
    else if ((fsm_spi == FSM_BIT6a) && (spi_tick)) d_sck_ <= 1'b1;
    else if ((fsm_spi == FSM_BIT6b) && (spi_tick)) d_sck_ <= 1'b0;
    else if ((fsm_spi == FSM_BIT5a) && (spi_tick)) d_sck_ <= 1'b1;
    else if ((fsm_spi == FSM_BIT5b) && (spi_tick)) d_sck_ <= 1'b0;
    else if ((fsm_spi == FSM_BIT4a) && (spi_tick)) d_sck_ <= 1'b1;
    else if ((fsm_spi == FSM_BIT4b) && (spi_tick)) d_sck_ <= 1'b0;
    else if ((fsm_spi == FSM_BIT3a) && (spi_tick)) d_sck_ <= 1'b1;
    else if ((fsm_spi == FSM_BIT3b) && (spi_tick)) d_sck_ <= 1'b0;
    else if ((fsm_spi == FSM_BIT2a) && (spi_tick)) d_sck_ <= 1'b1;
    else if ((fsm_spi == FSM_BIT2b) && (spi_tick)) d_sck_ <= 1'b0;
    else if ((fsm_spi == FSM_BIT1a) && (spi_tick)) d_sck_ <= 1'b1;
    else if ((fsm_spi == FSM_BIT1b) && (spi_tick)) d_sck_ <= 1'b0;
    else if ((fsm_spi == FSM_BIT0a) && (spi_tick)) d_sck_ <= 1'b1;
    else if ((fsm_spi == FSM_BIT0b) && (spi_tick)) d_sck_ <= 1'b0;
    else                                           d_sck_ <= d_sck_;
        
// d_do
wire d_do_change =      spi_tick &&
                        ((fsm_spi == FSM_TRX)|
                         (fsm_spi == FSM_BIT7b)|
                         (fsm_spi == FSM_BIT6b)|
                         (fsm_spi == FSM_BIT5b)|
                         (fsm_spi == FSM_BIT4b)|
                         (fsm_spi == FSM_BIT3b)|
                         (fsm_spi == FSM_BIT2b)|
                         (fsm_spi == FSM_BIT1b));
                         
always @(posedge CLK or posedge RST)
    if(RST)                         d_do_ <= 0;
     else   if(d_do_change)         d_do_ <= spi_dat[7];
     else                           d_do_ <= d_do_;
     
// Output
assign D_SCK    = d_sck_;
assign D_DO     = d_do_;
    
endmodule
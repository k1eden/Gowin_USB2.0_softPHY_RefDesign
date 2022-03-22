
// ===========Oooo==========================================Oooo========
// =  Copyright (C) 2014-2020 Gowin Semiconductor Technology Co.,Ltd.
// =                     All rights reserved.
// =====================================================================
//
//  __      __      __
//  \ \    /  \    / /   [File name   ] prim_sim.v
//   \ \  / /\ \  / /    [Description ] GW1N hardcore verilog functional simulation library
//    \ \/ /  \ \/ /     [Timestamp   ] Wed May 27 11:00:30 2020
//     \  /    \  /      [version     ] 1.2
//      \/      \/       
//
// ===========Oooo==========================================Oooo========


`timescale 1ns / 1ps

//GSR
module GSR (GSRI);

input GSRI;
wire GSRO;

assign GSRO = GSRI;

endmodule //GSR (global set/reset control)

//BANDGAP
module BANDGAP( BGEN );
input BGEN;

endmodule

//CLKDIV2
module CLKDIV2(CLKOUT, HCLKIN, RESETN);

parameter GSREN = "false"; //"false", "true"

input HCLKIN, RESETN;
output CLKOUT;

reg reset_0;
reg clk_div2;
wire grstn;

initial begin
    clk_div2 = 1'b0;
    reset_0 = 1'b0;
end

assign grstn = GSREN == "true" ? GSR.GSRO : 1'b1;

always @(posedge HCLKIN or negedge grstn or negedge RESETN) begin
    if (!grstn) begin
        reset_0 <= 1'b0;
    end else if (!RESETN) begin
        reset_0 <= 1'b0;
    end else begin
        reset_0 <= 1'b0;
    end
end

always @(posedge HCLKIN or negedge grstn or negedge RESETN) begin
    if (!grstn) begin
        clk_div2 <= 1'b0;
    end else if (!RESETN) begin
        clk_div2 <= 1'b0;
    end else begin
        clk_div2 <= (clk_div2 ^ (~reset_0));
    end
end

assign CLKOUT = clk_div2;

endmodule


//DCC,DutyCycleCorrection
module DCC(CLKOUT, CLKIN);
output CLKOUT;
input CLKIN;

parameter DCC_EN = 1'b1; //1'b1: enable dcc; 1'b0: disable dcc
parameter FCLKIN = 50.0;//frequency of the clkin(M)

reg dcc_out;
realtime clk_period_h;

initial begin
    dcc_out = 1'b0;
    clk_period_h = (1000/FCLKIN/2);
end

assign CLKOUT = (DCC_EN == 1'b1) ? dcc_out : CLKIN;

always @(posedge CLKIN) 
begin
    dcc_out <= 1'b1;
    #(clk_period_h - 0.06)
    dcc_out <= 1'b0;
end


endmodule

//DHCENC
module DHCENC (CLKOUT, CLKOUTN, CLKIN, CE);
input CLKIN, CE;
output CLKOUT, CLKOUTN;

reg ce_reg0,ce_reg1,ce_reg2,ce_reg3;

always @(negedge CLKIN)
begin
    ce_reg0 <= ~CE;    
    ce_reg1 <= ce_reg0;
    ce_reg2 <= ce_reg1;
    ce_reg3 <= ce_reg2;
end

assign CLKOUT = CLKIN & ce_reg3;
assign CLKOUTN = ~CLKOUT;

endmodule


//EMCU,Enhanced MCU,used for 1ns-4c
module EMCU (

  input            FCLK,                      // Free running clock
  input            PORESETN,                  // Power on reset
  input            SYSRESETN,                 // System reset
  input            RTCSRCCLK,                 // Used to generate RTC clock
  output  [15:0]   IOEXPOUTPUTO,              // 
  output  [15:0]   IOEXPOUTPUTENO,            // 
  input   [15:0]   IOEXPINPUTI,               // 
  output           UART0TXDO,                 // 
  output           UART1TXDO,                 // 
  output           UART0BAUDTICK,             //
  output           UART1BAUDTICK,             //
  input            UART0RXDI,                 // 
  input            UART1RXDI,                 //
  output           INTMONITOR,                //
  // ----------------------------------------------------------------------------
  // AHB2SRAM<0..3> Interfaces
  // ----------------------------------------------------------------------------
  output             MTXHRESETN,            // SRAM/Flash Chip reset
  output  [12:0]     SRAM0ADDR,             // SRAM address
  output  [3:0]      SRAM0WREN,             // SRAM Byte write enable
  output  [31:0]     SRAM0WDATA,            // SRAM Write data
  output             SRAM0CS,               // SRAM Chip select
  input   [31:0]     SRAM0RDATA,            // SRAM Read data bus
  // ----------------------------------------------------------------------------
  // AHB Target Expansion ports
  // ----------------------------------------------------------------------------
  //  amba.com/AMBA3/AHBLite/r2p0_0, TARGFLASH0, master
  output              TARGFLASH0HSEL,         // TARGFLASH0, HSELx,
  output  [28:0]      TARGFLASH0HADDR,        // TARGFLASH0, HADDR,
  output   [1:0]      TARGFLASH0HTRANS,       // TARGFLASH0, HTRANS,
  output   [2:0]      TARGFLASH0HSIZE,        // TARGFLASH0, HSIZE,
  output   [2:0]      TARGFLASH0HBURST,       // TARGFLASH0, HBURST,
  output              TARGFLASH0HREADYMUX,    // TARGFLASH0, HREADYOUT,
  input   [31:0]      TARGFLASH0HRDATA,       // TARGFLASH0, HRDATA,
  input    [2:0]      TARGFLASH0HRUSER,       // TARGFLASH0, HRUSER, Tie to 3'b0
  input               TARGFLASH0HRESP,        // TARGFLASH0, HRESP
  input               TARGFLASH0EXRESP,       // TARGFLASH0, EXRESP;  Tie to 1'b0
  input               TARGFLASH0HREADYOUT,    // TARGFLASH0, EXRESP
  //  amba.com/AMBA3/AHBLite/r2p0_0, TARGEXP0, master
  output              TARGEXP0HSEL,           // TARGEXP0, HSELx,
  output  [31:0]      TARGEXP0HADDR,          // TARGEXP0, HADDR,
  output   [1:0]      TARGEXP0HTRANS,         // TARGEXP0, HTRANS,
  output              TARGEXP0HWRITE,         // TARGEXP0, HWRITE,
  output   [2:0]      TARGEXP0HSIZE,          // TARGEXP0, HSIZE,
  output   [2:0]      TARGEXP0HBURST,         // TARGEXP0, HBURST,
  output   [3:0]      TARGEXP0HPROT,          // TARGEXP0, HPROT,
  output   [1:0]      TARGEXP0MEMATTR,
  output              TARGEXP0EXREQ,
  output   [3:0]      TARGEXP0HMASTER,
  output  [31:0]      TARGEXP0HWDATA,         // TARGEXP0, HWDATA,
  output              TARGEXP0HMASTLOCK,      // TARGEXP0, HMASTLOCK,
  output              TARGEXP0HREADYMUX,      // TARGEXP0, HREADYOUT,
  output              TARGEXP0HAUSER,         // TARGEXP0, HAUSER,
  output   [3:0]      TARGEXP0HWUSER,         // TARGEXP0, HWUSER,
  input   [31:0]      TARGEXP0HRDATA,         // TARGEXP0, HRDATA,
  input               TARGEXP0HREADYOUT,      // TARGEXP0, HREADY,
  input               TARGEXP0HRESP,          // TARGEXP0, HRESP,
  input               TARGEXP0EXRESP,
  input    [2:0]      TARGEXP0HRUSER,         // TARGEXP0, HRUSER,
  // ----------------------------------------------------------------------------
  // AHB Initiator Expansion ports
  // ----------------------------------------------------------------------------
  output  [31:0]      INITEXP0HRDATA,         // INITEXP0, HRDATA,
  output              INITEXP0HREADY,         // INITEXP0, HREADY,
  output              INITEXP0HRESP,          // INITEXP0, HRESP,
  output              INITEXP0EXRESP,
  output  [2:0]       INITEXP0HRUSER,         // INITEXP0, HRUSER,
  input               INITEXP0HSEL,           // INITEXP0, HSELx,
  input   [31:0]      INITEXP0HADDR,          // INITEXP0, HADDR,
  input    [1:0]      INITEXP0HTRANS,         // INITEXP0, HTRANS,
  input               INITEXP0HWRITE,         // INITEXP0, HWRITE,
  input    [2:0]      INITEXP0HSIZE,          // INITEXP0, HSIZE,
  input    [2:0]      INITEXP0HBURST,         // INITEXP0, HBURST,
  input    [3:0]      INITEXP0HPROT,          // INITEXP0, HPROT,
  input    [1:0]      INITEXP0MEMATTR,
  input               INITEXP0EXREQ,
  input    [3:0]      INITEXP0HMASTER,
  input   [31:0]      INITEXP0HWDATA,         // INITEXP0, HWDATA,
  input               INITEXP0HMASTLOCK,      // INITEXP0, HMASTLOCK,
  input               INITEXP0HAUSER,         // INITEXP0, HAUSER,
  input    [3:0]      INITEXP0HWUSER,         // INITEXP0, HWUSER,
  //  amba.com/AMBA4/APB4/r0p0_0, APBTARGEXP2, master
  output  [3:0]       APBTARGEXP2PSTRB,       // APBTARGEXP2, PSTRB,
  output  [2:0]       APBTARGEXP2PPROT,       // APBTARGEXP2, PPROT,
  output              APBTARGEXP2PSEL,        // APBTARGEXP2, PSELx,
  output              APBTARGEXP2PENABLE,     // APBTARGEXP2, PENABLE,
  output  [11:0]      APBTARGEXP2PADDR,       // APBTARGEXP2, PADDR,
  output              APBTARGEXP2PWRITE,      // APBTARGEXP2, PWRITE,
  output  [31:0]      APBTARGEXP2PWDATA,      // APBTARGEXP2, PWDATA,
  input    [31:0]     APBTARGEXP2PRDATA,      // APBTARGEXP2, PRDATA,
  input               APBTARGEXP2PREADY,      // APBTARGEXP2, PREADY,
  input               APBTARGEXP2PSLVERR,     // APBTARGEXP2, PSLVERR,
  // CPU Control, Status and Configuration
  // ----------------------------------------------------------------------------
  // CSS configuration inputs
  // ----------------------------------------------------------------------------
  input   [3:0]       MTXREMAP,               // Configration bits. MTXREMAP = 4'b1111
  // ----------------------------------------------------------------------------
  // Interrupts
  // ----------------------------------------------------------------------------
  //JTAG + SW Debug access clock reset
  // JTAG + SW Debug access functional signals
  output               DAPTDO,                // Debug TDO
  output               DAPJTAGNSW,            // JTAG or Serial-Wire selection JTAG mode(1) or SW mode(0)
  output               DAPNTDOEN,             // TDO output pad control signal
  input                DAPSWDITMS,            // Debug TMS
  input                DAPTDI,                // Debug TDI
  input                DAPNTRST,              // Test reset
  input                DAPSWCLKTCK,           // Test clock / SWCLK
  //TRACE Interface functional signals
  output  [3:0]        TPIUTRACEDATA,         // Output data. A system might not connect all the bits of
  output               TPIUTRACECLK,          // Output clock, used by the TPA to sample the other pins
  input   [4:0]        GPINT,                 // 5 more interrupts input 
  input                FLASHERR,              // Output clock, used by the TPA to sample the other pins
  input                FLASHINT               // Output clock, used by the TPA to sample the other pins

 );

endmodule

//FLASH64K with sleep mode for gw1nz_1
module FLASH64K (DOUT, DIN, XADR, YADR, XE, YE, SE, ERASE, PROG, NVSTR, SLEEP);
input[4:0]XADR;
input[5:0]YADR;
input XE,YE,SE;
input ERASE,PROG,NVSTR;
input SLEEP;
input [31:0] DIN;
output reg [31:0] DOUT;
reg[31:0]MEM_DATA[31:0][63:0];//32 XADR,64 YADR

//define the mapping of XADR and PAGE
wire [4:0]n;
assign n = {XADR[4:3],3'b0};
/////////////////////////////////////////
reg[3:0]p;
reg[6:0]q;
reg [3:0] state = 4'd0;
parameter IDLE    =  4'd0,
          ERA_S1  =  4'd1,
		  ERA_S2  =  4'd2,
		  ERA_S3  =  4'd3,
		  ERA_S4  =  4'd4,
		  ERA_S5  =  4'd5,
		  PRO_S1  =  4'd6,
		  PRO_S2  =  4'd7,
		  PRO_S3  =  4'd8,
		  PRO_S4  =  4'd9,
		  PRO_S5  =  4'd10,
		  RD_S1   =  4'd11,
		  RD_S2   =  4'd12;		  
always@(XE,YE,SE,ERASE,PROG,NVSTR)
begin
	case(state)
	IDLE:
	begin
	  if(XE && ~YE && ~SE && ~NVSTR)
	    begin
		  #10
		  state  =  ERA_S1;
		end
	  else
	    begin
	      if(XE && YE && ~PROG && ~NVSTR && ~ERASE)
		    begin
			  #1
			  state  =  RD_S1;
			end
		end
	end
	ERA_S1:
	begin
	  if(ERASE)
	    begin
		  #5_000
		  state  =  ERA_S2;
		end
	  else if(PROG)
	    begin
		      #5_000
		      state  =  PRO_S1;
		end
	  else if(~XE)
        begin 
          	state  = IDLE;	
		end
	end
	
	ERA_S2:
	begin
	  if(NVSTR)//erase
	    begin
		  #100_000_000
		  for(p=0;p<=7;p=p+1)
		  for(q=0;q<=63;q=q+1)
		  MEM_DATA[n+p][q] = 32'h0;
		  state  =  ERA_S3;
		end
	end
	ERA_S3:
	begin
	  if(~ERASE)
	    begin
		  #5_000
		  state  =  ERA_S4;
		end
	end
	ERA_S4:
	begin
	  if(~NVSTR)
	    begin
		  #1
		  state  =  ERA_S5;
		end
	end
	ERA_S5:
	begin
	  if(~XE)
	    begin
		  state  =  IDLE;
		end
	end
	PRO_S1:
	begin
	  if(NVSTR)
	  begin
	    #10_000
		state  =  PRO_S2;
	  end
	end
	PRO_S2:
	begin
	  if(~PROG)
	    begin
	      #5_000
		  state  =  PRO_S4;
		end
	  else
	    begin
	      if(YE)//program
	        begin
	          #8_000
		      MEM_DATA[XADR][YADR] = DIN;
		      state  =  PRO_S3;
			end
	    end
	end
	PRO_S3:
	begin
		  if(~YE)
	        begin
	          #20
		      state  =  PRO_S2;
	        end	
	end
	PRO_S4:
	begin
	  if(~NVSTR)
	    begin
		  #1
		  state  =  PRO_S5;
		end
	end
	PRO_S5:
	begin
	  if(~XE)
	    begin
		  state  =  IDLE;
		end
	end
	RD_S1:
	begin
	  if(~XE && ~YE)
	    begin
		  state  =  IDLE;
		end
	  else
	    begin
	      if(SE)//read
	        begin
		      #5
		      state  =  RD_S2;
		      DOUT = MEM_DATA[XADR][YADR];
			end
		end
	end
	RD_S2:
	begin
	  if(~SE)
	    begin
		  #20
		  state  =  IDLE;
		end
	end
	default:
	begin
	  state  =  IDLE;
	end
	endcase
end

endmodule 

//FLASH64K for gw1nz_1
module FLASH64KZ (DOUT, DIN, XADR, YADR, XE, YE, SE, ERASE, PROG, NVSTR);
input[4:0]XADR;
input[5:0]YADR;
input XE,YE,SE;
input ERASE,PROG,NVSTR;
input [31:0] DIN;
output reg [31:0] DOUT;
reg[31:0]MEM_DATA[31:0][63:0];//32 XADR,64 YADR

//define the mapping of XADR and PAGE
wire [4:0]n;
assign n = {XADR[4:3],3'b0};
/////////////////////////////////////////
reg[3:0]p;
reg[6:0]q;
reg [3:0] state = 4'd0;
parameter IDLE    =  4'd0,
          ERA_S1  =  4'd1,
		  ERA_S2  =  4'd2,
		  ERA_S3  =  4'd3,
		  ERA_S4  =  4'd4,
		  ERA_S5  =  4'd5,
		  PRO_S1  =  4'd6,
		  PRO_S2  =  4'd7,
		  PRO_S3  =  4'd8,
		  PRO_S4  =  4'd9,
		  PRO_S5  =  4'd10,
		  RD_S1   =  4'd11,
		  RD_S2   =  4'd12;		  
always@(XE,YE,SE,ERASE,PROG,NVSTR)
begin
	case(state)
	IDLE:
	begin
	  if(XE && ~YE && ~SE && ~NVSTR)
	    begin
		  #10
		  state  =  ERA_S1;
		end
	  else
	    begin
	      if(XE && YE && ~PROG && ~NVSTR && ~ERASE)
		    begin
			  #1
			  state  =  RD_S1;
			end
		end
	end
	ERA_S1:
	begin
	  if(ERASE)
	    begin
		  #5_000
		  state  =  ERA_S2;
		end
	  else if(PROG)
	    begin
		      #5_000
		      state  =  PRO_S1;
		end
	  else if(~XE)
        begin 
          	state  = IDLE;	
		end
	end
	
	ERA_S2:
	begin
	  if(NVSTR)//erase
	    begin
		  #100_000_000
		  for(p=0;p<=7;p=p+1)
		  for(q=0;q<=63;q=q+1)
		  MEM_DATA[n+p][q] = 32'h0;
		  state  =  ERA_S3;
		end
	end
	ERA_S3:
	begin
	  if(~ERASE)
	    begin
		  #5_000
		  state  =  ERA_S4;
		end
	end
	ERA_S4:
	begin
	  if(~NVSTR)
	    begin
		  #1
		  state  =  ERA_S5;
		end
	end
	ERA_S5:
	begin
	  if(~XE)
	    begin
		  state  =  IDLE;
		end
	end
	PRO_S1:
	begin
	  if(NVSTR)
	  begin
	    #10_000
		state  =  PRO_S2;
	  end
	end
	PRO_S2:
	begin
	  if(~PROG)
	    begin
	      #5_000
		  state  =  PRO_S4;
		end
	  else
	    begin
	      if(YE)//program
	        begin
	          #8_000
		      MEM_DATA[XADR][YADR] = DIN;
		      state  =  PRO_S3;
			end
	    end
	end
	PRO_S3:
	begin
		  if(~YE)
	        begin
	          #20
		      state  =  PRO_S2;
	        end	
	end
	PRO_S4:
	begin
	  if(~NVSTR)
	    begin
		  #1
		  state  =  PRO_S5;
		end
	end
	PRO_S5:
	begin
	  if(~XE)
	    begin
		  state  =  IDLE;
		end
	end
	RD_S1:
	begin
	  if(~XE && ~YE)
	    begin
		  state  =  IDLE;
		end
	  else
	    begin
	      if(SE)//read
	        begin
		      #5
		      state  =  RD_S2;
		      DOUT = MEM_DATA[XADR][YADR];
			end
		end
	end
	RD_S2:
	begin
	  if(~SE)
	    begin
		  #20
		  state  =  IDLE;
		end
	end
	default:
	begin
	  state  =  IDLE;
	end
	endcase
end

endmodule 


//I3C
module I3C (
	AAC,        //assert ACK clear
	AAO,        //assert ACK output
	AAS,        //assert ACK set
	ACC,        //assert continuity clear
	ACKHS,      //ACK high period divider
	ACKLS,      //ACK low period divider
	ACO,        //assert continuity output
	ACS,        //assert continuity set
	ADDRS,      //set dynamic address
	CE,         //clock enable
	CLK,        //clock input
	CMC,        //current master set
	CMO,        //current master output
	CMS,        //current master set
	DI,         //data input
	DO,         //unbuffered data output
	DOBUF,      //buffered data output
	LGYC,       //legacy mode clear
	LGYO,       //legacy mode output
	LGYS,       //enter legacy mode set
	PARITYERROR,//indicater of parit bit error
	RECVDHS,    //set receiving data high period divider
	RECVDLS,    //set receiving data low period divider
	RESET,      //asyn.reset, active high  
	SCLI,       //scl input
	SCLO,       //scl output
	SCLOEN,     //scl output enable, active low
	SCLPULLO,   //scl pull-up output
	SCLPULLOEN, //scl pull-up output enable, active low
	SDAI,       //sda input
	SDAO,       //sda output
	SDAOEN,     //sda output enable, active low
	SDAPULLO,   //sda pull-up output
	SDAPULLOEN, //sda pull-up output enable, active low
	SENDAHS,    //set sending address high period divider
	SENDALS,    //set sending address low period divider
	SENDDHS,    //set sending data high period divider
	SENDDLS,    //set sending data low period divider
	SIC,        //system interrupt clear
	SIO,        //system interrupt output
	STRTC,      //start celar
	STRTO,      //start output
	STRTS,      //start set
	STATE,      //state output
	STRTHDS,    //set start hold time
	STOPC,      //stop clear
	STOPO,      //stop output
	STOPS,      //stop set
	
	STOPSUS,     //set stop setup time
	STOPHDS      //set stop hold time

);

parameter ADDRESS = 7'b0000000;

input 	LGYS, CMS, ACS, AAS, STOPS, STRTS;
output 	LGYO, CMO, ACO, AAO, SIO, STOPO, STRTO;
input 	LGYC, CMC, ACC, AAC, SIC, STOPC, STRTC;

input	STRTHDS, SENDAHS, SENDALS, ACKHS;
input	ACKLS, STOPSUS, STOPHDS, SENDDHS;
input	SENDDLS, RECVDHS, RECVDLS, ADDRS;

output	PARITYERROR;
input 	[7:0] DI;
output 	[7:0] DOBUF;
output 	[7:0] DO;
output 	[7:0] STATE;

input	SDAI, SCLI;
output	SDAO, SCLO;
output	SDAOEN, SCLOEN;

output	SDAPULLO, SCLPULLO;
output	SDAPULLOEN, SCLPULLOEN;

input 	CE, RESET, CLK;


endmodule

//IODELAYA
module IODELAYA (DO, DF, DI, SDTAP, VALUE, SETN);

parameter C_STATIC_DLY = 0; //integer, 0~127

input DI;
input  SDTAP;
input  SETN;
input  VALUE;
output DF;
output DO;

reg [6:0] delay_data;
realtime delay_time;
wire [127:0] delay_in;
wire value_en;
reg value_sig,pre_value_sig,pre_DO;

always @(SDTAP or VALUE) begin
    if (!SDTAP) begin
        delay_data <= C_STATIC_DLY;
    end else begin
        if(pre_value_sig == 1'b1 && value_sig == 1'b0) begin
  	        if (SDTAP) begin
   	            //if (SETN && (delay_data != 7'd0))
   	            if (SETN && (DF == 1'b0 || (delay_data == 7'b1111111)))
      		        delay_data <= delay_data - 1;
   	            //else if ((!SETN) && (delay_data != 7'd127))
   	            else if ((!SETN) && (DF == 1'b0 || (delay_data == 7'b0000000)))
      		        delay_data <= delay_data + 1;
  	        end
        end
    end
end

always @(DO) begin
    pre_DO <= DO;
end

assign value_en = VALUE & (~DF);

always @(value_en or DO)
begin
    if (DO == 1'b0 && pre_DO == 1'b1)
    begin
        value_sig <= value_en;
    end
end

always @(value_sig) begin
    pre_value_sig <= value_sig;
end

assign DF = (SETN && (delay_data == 7'd0)) || ((!SETN) && (delay_data == 7'd127));

assign #0.025 delay_in[0] =  DI;
generate 
    genvar i;
    for(i=1;i<128;i=i+1) begin: gen_delay
      assign #0.025 delay_in[i] = delay_in[i-1];
    end
endgenerate

assign DO = (delay_data == 0) ? DI : delay_in[delay_data-1];

endmodule


//IODELAYC
module IODELAYC (DO, DAO, DF, DI, SDTAP, VALUE, SETN, DAADJ, DASEL);

parameter C_STATIC_DLY = 0; //integer,0~127
parameter DYN_DA_SEL = "false"; //false:DA_SEL; true:DASEL
parameter DA_SEL = 2'b00;

input DI;
input  SDTAP;
input  SETN;
input  VALUE;
input [1:0] DASEL;
input [1:0] DAADJ;

output DF;
output DO;
output DAO;

reg [6:0] delay_data;
wire [127:0] delay_in;
reg pre_value;
wire [1:0] dout_sel;
reg DAO,dlyout_mid;

initial begin
    delay_data = 7'b0;
end

assign dout_sel = (DYN_DA_SEL == "true") ? DASEL : DA_SEL;

always @(SDTAP or VALUE) begin
    if (!SDTAP) begin
        delay_data <= C_STATIC_DLY;
    end else begin
        if(pre_value == 1'b1 && VALUE == 1'b0) begin
  	        if (SDTAP) begin
   	            if (SETN && (delay_data != 7'd0))
      		        delay_data <= delay_data - 1;
   	            else if ((!SETN) && (delay_data != 7'd127))
      		        delay_data <= delay_data + 1;
  	        end
        end
    end
end

always @(VALUE) begin
    pre_value <= VALUE;
end

assign DF = (SETN && (delay_data == 7'd0)) || ((!SETN) && (delay_data == 7'd127));

assign #0.025 delay_in[0] =  DI;
generate 
   genvar i;
    for(i=1;i<128;i=i+1) begin: gen_delay
      assign #0.025 delay_in[i] = delay_in[i-1];
    end
endgenerate

assign DO = (delay_data == 0) ? DI : delay_in[delay_data-1];

//delay_out_adjust
//
wire [3:0] dly_data_mid;
assign dly_data_mid = delay_data[3:0];

always@(*)
begin
    if(delay_data[5:4] == 2'b00) begin
        dlyout_mid <= 1'b0;
    end else begin
        dlyout_mid <= delay_in[dly_data_mid + 1];
    end
end


always@(dlyout_mid or DO or dout_sel or DAADJ) 
begin
    if(dout_sel == 2'b00) begin
        DAO <= DO;
    end else if(dout_sel == 2'b01) begin
        if(DAADJ == 2'b00) begin
            DAO <= #0.15 DO;
        end else if (DAADJ == 2'b01) begin
            DAO <= #0.1 DO;
        end else if (DAADJ == 2'b10) begin
            DAO <= #0.05 DO;
        end else if (DAADJ == 2'b11) begin
            DAO <= DO;
        end 

    end else if(dout_sel == 2'b10) begin
        if(DAADJ == 2'b00) begin
            DAO <= #0.2 DO;
        end else if (DAADJ == 2'b01) begin
            DAO <= #0.25 DO;
        end else if (DAADJ == 2'b10) begin
            DAO <= #0.3 DO;
        end else if (DAADJ == 2'b11) begin
            DAO <= #0.35 DO;
        end

    end else if(dout_sel == 2'b11) begin
        DAO <= dlyout_mid;
    end
end


endmodule // IODELAYC



//PLLVR,PLL with regulator
module PLLVR (CLKOUT, CLKOUTP, CLKOUTD, CLKOUTD3, LOCK, CLKIN, CLKFB, FBDSEL, IDSEL, ODSEL, DUTYDA, PSDA, FDLY, RESET, RESET_P, VREN);
input CLKIN;
input CLKFB;
input RESET;
input RESET_P;
input [5:0] FBDSEL;
input [5:0] IDSEL;
input [5:0] ODSEL;
input [3:0] PSDA,FDLY;
input [3:0] DUTYDA;
input VREN; //regulator enable

output CLKOUT;
output LOCK;
output CLKOUTP;
output CLKOUTD;
output CLKOUTD3;

parameter FCLKIN = "100.0"; // frequency of the CLKIN(M)
parameter DYN_IDIV_SEL= "false";//true:IDSEL; false:IDIV_SEL
parameter IDIV_SEL = 0; // 0:1,1:2...63:64. 1~64
parameter DYN_FBDIV_SEL= "false";//true:FBDSEL; false:FBDIV_SEL
parameter FBDIV_SEL = 0; // 0:1,1:2...63:64. 1~64
parameter DYN_ODIV_SEL= "false";//true:ODSEL; false:ODIV_SEL
parameter ODIV_SEL = 8; // 2/4/8/16/32/48/64/80/96/112/128

parameter PSDA_SEL= "0000";//
parameter DYN_DA_EN = "false";//true:PSDA or DUTYDA or FDA; false: DA_SEL
parameter DUTYDA_SEL= "1000";//

parameter CLKOUT_FT_DIR = 1'b1; // CLKOUT fine tuning direction. 1'b1 only
parameter CLKOUTP_FT_DIR = 1'b1; // 1'b1 only
parameter CLKOUT_DLY_STEP = 0; // 0,1,2,4
parameter CLKOUTP_DLY_STEP = 0; // 0,1,2

parameter CLKFB_SEL = "internal"; //"internal", "external";
parameter CLKOUT_BYPASS = "false";  //"true"; "false"
parameter CLKOUTP_BYPASS = "false";   //"true"; "false"
parameter CLKOUTD_BYPASS = "false";  //"true"; "false"
parameter DYN_SDIV_SEL = 2; // 2~128,only even num
parameter CLKOUTD_SRC =  "CLKOUT";  //CLKOUT,CLKOUTP
parameter CLKOUTD3_SRC = "CLKOUT"; //CLKOUT,CLKOUTP
parameter DEVICE = "GW1NS-4";//"GW1NS-4","GW1NS-4C","GW1NSR-4","GW1NSER-4C","GW1NSR-4C"

wire resetn;
wire [5:0] IDIV_SEL_reg,FBDIV_SEL_reg;
wire [5:0] IDIV_dyn,FBDIV_dyn;
reg [5:0] IDIV_SEL_reg1,FBDIV_SEL_reg1,ODSEL_reg;
wire div_dyn_change;
integer IDIV_reg,FBDIV_reg;
wire clk_div_src;
reg clk_effect,oclk_effect,oclk_build;
realtime curtime,pretime,fb_delay;
realtime clkin_cycle[4:0];
realtime clkin_period,clkin_period1,clkout_period,tclkout_half,tclkout_half_new;
realtime clkfb_curtime,clkin_curtime,FB_dly,FB_dly0;
reg clkin_init,fb_clk_init;
reg clkout,clk_out,clkfb_reg,clkoutp,clk_ps_reg,clk_ps_reg0;
reg clkfb;
reg lock_reg;
realtime ps_dly,f_dly,clkout_duty, ps_value, duty_value,tclkp_duty;
real unit_div=1.0, real_fbdiv=1.0;
integer cnt_div;
reg clkout_div_reg;
integer multi_clkin;
wire div3_in;
integer cnt_div3;
reg div3_reg;
reg clkfb_init,div3_init,pre_div3_in;


initial begin
IDIV_reg = 1;
FBDIV_reg = 1;
clkin_cycle[0] = 0;
clkin_cycle[1] = 0;
clkin_cycle[2] = 0;
clkin_cycle[3] = 0;
clkin_cycle[4] = 0;
clkin_period = 0;
clkin_period1 = 0;
clkout_period = 0;
clk_effect = 1'b0;
oclk_effect = 1'b0;
oclk_build = 1'b0;
clkfb_reg = 1'b0;
clkout = 1'b0;
clk_out = 1'b0;
clkfb = 1'b0;
clkoutp = 1'b0;
clkin_init = 1'b1;
fb_clk_init = 1'b1;
clkfb_init = 1'b1;
FB_dly = 0.0;
FB_dly0 = 0.0;
clkin_curtime = 0.0;
clkfb_curtime = 0.0;
lock_reg = 0;
clk_ps_reg=0;
clk_ps_reg0=0;
clkout_div_reg=0;
cnt_div=0;
div3_init = 1'b1;
cnt_div3=0;
div3_reg=0;
f_dly = 0.0;
ps_dly = 0.0;
////////////
end

assign resetn = ~( RESET | RESET_P);

// determine period of CLKIN and clkout
always @(posedge CLKIN or negedge resetn) begin
    if(!resetn) begin
        clk_effect <= 1'b0;
    end else begin
        pretime <= curtime;
        curtime <= $realtime;

        if(pretime>0) begin
	        clkin_cycle[0] <= curtime -  pretime;
        end

        if(clkin_cycle[0] > 0) begin
            clkin_cycle[1] <= clkin_cycle[0];
	        clkin_cycle[2] <= clkin_cycle[1];
	        clkin_cycle[3] <= clkin_cycle[2];
            clkin_cycle[4] <= clkin_cycle[3];
        end
    
        if (clkin_cycle[0] > 0) begin
            if(((clkin_cycle[0] - clkin_period1 < 0.01) && (clkin_cycle[0] - clkin_period1 > -0.01)) &&(!div_dyn_change)) begin
                clk_effect <= 1'b1;
                clkin_period <= clkin_period1;
            end else begin
                clk_effect <= 1'b0;
            end
        end
    end
end

always @(clkin_cycle[0] or clkin_cycle[1] or clkin_cycle[2] or clkin_cycle[3] or clkin_cycle[4]  or clkin_period1) begin
    if(clkin_cycle[0]!=clkin_period1) begin
		clkin_period1 <= (clkin_cycle[0]+clkin_cycle[1]+clkin_cycle[2]+clkin_cycle[3]+clkin_cycle[4])/5;
    end
end

/*IDSEL/FBDSEL    IDIV_dyn/FBDIV_dyn
111111	divider   /1
111110	divider   /2
.	.
.	.
.	.
000000	divider   /64
*/
assign IDIV_dyn = 64 - IDSEL;
assign FBDIV_dyn = 64 - FBDSEL;

assign IDIV_SEL_reg = (DYN_IDIV_SEL == "true") ? IDIV_dyn : (IDIV_SEL+1) ;
assign FBDIV_SEL_reg = (DYN_FBDIV_SEL == "true") ? FBDIV_dyn : (FBDIV_SEL+1) ;

always @(posedge CLKIN) begin
    IDIV_SEL_reg1 <= IDIV_SEL_reg;
    FBDIV_SEL_reg1 <= FBDIV_SEL_reg;
    ODSEL_reg <= ODSEL;
end

assign div_dyn_change = (IDIV_SEL_reg1 != IDIV_SEL_reg) || (FBDIV_SEL_reg1 != FBDIV_SEL_reg) || (ODSEL_reg != ODSEL);

always @(clkin_period or IDIV_SEL_reg or FBDIV_SEL_reg) begin
    real_fbdiv = (FBDIV_SEL_reg * unit_div);
    clkout_period = ((clkin_period * IDIV_SEL_reg) / real_fbdiv);
    tclkout_half = (clkout_period / 2);
end

realtime clk_tlock_cur;
realtime max_tlock;
integer cnt_lock;
initial begin
    clk_tlock_cur = 0.0;
    max_tlock = 0.0;
    cnt_lock = 0;
end

// lock time
always @(posedge CLKIN or negedge resetn) begin
    if (resetn == 1'b0) begin
        max_tlock <= 0.0;
    end else begin
        if((clkin_cycle[0] >= 2) && (clkin_cycle[0] <= 40)) begin
            max_tlock <= 50000;
        end else if ((clkin_cycle[0] > 40) && (clkin_cycle[0] <= 500)) begin
            max_tlock <= 200000;
        end
    end
end

always @(posedge CLKIN or negedge resetn) begin
    if (resetn == 1'b0) begin
        lock_reg <= 1'b0;
        oclk_effect <= 1'b0;
    end else begin
        if(clk_effect == 1'b1) begin
            cnt_lock <= cnt_lock + 1;

            if(cnt_lock > ((max_tlock/clkin_period) - 10)) begin
                oclk_effect <= 1'b1;
            end else begin
                oclk_effect <= 1'b0;
            end

            if(cnt_lock > (max_tlock/clkin_period)) begin
                lock_reg <= 1'b1;
            end else begin
                lock_reg <= 1'b0;
            end
        end else begin
            oclk_effect <= 1'b0;
            cnt_lock <= 0;
            lock_reg <= 1'b0;
        end
    end
end

// calculate CLKFB feedback delay
always @(posedge CLKIN) begin
    if (clkin_init == 1'b1) begin
        clkin_curtime=$realtime;
        clkin_init = 1'b0;
    end
end

always @(posedge CLKFB) begin
    if (fb_clk_init == 1'b1) begin
        clkfb_curtime=$realtime;
        fb_clk_init = 1'b0;
    end
end

always @(CLKFB or CLKIN) begin
    if ((clkfb_curtime > 0) && (clkin_curtime > 0)) begin
        FB_dly0 = clkfb_curtime - clkin_curtime;
        if ((FB_dly0 >= 0) && (clkin_cycle[0] > 0)) begin
            multi_clkin = FB_dly0 / (clkin_cycle[0]);
            FB_dly = clkin_cycle[0] - (FB_dly0 - (clkin_cycle[0]) * multi_clkin);
        end
    end
end

// clkout
always @(clkfb_reg or oclk_effect) begin
    if(oclk_effect == 1'b0) begin
        clkfb_reg = 1'b0;
    end
    else begin
        if(clkfb_init == 1'b1) begin
            clkfb_reg <= 1'b1;
            clkfb_init = 1'b0;
        end
        else begin
            clkfb_reg <= #tclkout_half ~clkfb_reg;
        end
    end
end

always @(clkfb_reg) begin
    if (CLKFB_SEL == "internal") begin
        clkfb <= clkfb_reg;
    end else begin
        clkfb <= #(FB_dly) clkfb_reg;
    end
end

always @(posedge clkfb) begin
    clkout <= 1'b1;
    #tclkout_half_new
    clkout <= 1'b0;
end

always @(CLKIN or oclk_effect or clkout or resetn) begin
    if (resetn == 1'b0) begin
        clk_out <= 1'b0;
    end else if(CLKOUT_BYPASS == "true") begin
        clk_out <= CLKIN;
    end
    //else if (oclk_effect == 1'b1) begin
    else begin
        clk_out <= clkout;
    end
end

assign CLKOUT = clk_out;
assign LOCK = lock_reg;  

//clkout_p
// DYN_DA_EN == "false".
// phase_shift_value
always @(*) begin
    case (PSDA_SEL)
	    "0000": ps_value = (clkout_period *  0)/16;
	    "0001": ps_value = (clkout_period *  1)/16;
	    "0010": ps_value = (clkout_period *  2)/16;
	    "0011": ps_value = (clkout_period *  3)/16;
	    "0100": ps_value = (clkout_period *  4)/16;
	    "0101": ps_value = (clkout_period *  5)/16;
	    "0110": ps_value = (clkout_period *  6)/16;
	    "0111": ps_value = (clkout_period *  7)/16;
	    "1000": ps_value = (clkout_period *  8)/16;
	    "1001": ps_value = (clkout_period *  9)/16;
	    "1010": ps_value = (clkout_period * 10)/16;
	    "1011": ps_value = (clkout_period * 11)/16;
	    "1100": ps_value = (clkout_period * 12)/16;
	    "1101": ps_value = (clkout_period * 13)/16;
	    "1110": ps_value = (clkout_period * 14)/16;
	    "1111": ps_value = (clkout_period * 15)/16;
	endcase
end

always @(*) begin
	case (DUTYDA_SEL)
	    "0000": duty_value = (clkout_period *  0)/16;
	    "0001": duty_value = (clkout_period *  1)/16;
	    "0010": duty_value = (clkout_period *  2)/16;
	    "0011": duty_value = (clkout_period *  3)/16;
	    "0100": duty_value = (clkout_period *  4)/16;
	    "0101": duty_value = (clkout_period *  5)/16;
	    "0110": duty_value = (clkout_period *  6)/16;
	    "0111": duty_value = (clkout_period *  7)/16;
	    "1000": duty_value = (clkout_period *  8)/16;
	    "1001": duty_value = (clkout_period *  9)/16;
	    "1010": duty_value = (clkout_period * 10)/16;
	    "1011": duty_value = (clkout_period * 11)/16;
	    "1100": duty_value = (clkout_period * 12)/16;
	    "1101": duty_value = (clkout_period * 13)/16;
	    "1110": duty_value = (clkout_period * 14)/16;
	    "1111": duty_value = (clkout_period * 15)/16;
	endcase
end

//DYN_DA_EN = "true"
always @(FDLY) begin
    if(DYN_DA_EN == "true") begin
        if(DEVICE == "GW1N-1" || DEVICE == "GW1N-1S")begin
            case(FDLY)
                4'b0000  : f_dly = 0.000;
                4'b0001  : f_dly = 0.125;
                4'b0010  : f_dly = 0.250;
                4'b0100  : f_dly = 0.500;
                4'b1000  : f_dly = 1.000;
                default : f_dly = 0.000;
            endcase
        end else begin
            case(FDLY)
                4'b1111  : f_dly = 0.000;
                4'b1110  : f_dly = 0.125;
                4'b1101  : f_dly = 0.250;
                4'b1011  : f_dly = 0.500;
                4'b0111  : f_dly = 1.000;
                default : f_dly = 0.000;
            endcase
        end
    end
end

always @ (PSDA or DUTYDA or ps_value or duty_value) begin
    if (DYN_DA_EN == "true") begin
        ps_dly = (clkout_period *PSDA)/16;
        if (DUTYDA > PSDA) begin
            clkout_duty = (clkout_period * (DUTYDA - PSDA))/16;
        end else if (DUTYDA < PSDA) begin
            clkout_duty = (clkout_period*(16 + DUTYDA - PSDA))/16;
        end else begin
            clkout_duty = (clkout_period)/2;
        end
    end else begin
        ps_dly= ps_value;
        clkout_duty = duty_value;
    end
end

always @(tclkout_half or clkout_duty) begin
    if (DYN_DA_EN == "false") begin
        tclkout_half_new <= tclkout_half;
        tclkp_duty <= clkout_duty;
    end else begin
        if (CLKOUT_FT_DIR == 1'b1) begin
            tclkout_half_new <= tclkout_half - (0.05 * CLKOUT_DLY_STEP);
        end else begin
            tclkout_half_new <= tclkout_half + (0.05 * CLKOUT_DLY_STEP);
        end

        if (CLKOUTP_FT_DIR == 1'b1) begin
            tclkp_duty <= clkout_duty - (0.05 * CLKOUTP_DLY_STEP);
	    end else begin
            tclkp_duty <= clkout_duty + (0.05 * CLKOUTP_DLY_STEP);
        end
	end
end

always @(posedge clkfb) begin
    clkoutp <= 1'b1;
    #tclkp_duty
    clkoutp <= 1'b0;
end

always @(clkoutp) begin
    clk_ps_reg0 <= #(ps_dly+f_dly) clkoutp;    
end
      
always @(CLKIN or oclk_effect or clk_ps_reg0 or resetn) begin
    if (resetn == 1'b0) begin
        clk_ps_reg <= 1'b0;
    end else if(CLKOUTP_BYPASS == "true") begin
        clk_ps_reg <= CLKIN;
    end 
    //else if (oclk_effect == 1'b1) begin
    else begin
        clk_ps_reg <= clk_ps_reg0;
    end
end

assign CLKOUTP = clk_ps_reg;

//divide
assign clk_div_src = (CLKOUTD_SRC=="CLKOUTP") ? clk_ps_reg0:clkout;

always @(posedge clk_div_src or negedge resetn) begin
    if (!resetn) begin
        cnt_div <= 0;
	    clkout_div_reg <= 0;
    end else begin
        cnt_div = cnt_div + 1;
		if (cnt_div == DYN_SDIV_SEL/2) begin
	        clkout_div_reg <= ~clkout_div_reg;
			cnt_div <= 0;
        end
	end
end    
    
assign CLKOUTD = (CLKOUTD_BYPASS == "true") ? CLKIN : clkout_div_reg;

// div3
assign div3_in=(CLKOUTD3_SRC=="CLKOUTP")?clk_ps_reg:clk_out; 

always @ (div3_in) begin
    pre_div3_in <= div3_in;
end

always @(div3_in or negedge resetn) begin
    if(div3_init == 1'b1) begin
        if(pre_div3_in == 1'b1 && div3_in == 1'b0) begin
	        div3_reg <= 1;
            div3_init = 1'b0;
            cnt_div3 = 0;
        end
    end else if(resetn == 1'b0) begin
         div3_reg <= 0;
         cnt_div3 = 0;
    end else begin
        cnt_div3 = cnt_div3+1;
        if(cnt_div3 == 3) begin
            div3_reg <= ~div3_reg;
            cnt_div3 = 0;
        end
    end
end

assign CLKOUTD3 = div3_reg;

endmodule


//rPLL, revision PLL
module rPLL (CLKOUT, CLKOUTP, CLKOUTD, CLKOUTD3, LOCK, CLKIN, CLKFB, FBDSEL, IDSEL, ODSEL, DUTYDA, PSDA, FDLY, RESET, RESET_P);
input CLKIN;
input CLKFB;
input RESET; 
input RESET_P; 
input [5:0] FBDSEL; 
input [5:0] IDSEL;
input [5:0] ODSEL;
input [3:0] PSDA,FDLY; 
input [3:0] DUTYDA;

output CLKOUT;
output LOCK;
output CLKOUTP;
output CLKOUTD;
output CLKOUTD3;

parameter FCLKIN = "100.0"; // frequency of the CLKIN(M)
parameter DYN_IDIV_SEL= "false";//true:IDSEL; false:IDIV_SEL
parameter IDIV_SEL = 0; // 0:1,1:2...63:64. 1~64
parameter DYN_FBDIV_SEL= "false";//true:FBDSEL; false:FBDIV_SEL
parameter FBDIV_SEL = 0; // 0:1,1:2...63:64. 1~64
parameter DYN_ODIV_SEL= "false";//true:ODSEL; false:ODIV_SEL
parameter ODIV_SEL = 8; // 2/4/8/16/32/48/64/80/96/112/128

parameter PSDA_SEL= "0000";//
parameter DYN_DA_EN = "false";//true:PSDA or DUTYDA or FDA; false: DA_SEL
parameter DUTYDA_SEL= "1000";//

parameter CLKOUT_FT_DIR = 1'b1; // CLKOUT fine tuning direction. 1'b1 only
parameter CLKOUTP_FT_DIR = 1'b1; // 1'b1 only
parameter CLKOUT_DLY_STEP = 0; // 0,1,2,4
parameter CLKOUTP_DLY_STEP = 0; // 0,1,2

parameter CLKFB_SEL = "internal"; //"internal", "external";
parameter CLKOUT_BYPASS = "false";  //"true"; "false"
parameter CLKOUTP_BYPASS = "false";   //"true"; "false"
parameter CLKOUTD_BYPASS = "false";  //"true"; "false"
parameter DYN_SDIV_SEL = 2; // 2~128,only even num
parameter CLKOUTD_SRC =  "CLKOUT";  //CLKOUT,CLKOUTP
parameter CLKOUTD3_SRC = "CLKOUT"; //CLKOUT,CLKOUTP
parameter DEVICE = "GW1N-4";//"GW1N-1","GW1N-4","GW1N-9","GW1NR-4","GW1NR-9","GW1N-4B","GW1NR-4B","GW1NS-2","GW1NS-2C","GW1NZ-1","GW1NSR-2","GW1NSR-2C","GW1N-1S","GW1NSE-2C","GW1NRF-4B","GW1N-9C","GW1NR-9C"

wire resetn;
wire [5:0] IDIV_SEL_reg,FBDIV_SEL_reg;
wire [5:0] IDIV_dyn,FBDIV_dyn;
reg [5:0] IDIV_SEL_reg1,FBDIV_SEL_reg1,ODSEL_reg;
wire div_dyn_change;
integer IDIV_reg,FBDIV_reg;
wire clk_div_src;
reg clk_effect,oclk_effect,oclk_build;
realtime curtime,pretime,fb_delay;
realtime clkin_cycle[4:0];
realtime clkin_period,clkin_period1,clkout_period,tclkout_half,tclkout_half_new;
realtime clkfb_curtime,clkin_curtime,FB_dly,FB_dly0;
reg clkin_init,fb_clk_init;
reg clkout,clk_out,clkfb_reg,clkoutp,clk_ps_reg,clk_ps_reg0;
reg clkfb;
reg lock_reg;
realtime ps_dly,f_dly,clkout_duty, ps_value, duty_value,tclkp_duty;
real unit_div=1.0, real_fbdiv=1.0;
integer cnt_div;
reg clkout_div_reg;
integer multi_clkin;
wire div3_in;
integer cnt_div3;
reg div3_reg;
reg clkfb_init,div3_init,pre_div3_in;


initial begin
IDIV_reg = 1;
FBDIV_reg = 1;
clkin_cycle[0] = 0;
clkin_cycle[1] = 0;
clkin_cycle[2] = 0;
clkin_cycle[3] = 0;
clkin_cycle[4] = 0;
clkin_period = 0;
clkin_period1 = 0;
clkout_period = 0;
clk_effect = 1'b0;
oclk_effect = 1'b0;
oclk_build = 1'b0;
clkfb_reg = 1'b0;
clkout = 1'b0;
clk_out = 1'b0;
clkfb = 1'b0;
clkoutp = 1'b0;
clkin_init = 1'b1;
fb_clk_init = 1'b1;
clkfb_init = 1'b1;
FB_dly = 0.0;
FB_dly0 = 0.0;
clkin_curtime = 0.0;
clkfb_curtime = 0.0;
lock_reg = 0;
clk_ps_reg=0;
clk_ps_reg0=0;
clkout_div_reg=0;
cnt_div=0;
div3_init = 1'b1;
cnt_div3=0;
div3_reg=0;
f_dly = 0.0;
ps_dly = 0.0;
////////////
end

assign resetn = ~( RESET | RESET_P );

// determine period of CLKIN and clkout
always @(posedge CLKIN or negedge resetn) begin
    if(!resetn) begin
        clk_effect <= 1'b0;
    end else begin
        pretime <= curtime;
        curtime <= $realtime;

        if(pretime>0) begin
	        clkin_cycle[0] <= curtime -  pretime;
        end

        if(clkin_cycle[0] > 0) begin
            clkin_cycle[1] <= clkin_cycle[0];
	        clkin_cycle[2] <= clkin_cycle[1];
	        clkin_cycle[3] <= clkin_cycle[2];
            clkin_cycle[4] <= clkin_cycle[3];
        end
    
        if (clkin_cycle[0] > 0) begin
            if(((clkin_cycle[0] - clkin_period1 < 0.01) && (clkin_cycle[0] - clkin_period1 > -0.01)) &&(!div_dyn_change)) begin
                clk_effect <= 1'b1;
                clkin_period <= clkin_period1;
            end else begin
                clk_effect <= 1'b0;
            end
        end
    end
end

always @(clkin_cycle[0] or clkin_cycle[1] or clkin_cycle[2] or clkin_cycle[3] or clkin_cycle[4]  or clkin_period1) begin
    if(clkin_cycle[0]!=clkin_period1) begin
		clkin_period1 <= (clkin_cycle[0]+clkin_cycle[1]+clkin_cycle[2]+clkin_cycle[3]+clkin_cycle[4])/5;
    end
end

/*IDSEL/FBDSEL    IDIV_dyn/FBDIV_dyn
111111	divider   /1
111110	divider   /2
.	.
.	.
.	.
000000	divider   /64
*/
assign IDIV_dyn = 64 - IDSEL;
assign FBDIV_dyn = 64 - FBDSEL;

assign IDIV_SEL_reg = (DYN_IDIV_SEL == "true") ? IDIV_dyn : (IDIV_SEL+1) ;
assign FBDIV_SEL_reg = (DYN_FBDIV_SEL == "true") ? FBDIV_dyn : (FBDIV_SEL+1) ;

always @(posedge CLKIN) begin
    IDIV_SEL_reg1 <= IDIV_SEL_reg;
    FBDIV_SEL_reg1 <= FBDIV_SEL_reg;
    ODSEL_reg <= ODSEL;
end

assign div_dyn_change = (IDIV_SEL_reg1 != IDIV_SEL_reg) || (FBDIV_SEL_reg1 != FBDIV_SEL_reg) || (ODSEL_reg != ODSEL);

always @(clkin_period or IDIV_SEL_reg or FBDIV_SEL_reg) begin
    real_fbdiv = (FBDIV_SEL_reg * unit_div);
    clkout_period = ((clkin_period * IDIV_SEL_reg) / real_fbdiv);
    tclkout_half = (clkout_period / 2);
end

realtime clk_tlock_cur;
realtime max_tlock;
integer cnt_lock;
initial begin
    clk_tlock_cur = 0.0;
    max_tlock = 0.0;
    cnt_lock = 0;
end

// lock time
always @(posedge CLKIN or negedge resetn) begin
    if (resetn == 1'b0) begin
        max_tlock <= 0.0;
    end else begin
        if((clkin_cycle[0] >= 2) && (clkin_cycle[0] <= 40)) begin
            max_tlock <= 50000;
        end else if ((clkin_cycle[0] > 40) && (clkin_cycle[0] <= 500)) begin
            max_tlock <= 200000;
        end
    end
end

always @(posedge CLKIN or negedge resetn) begin
    if (resetn == 1'b0) begin
        lock_reg <= 1'b0;
        oclk_effect <= 1'b0;
    end else begin
        if(clk_effect == 1'b1) begin
            cnt_lock <= cnt_lock + 1;

            if(cnt_lock > ((max_tlock/clkin_period) - 10)) begin
                oclk_effect <= 1'b1;
            end else begin
                oclk_effect <= 1'b0;
            end

            if(cnt_lock > (max_tlock/clkin_period)) begin
                lock_reg <= 1'b1;
            end else begin
                lock_reg <= 1'b0;
            end
        end else begin
            oclk_effect <= 1'b0;
            cnt_lock <= 0;
            lock_reg <= 1'b0;
        end
    end
end

// calculate CLKFB feedback delay
always @(posedge CLKIN) begin
    if (clkin_init == 1'b1) begin
        clkin_curtime=$realtime;
        clkin_init = 1'b0;
    end
end

always @(posedge CLKFB) begin
    if (fb_clk_init == 1'b1) begin
        clkfb_curtime=$realtime;
        fb_clk_init = 1'b0;
    end
end

always @(CLKFB or CLKIN) begin
    if ((clkfb_curtime > 0) && (clkin_curtime > 0)) begin
        FB_dly0 = clkfb_curtime - clkin_curtime;
        if ((FB_dly0 >= 0) && (clkin_cycle[0] > 0)) begin
            multi_clkin = FB_dly0 / (clkin_cycle[0]);
            FB_dly = clkin_cycle[0] - (FB_dly0 - (clkin_cycle[0]) * multi_clkin);
        end
    end
end

// clkout
always @(clkfb_reg or oclk_effect) begin
    if(oclk_effect == 1'b0) begin
        clkfb_reg = 1'b0;
    end
    else begin
        if(clkfb_init == 1'b1) begin
            clkfb_reg <= 1'b1;
            clkfb_init = 1'b0;
        end
        else begin
            clkfb_reg <= #tclkout_half ~clkfb_reg;
        end
    end
end

always @(clkfb_reg) begin
    if (CLKFB_SEL == "internal") begin
        clkfb <= clkfb_reg;
    end else begin
        clkfb <= #(FB_dly) clkfb_reg;
    end
end

always @(posedge clkfb) begin
    clkout <= 1'b1;
    #tclkout_half_new
    clkout <= 1'b0;
end

always @(CLKIN or oclk_effect or clkout or resetn) begin
    if (resetn == 1'b0) begin
        clk_out <= 1'b0;
    end else if(CLKOUT_BYPASS == "true") begin
        clk_out <= CLKIN;
    end
    //else if (oclk_effect == 1'b1) begin
    else begin
        clk_out <= clkout;
    end
end

assign CLKOUT = clk_out;
assign LOCK = lock_reg;  

//clkout_p
// DYN_DA_EN == "false".
// phase_shift_value
always @(*) begin
    case (PSDA_SEL)
	    "0000": ps_value = (clkout_period *  0)/16;
	    "0001": ps_value = (clkout_period *  1)/16;
	    "0010": ps_value = (clkout_period *  2)/16;
	    "0011": ps_value = (clkout_period *  3)/16;
	    "0100": ps_value = (clkout_period *  4)/16;
	    "0101": ps_value = (clkout_period *  5)/16;
	    "0110": ps_value = (clkout_period *  6)/16;
	    "0111": ps_value = (clkout_period *  7)/16;
	    "1000": ps_value = (clkout_period *  8)/16;
	    "1001": ps_value = (clkout_period *  9)/16;
	    "1010": ps_value = (clkout_period * 10)/16;
	    "1011": ps_value = (clkout_period * 11)/16;
	    "1100": ps_value = (clkout_period * 12)/16;
	    "1101": ps_value = (clkout_period * 13)/16;
	    "1110": ps_value = (clkout_period * 14)/16;
	    "1111": ps_value = (clkout_period * 15)/16;
	endcase
end

always @(*) begin
	case (DUTYDA_SEL)
	    "0000": duty_value = (clkout_period *  0)/16;
	    "0001": duty_value = (clkout_period *  1)/16;
	    "0010": duty_value = (clkout_period *  2)/16;
	    "0011": duty_value = (clkout_period *  3)/16;
	    "0100": duty_value = (clkout_period *  4)/16;
	    "0101": duty_value = (clkout_period *  5)/16;
	    "0110": duty_value = (clkout_period *  6)/16;
	    "0111": duty_value = (clkout_period *  7)/16;
	    "1000": duty_value = (clkout_period *  8)/16;
	    "1001": duty_value = (clkout_period *  9)/16;
	    "1010": duty_value = (clkout_period * 10)/16;
	    "1011": duty_value = (clkout_period * 11)/16;
	    "1100": duty_value = (clkout_period * 12)/16;
	    "1101": duty_value = (clkout_period * 13)/16;
	    "1110": duty_value = (clkout_period * 14)/16;
	    "1111": duty_value = (clkout_period * 15)/16;
	endcase
end

// DYN_DA_EN == "true".
always @(FDLY) begin
    if(DYN_DA_EN == "true") begin
        if(DEVICE == "GW1N-1" || DEVICE == "GW1N-1S")begin
            case(FDLY)
                4'b0000  : f_dly = 0.000;
                4'b0001  : f_dly = 0.125;
                4'b0010  : f_dly = 0.250;
                4'b0100  : f_dly = 0.500;
                4'b1000  : f_dly = 1.000;
                default : f_dly = 0.000;
            endcase
        end else begin
            case(FDLY)
                4'b1111  : f_dly = 0.000;
                4'b1110  : f_dly = 0.125;
                4'b1101  : f_dly = 0.250;
                4'b1011  : f_dly = 0.500;
                4'b0111  : f_dly = 1.000;
                default : f_dly = 0.000;
            endcase
        end
    end
end

always @ (PSDA or DUTYDA or ps_value or duty_value) begin
    if (DYN_DA_EN == "true") begin
        ps_dly = (clkout_period *PSDA)/16;
        if (DUTYDA > PSDA) begin
            clkout_duty = (clkout_period * (DUTYDA - PSDA))/16;
        end else if (DUTYDA < PSDA) begin
            clkout_duty = (clkout_period*(16 + DUTYDA - PSDA))/16;
        end else begin
            clkout_duty = (clkout_period)/2;
        end
    end else begin
        ps_dly= ps_value;
        clkout_duty = duty_value;
    end
end

always @(tclkout_half or clkout_duty) begin
    if (DYN_DA_EN == "false") begin
        tclkout_half_new <= tclkout_half;
        tclkp_duty <= clkout_duty;
    end else begin
        if (CLKOUT_FT_DIR == 1'b1) begin
            tclkout_half_new <= tclkout_half - (0.05 * CLKOUT_DLY_STEP);
        end else begin
            tclkout_half_new <= tclkout_half + (0.05 * CLKOUT_DLY_STEP);
        end

        if (CLKOUTP_FT_DIR == 1'b1) begin
            tclkp_duty <= clkout_duty - (0.05 * CLKOUTP_DLY_STEP);
	    end else begin
            tclkp_duty <= clkout_duty + (0.05 * CLKOUTP_DLY_STEP);
        end
	end
end

always @(posedge clkfb) begin
    clkoutp <= 1'b1;
    #tclkp_duty
    clkoutp <= 1'b0;
end

always @(clkoutp) begin
    clk_ps_reg0 <= #(ps_dly+f_dly) clkoutp;    
end
      
always @(CLKIN or oclk_effect or clk_ps_reg0 or resetn) begin
    if (resetn == 1'b0) begin
        clk_ps_reg <= 1'b0;
    end else if(CLKOUTP_BYPASS == "true") begin
        clk_ps_reg <= CLKIN;
    end 
    //else if (oclk_effect == 1'b1) begin
    else begin
        clk_ps_reg <= clk_ps_reg0;
    end
end

assign CLKOUTP = clk_ps_reg;

//divide
assign clk_div_src = (CLKOUTD_SRC=="CLKOUTP") ? clk_ps_reg0:clkout;

always @(posedge clk_div_src or negedge resetn) begin
    if (!resetn) begin
        cnt_div <= 0;
	    clkout_div_reg <= 0;
    end else begin
        cnt_div = cnt_div + 1;
		if (cnt_div == DYN_SDIV_SEL/2) begin
	        clkout_div_reg <= ~clkout_div_reg;
			cnt_div <= 0;
        end
	end
end    
    
assign CLKOUTD = (CLKOUTD_BYPASS == "true") ? CLKIN : clkout_div_reg;

// div3
assign div3_in=(CLKOUTD3_SRC=="CLKOUTP")?clk_ps_reg:clk_out; 

always @ (div3_in) begin
    pre_div3_in <= div3_in;
end

always @(div3_in or negedge resetn) begin
    if(div3_init == 1'b1) begin
        if(pre_div3_in == 1'b1 && div3_in == 1'b0) begin
	        div3_reg <= 1;
            div3_init = 1'b0;
            cnt_div3 = 0;
        end
    end else if(resetn == 1'b0) begin
         div3_reg <= 0;
         cnt_div3 = 0;
    end else begin
        cnt_div3 = cnt_div3+1;
        if(cnt_div3 == 3) begin
            div3_reg <= ~div3_reg;
            cnt_div3 = 0;
        end
    end
end

assign CLKOUTD3 = div3_reg;

endmodule



//SPMI
module SPMI (
	CLK	,
	CLKEXT,
	CE	,
	RESETN	,
	ENEXT,
	LOCRESET,
	PA	,
	SA	,
	CA	,
	ADDRI	,
	DATAI	,
	ADDRO	,
	DATAO	,
	STATE	,
	CMD	,
	SDATA,
	SCLK
);

parameter FUNCTION_CTRL = 7'b0000000; 
parameter MSID_CLKSEL = 7'b0000000;//M_S_ID & CLK_FROM_EXT_SEL
parameter RESPOND_DELAY = 4'b0000;
parameter SCLK_NORMAL_PERIOD = 7'b0000000;
parameter SCLK_LOW_PERIOD = 7'b0000000;
parameter CLK_FREQ = 7'b0000000;
parameter SHUTDOWN_BY_ENABLE = 1'b0; //1'b1:shutdown by ENEXT pin(active low); 1'b0:enable spmi

//dedicated multiplex pin
input	CLKEXT, ENEXT;
inout	SDATA, 	SCLK;

//
input 	CLK, CE, RESETN, LOCRESET;
input 	PA, SA, CA;
input	[3:0] 	ADDRI;
input	[7:0] 	DATAI;

output 	[3:0] 	ADDRO;
output 	[7:0] 	DATAO;
output 	[15:0] 	STATE;
output	[3:0]	CMD;

endmodule




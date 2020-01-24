// Test the AESOP-Lite tracker FPGA command interface
// R. Johnson   July 15, 2015

`include "AESOP_TKR.v"
`include "AESOP_Board.v"
`include "TkrDataMerge.v"
`include "TkrDatReceive.v"
`include "CRC6.v"
`include "TrgStretch.v"
`include "DataRAMbufferAESOP_small.v"
`include "LoadIna226.v"
`include "TkrInitial.v"

module DAT_tb();

task sendByte;
	input [7:0] Byte;
	begin
		RxD_data = Byte;         
		RxD_data_ready = 1'b1;
		#10 RxD_data_ready = 1'b0;
	end
endtask 

reg [7:0] RxD_data, Dummy_data;
reg RxD_data_ready;
wire [7:0] TxD_data;
reg Reset, Clock, Trigger;
reg CmdIn, DataIn, SDAin;
reg [63:0] D00,D01,D02,D03,D04,D05,D06,D07,D08,D09,D0A,D0B;
reg [63:0] D10,D11,D12,D13,D14,D15,D16,D17,D18,D19,D1A,D1B;
reg [63:0] D20,D21,D22,D23,D24,D25,D26,D27,D28,D29,D2A,D2B;
reg [63:0] D30,D31,D32,D33,D34,D35,D36,D37,D38,D39,D3A,D3B;
reg TxBusy;

initial begin
    TxBusy = 1'b0;
    Cntr = 0;
	Clock = 1'b0;
    Reset = 1'b0;
	CmdIn = 1'b0;
	DataIn = 1'b0;
	SDAin = 1'b0;
	Dummy_data = 8'b0;
	D00=0; D01=0; D02=0; D03=0; D04=0; D05=0; D06=0; D07=0; D08=0; D09=0; D0A=0; D0B=0;
    D10=0; D11=0; D12=0; D13=0; D14=0; D15=0; D16=0; D17=0; D18=0; D19=0; D1A=0; D1B=0;
    D20=0; D21=0; D22=0; D23=0; D24=0; D25=0; D26=0; D27=0; D28=0; D29=0; D2A=0; D2B=0;
	D30=0; D31=0; D32=0; D33=0; D34=0; D35=0; D36=0; D37=0; D38=0; D39=0; D3A=0; D3B=0;
	
	RxD_data_ready = 1'b0;
	#10 Reset = 1'b1;
	#10 Reset = 1'b0;
	
	#200 sendByte(8'h07);  $display("%g\t Turn the power on to the ASICs",$time);
	#80  sendByte(8'h08);
	#80  sendByte(8'h00);
	
	#2000 sendByte(8'h07);  $display("%g\t Send a RESET to the ASICs",$time);
	#80   sendByte(8'h05);
	#80   sendByte(8'h00);

	#10000 sendByte(8'h07);  $display("%g\t Send a soft RESET command to the ASICs",$time);
	#80   sendByte(8'h0c);
	#80   sendByte(8'h01);
	#80   sendByte(8'h1f);
	
	#4000 sendByte(8'h07);  $display("%g\t Send a reset to the FPGAs",$time);
	#80   sendByte(8'h03);
	#80   sendByte(8'h00);
	
	#2000 sendByte(8'h07);  $display("%g\t Reset the FPGA congfiguration",$time);
	#80   sendByte(8'h04);
	#80   sendByte(8'h00);
/*	
	#2000 sendByte(8'h02);  $display("%g\t Load the ASIC mask",$time);
	#80   sendByte(8'h0e);
	#80   sendByte(8'h02);
	#80   sendByte(8'b00001001);
	#80   sendByte(8'b10001001);
	
	#2000 sendByte(8'h02);   $display("%g\t Read ASIC mask",$time);
	#80   sendByte(8'h1e);
	#80   sendByte(8'h00);
*/
//	#2000 sendByte(8'h07);  $display("%g\t Load the internal trigger timing",$time);
//	#80   sendByte(8'h61);
//	#80   sendByte(8'h02);
//	#80   sendByte(8'h3);
//	#80   sendByte(8'h5);
	
//	#2000 sendByte(8'h01);   $display("%g\t Read internal trigger timing",$time);
//	#80   sendByte(8'h71);
//	#80   sendByte(8'h00);

//    #2000 sendByte(8'h07);  $display("%g\t Load the trigger setup",$time);
//	#80   sendByte(8'h06);
//	#80   sendByte(8'h02);
//	#80   sendByte(8'h01);
//	#80   sendByte(8'h03);
	
//	#2000 sendByte(8'h00);
//	#80   sendByte(8'h07);
//	#80   sendByte(8'h00);
/*	
	#5000 sendByte(8'h01);  $display("%g\t Send command to read the FPGA code version",$time);
	#80  sendByte(8'h0A);
	#80  sendByte(8'h00);

	#4000 sendByte(8'h07);  $display("%g\t Send command to load a calibration DAC register",$time);
	#80  sendByte(8'h10);
	#80  sendByte(8'h02);
	#80  sendByte(8'h05);
    #80	 sendByte(8'hAB);
*/	
//	#2000 sendByte(8'h00);  $display("%g\t Send command to read a calibration DAC register",$time);
//	#80  sendByte(8'h20);
//	#80  sendByte(8'h01);
//	#80  sendByte(8'h05);
	
//	#4000 sendByte(8'h01);  $display("%g\t Send command to read a calibration DAC register",$time);
//	#80  sendByte(8'h20);
//	#80  sendByte(8'h01);
//	#80  sendByte(8'h05);

    #4000 sendByte(8'h00);  $display("%g\t Set time to wait for GO signal",$time);
	#80   sendByte(8'h56);
	#80   sendByte(8'h01);
	#80   sendByte(8'h29);

	#2000 sendByte(8'h07);   	$display("%g\t Load ASIC configuration registers.",$time);     
    #80 sendByte(8'h12);
	#80 sendByte(8'h04);
	#80 sendByte(8'hff);
	#80 sendByte(8'b10101110);    //Load Configuration register 1010 11 1 00001 011 1000
	#80 sendByte(8'b00010111);                                           
	#80 sendByte(8'b00000000);
/*
    #2000 sendByte(8'h00);      $display("%g\t Read the ASIC configuration register.",$time);
    #80 sendByte(8'h22);
    #80 sendByte(8'h01);
    #80 sendByte(8'h01);	
*/	
	#4000 sendByte(8'h07);      $display("%g\t Set the number of layers to be read out.",$time);
	#80 sendByte(8'h0f);
	#80 sendByte(8'h01);
	#80 sendByte(8'h04);
/*	
	#2000 sendByte(8'h00);      $display("%g\t Return the command counter from layer 0.",$time);
	#80 sendByte(8'h60);
	#80 sendByte(8'h00);
	
	#4000 sendByte(8'h01);      $display("%g\t Return the command counter from layer 1.",$time);
	#80 sendByte(8'h60);
	#80 sendByte(8'h00);
*/
//    #4000 sendByte(8'h00);      $display("%g\t Try an i2c command to read a shunt voltage",$time);
//    #80   sendByte(8'h45);
//    #80   sendByte(8'h01);
//    #80   sendByte(8'h45);	
//	#2000 sendByte(8'h00);  $display("%g\t Send command to read a configuration register",$time);
//	#80  sendByte(8'h22);
//	#80  sendByte(8'h01);
//	#80  sendByte(8'h03);

    #2000 sendByte(8'h02);       $display("%g\t Set the end layers for the trigger",$time);
	#80   sendByte(8'h5A);
	#80   sendByte(8'h01);
	#80   sendByte(8'h01);
	
    #2000 sendByte(8'h03);       $display("%g\t Set the end layers for the trigger",$time);
	#80   sendByte(8'h5A);
	#80   sendByte(8'h01);
	#80   sendByte(8'h01);

    #2000 sendByte(8'h07);      $display("%g\t Enable the trigger",$time);
	#80   sendByte(8'h65);
	#80   sendByte(8'h00);
	
	// event with no GO
	#4000 Trigger = 1'b0;  D01=64'h0100000000000000;  D13=64'h0000a00000000000; D20=64'h000bc00000000000; $display("%g\t Send a trigger and input some tracker hits 1",$time);
                           D34=64'h0000000000000f00;  D35=64'h000000010000b000; D37=64'h00000e0000000000;
	#80 D01=64'h0000000000000000; D13=64'h0000000000000000; D20=64'h0000000000000000;
	    D34=64'h0000000000000000; D35=64'h0000000000000000; D37=64'h0000000000000000;
    #300 Trigger = 1'b0;
    #10 Trigger = 1'b0;  
	
	// event with a GO
	#2000 Trigger = 1'b0;  D01=64'h0000001000000000;  D13=64'h0000000000030000; D20=64'h00abc00000000000; $display("%g\t Send a trigger and input some tracker hits 2",$time);
                          D34=64'h0000000000000f00;  D35=64'h0000000100000000; D37=64'h0000000300000000;
	#80 D01=64'h0000000000000000; D13=64'h0000000000000000; D20=64'h0000000000000000;
	    D34=64'h0000000000000000; D35=64'h0000000000000000; D37=64'h0000000000000000;
    #300 Trigger = 1'b1;
    #10 Trigger = 1'b0; 

    #2000 sendByte(8'h00);  $display("%g\t See if the tracker has an event ready",$time);
	#80   sendByte(8'h57);
	#80   sendByte(8'h00);
	
	#4000 sendByte(8'h07);  $display("%g\t Send a read event command",$time);
	#80   sendByte(8'h01);
	#80   sendByte(8'h01);
	#80   sendByte(8'h00);
	
	// event with a GO 
	#20000 Trigger = 1'b0;  D01=64'h0000001001000000;  D13=64'h0000000000030000; D20=64'h0000300000000000; $display("%g\t Send a trigger and input some tracker hits 3",$time);
                          D34=64'h0000000000000f00;  D35=64'h0000000001000000; D37=64'h0000000030000000;
	#80 D01=64'h0000000000000000; D13=64'h0000000000000000; D20=64'h0000000000000000;
	    D34=64'h0000000000000000; D35=64'h0000000000000000; D37=64'h0000000000000000;
    #300 Trigger = 1'b1;
    #10 Trigger = 1'b0; 

    #4000 sendByte(8'h00);  $display("%g\t See if the tracker has an event ready",$time);
	#80   sendByte(8'h57);
	#80   sendByte(8'h00);
	
	#3000 sendByte(8'h07);  $display("%g\t Send a read event command",$time);
	#80   sendByte(8'h01);
	#80   sendByte(8'h01);
	#80   sendByte(8'h00);	

	// event with no GO
	#20000 Trigger = 1'b0;  D01=64'h0100000000000000;  D13=64'h0000100000000000; D20=64'h0000c00000000000; $display("%g\t Send a trigger and input some tracker hits 4",$time);
                           D34=64'h000000000f000000;  D35=64'h000000010000b000; D37=64'h00000f0000000000;
	#80 D01=64'h0000000000000000; D13=64'h0000000000000000; D20=64'h0000000000000000;
	    D34=64'h0000000000000000; D35=64'h0000000000000000; D37=64'h0000000000000000;
    #300 Trigger = 1'b0;
    #10 Trigger = 1'b0; 
	
    #5000 sendByte(8'h00);  $display("%g\t See if the tracker has an event ready",$time);
	#80   sendByte(8'h57);
	#80   sendByte(8'h00);
	
	// event with a GO 
	#6000 Trigger = 1'b0;  D01=64'h0300000000000000;  D13=64'h0000000000030000; D20=64'h0000300000000000; $display("%g\t Send a trigger and input some tracker hits 5",$time);
                          D34=64'h0000000000000000;  D35=64'h0000000000000000; D37=64'h0000000000000000;
	#80 D01=64'h0000000000000000; D13=64'h0000000000000000; D20=64'h0000000000000000;
	    D34=64'h0000000000000000; D35=64'h0000000000000000; D37=64'h0000000000000000;
    #300 Trigger = 1'b1;
    #10 Trigger = 1'b0; 

    #2000 sendByte(8'h00);  $display("%g\t See if the tracker has an event ready",$time);
	#80   sendByte(8'h57);
	#80   sendByte(8'h00);
	
	#3000 sendByte(8'h07);  $display("%g\t Send a read event command",$time);
	#80   sendByte(8'h01);
	#80   sendByte(8'h01);
	#80   sendByte(8'h00);	

	// event with a GO
	#20000 Trigger = 1'b0;  D04=64'h0000007000000000;  D13=64'h0000000000030000; D25=64'h0000c00000000000; $display("%g\t Send a trigger and input some tracker hits 6",$time);
                          D34=64'h0001000000000f00;  D35=64'h0000000100000000; D37=64'h0000000300000000;
	#80 D04=64'h0000000000000000; D13=64'h0000000000000000; D25=64'h0000000000000000;
	    D34=64'h0000000000000000; D35=64'h0000000000000000; D37=64'h0000000000000000;
    #300 Trigger = 1'b1;
    #10 Trigger = 1'b0; 
	/*
    #4000 sendByte(8'h07);  $display("%g\t Load the calibration mask registers",$time);
	#80   sendByte(8'h15);
	#80   sendByte(8'h09);
	#80   sendByte(8'h1f);
	#80   sendByte(8'b00100000);
	#80   sendByte(8'b00100000);
	#80   sendByte(8'b00000000);
	#80   sendByte(8'b00000100);
	#80   sendByte(8'b00100000);
	#80   sendByte(8'b00001100);
	#80   sendByte(8'b00000000);
	#80   sendByte(8'b00100000);	

	#4000 sendByte(8'h07);  $display("%g\t Send a calibration strobe command",$time);
	#80   sendByte(8'h02);
	#80   sendByte(8'h03);
	#80   sendByte(8'b00011111);
	#80   sendByte(8'b00001000);
	#80   sendByte(8'h00);

*/	
    #2000 sendByte(8'h00);  $display("%g\t See if the tracker has an event ready",$time);
	#80   sendByte(8'h57);
	#80   sendByte(8'h00);
	
	#5000 sendByte(8'h07);  $display("%g\t Send a read event command",$time);
	#80   sendByte(8'h01);
	#80   sendByte(8'h01);
	#80   sendByte(8'h00);

//	#50000 sendByte(8'h07);  $display("%g\t Send a soft RESET command to the ASICs",$time);
//	#80   sendByte(8'h0c);
//	#80   sendByte(8'h01);
//	#80   sendByte(8'h1f);	

	// event with no GO
	#20000 Trigger = 1'b0;  D06=64'h0100000000000000;  D13=64'h0000100000000000; D21=64'h0000c00000000000; $display("%g\t Send a trigger and input some tracker hits 7",$time);
                           D34=64'h000000000f000000;  D35=64'h000000010000b000; D37=64'h00000f0000000000;
	#80 D06=64'h0000000000000000; D13=64'h0000000000000000; D21=64'h0000000000000000;
	    D34=64'h0000000000000000; D35=64'h0000000000000000; D37=64'h0000000000000000;
    #300 Trigger = 1'b0;
    #10 Trigger = 1'b0; 
	
    #5000 sendByte(8'h00);  $display("%g\t See if the tracker has an event ready",$time);
	#80   sendByte(8'h57);
	#80   sendByte(8'h00);
	
	// event with no GO
	#6000 Trigger = 1'b0;  D06=64'h01aa00aaa0003000;  D13=64'haa0010000000a000; D21=64'h0000c00000001000; $display("%g\t Send a trigger and input some tracker hits 8",$time);
                           D34=64'h001000a00f000000;  D35=64'h0a0000010000b000; D37=64'h00000f000a000000;
	#80 D06=64'h0000000000000000; D13=64'h0000000000000000; D21=64'h0000000000000000;
	    D34=64'h0000000000000000; D35=64'h0000000000000000; D37=64'h0000000000000000;
    #300 Trigger = 1'b0;
    #10 Trigger = 1'b0; 
	
    #5000 sendByte(8'h00);  $display("%g\t See if the tracker has an event ready",$time);
	#80   sendByte(8'h57);
	#80   sendByte(8'h00);	

	// event with a GO
	#20000 Trigger = 1'b0;  D04=64'h00a00070000f0000;  D13=64'h000000ff00030000; D25=64'h0000c0aaa0000000; $display("%g\t Send a trigger and input some tracker hits 9",$time);
                          D34=64'h0001003300000f00;  D35=64'h0030300100000000; D37=64'h0070000300000000;
	#80 D04=64'h0000000000000000; D13=64'h0000000000000000; D25=64'h0000000000000000;
	    D34=64'h0000000000000000; D35=64'h0000000000000000; D37=64'h0000000000000000;
    #300 Trigger = 1'b1;
    #10 Trigger = 1'b0; 

 	
    #2000 sendByte(8'h00);  $display("%g\t See if the tracker has an event ready",$time);
	#80   sendByte(8'h57);
	#80   sendByte(8'h00);
	
	#5000 sendByte(8'h07);  $display("%g\t Send a read event command",$time);
	#80   sendByte(8'h01);
	#80   sendByte(8'h01);
	#80   sendByte(8'h00);	
	
    #20000 sendByte(8'h00);  $display("%g\t See if the tracker has an event ready",$time);
	#80   sendByte(8'h57);
	#80   sendByte(8'h00);
	
    #2000 sendByte(8'h00);  $display("%g\t Read number of triggers filling ASIC buffers",$time);
	#80   sendByte(8'h54);
	#80   sendByte(8'h00);
	
    #4000 sendByte(8'h00);  $display("%g\t Read number of triggers overflowing ASIC buffers",$time);
	#80   sendByte(8'h55);
	#80   sendByte(8'h00);
	
	#5000 $display("%g\t Reached the end of the road.  Quitting now.",$time);
	$finish;
end

AESOP_Board AESOP_Master(.ResetExt(Reset), .SysCLK(Clock), .TxD_start(TxD_start), .TxD_data(TxD_data), .TxD_busy(TxBusy), .RxD_data_ready(RxD_data_ready), .RxD_data(RxD_data),
            .TrigExt(Trigger), .TrigPrimIn(TrigPrim3), .TrigPrimIn2(TrigPrim1), .TrigPrimOut(dummy9), .BrdAddress(4'b1000), .CmdIn(CmdIn), .CmdNextLyr(CmdOutMstr), .DataIn(DataOut1), .DataOut(dummy4), .TrigNextLyr(TrigOutMstr),
			.DataIn0(D00),.DataIn1(D01),.DataIn2(D02),.DataIn3(D03),.DataIn4(D04),.DataIn5(D05),.DataIn6(D06),.DataIn7(D07),.DataIn8(D08),.DataIn9(D09),.DataInA(D0A),.DataInB(D0B));

reg dummy5, dummy8, dummy15;
wire [7:0] dummy2, dummy11, dummy12;

AESOP_Board AESOP_1(.ResetExt(Reset), .SysCLK(Clock), .TxD_start(dummy1), .TxD_data(dummy2), .TxD_busy(TxBusy), .RxD_data_ready(1'b0), .RxD_data(Dummy_data),
            .TrigExt(TrigOutMstr), .TrigPrimIn(TrigPrim2), .TrigPrimIn2(1'b0), .TrigPrimOut(TrigPrim1), .BrdAddress(4'b0001), .CmdIn(CmdOutMstr), .CmdNextLyr(CmdOut1), .DataIn(DataOut2), .DataOut(DataOut1), .TrigNextLyr(TrigOut1),
			.DataIn0(D10),.DataIn1(D11),.DataIn2(D12),.DataIn3(D13),.DataIn4(D14),.DataIn5(D15),.DataIn6(D16),.DataIn7(D17),.DataIn8(D18),.DataIn9(D19),.DataInA(D1A),.DataInB(D1B));

AESOP_Board AESOP_2(.ResetExt(Reset), .SysCLK(Clock), .TxD_start(dummy10), .TxD_data(dummy11), .TxD_busy(TxBusy), .RxD_data_ready(1'b0), .RxD_data(Dummy_data),
            .TrigExt(TrigOut1),    .TrigPrimIn(dummy15), .TrigPrimIn2(1'b0), .TrigPrimOut(TrigPrim2), .BrdAddress(4'b0010), .CmdIn(CmdOut1),    .CmdNextLyr(CmdOut2), .DataIn(DataOut3),   .DataOut(DataOut2), .TrigNextLyr(TrigOut2),
			.DataIn0(D20),.DataIn1(D21),.DataIn2(D22),.DataIn3(D23),.DataIn4(D24),.DataIn5(D25),.DataIn6(D26),.DataIn7(D27),.DataIn8(D28),.DataIn9(D29),.DataInA(D2A),.DataInB(D2B));

AESOP_Board AESOP_3(.ResetExt(Reset), .SysCLK(Clock), .TxD_start(dummy10), .TxD_data(dummy12), .TxD_busy(TxBusy), .RxD_data_ready(1'b0), .RxD_data(Dummy_data),
            .TrigExt(TrigOut2),    .TrigPrimIn(dummy8), .TrigPrimIn2(1'b0), .TrigPrimOut(TrigPrim3), .BrdAddress(4'b0011), .CmdIn(CmdOut2),    .CmdNextLyr(CmdOut3), .DataIn(dummy5),   .DataOut(DataOut3), .TrigNextLyr(TrigOut3),
			.DataIn0(D30),.DataIn1(D31),.DataIn2(D32),.DataIn3(D33),.DataIn4(D34),.DataIn5(D35),.DataIn6(D36),.DataIn7(D37),.DataIn8(D38),.DataIn9(D39),.DataInA(D3A),.DataInB(D3B));
			
always #5 Clock = ~Clock;     //system clock, 10 tics in period

reg[15:0] Cntr;
reg newByte;
always @ (posedge Clock) begin
    //if (TxD_start) $display("%g\t UART TxD_start is high, Cntr=%d, TxBusy=%b, newByte=%b",$time,Cntr,TxBusy,newByte);
    if (Cntr == 0) begin
		if (TxD_start) begin
			//$display("%g\t Byte sent to UART:    %h   %b   %d",$time,TxD_data,TxD_data,TxD_data);
			Cntr <= Cntr + 1;
			TxBusy <= 1'b1;
			newByte <= 1'b1;
		end else newByte <= 1'b0;
    end else begin
	    newByte <= 1'b0;
	    if (Cntr >= 5 && !TxD_start) begin
			Cntr <= 0;
			TxBusy <= 'b0;
		end else Cntr <= Cntr + 1;
	end
	if (RxD_data_ready) $display("%g\t Byte sent to the DAQ: %h   %b",$time,RxD_data,RxD_data);
end

DataPrint DataPrint_U(.Clock(Clock), .Reset(Reset), .RxD_data_ready(newByte), .RxD_data(TxD_data));

endmodule

module DataPrint(Clock, Reset, RxD_data_ready, RxD_data);

input Clock;
input Reset;
input RxD_data_ready;
input [7:0] RxD_data;

wire [7:0] RxD_data;

parameter [1:0] Wait=2'h0;
parameter [1:0] Byts=2'h1;
parameter [1:0] DoIt=2'h2;
reg [1:0] State, NextState;
reg [7:0] nBytes, nChips;
reg [7:0] ByteList[0:255];
reg [2047:0] Hitlist;
reg [11:0] Header, Cluster;
reg [7:0] ByteCnt, BitCnt, Byte0;
reg [4:0] BrdAddr;
reg [15:0] chip, nClust, clust;

always @ (RxD_data_ready or State or BCnt or nBytes or RxD_data) begin
    case (State)
        Wait: begin
		          if (RxD_data_ready && RxD_data != 0) NextState = Byts;
				  else NextState = Wait;
              end
		Byts: begin
		          if (RxD_data_ready) begin
				      if (BCnt == nBytes - 1) NextState = DoIt;
					  else NextState = Byts;
				  end else NextState = Byts;
		      end
		DoIt: begin
		          NextState = Wait;
			  end
		default: begin
		             NextState = Wait;
		         end
	endcase
end

reg [7:0] BCnt, HLL, PckCnt, LyrNum;
always @ (posedge Clock) begin
    if (Reset) begin
        State <= Wait;
		PckCnt <= 0;
	end else begin
	    State <= NextState;
		if (RxD_data_ready) $display("%g\t DataPrint byte = %h",$time,RxD_data);
		case (State) 
		    Wait: begin
			          nBytes <= RxD_data;
					  BCnt <= 0;
					  if (RxD_data_ready) $display("%g\t DataPrint receiving packet of %d bytes.",$time,RxD_data);
			      end
			Byts: begin
			          if (RxD_data_ready) begin
					      ByteList[BCnt] <= RxD_data;
						  Hitlist <= {Hitlist[2039:0],RxD_data};
						  BCnt = BCnt + 1;
					  end
				  end
			DoIt: begin
					  if (ByteList[0] == 8'b11010011 && PckCnt == 0) begin
					      $display("%g\t Event data dump: total number of bytes=%d",$time,nBytes);
					      $display("    Trigger number = %d",{ByteList[1],ByteList[2]});
						  $display("    Command count= %d",ByteList[3]);
						  $display("    Number of hit-list data packets= %d",ByteList[4][2:0]);
						  PckCnt <= ByteList[4][2:0];
						  LyrNum <= 0;
					  end else if (PckCnt != 0) begin
						  PckCnt <= PckCnt - 1;
						  LyrNum <= LyrNum + 1;
						  $display("    Hit list for layer %d, length=%d",LyrNum,nBytes);
						  //$display("    Bits=%b",Hitlist[63:0]);
						  $display("    Identifier byte = %b",ByteList[0]);
						  for (ByteCnt=nBytes; ByteCnt>1; ByteCnt=ByteCnt-1) begin
						      Byte0 = ByteList[ByteCnt-1];
							  for (BitCnt=0; BitCnt<8; BitCnt=BitCnt+1) begin
						          Hitlist = {Hitlist[2046:0],Byte0[BitCnt]};
							  end
						  end
						  //$display("    Bits=%b",Hitlist[63:0]);
						  for (BitCnt=0; BitCnt<5; BitCnt=BitCnt+1) begin
						      BrdAddr[4-BitCnt] = Hitlist[0];
							  Hitlist = {1'b0,Hitlist[2048:1]};
						  end
						  $display("    Board address=%h",BrdAddr[3:0]);
						  for (BitCnt=0; BitCnt<12; BitCnt=BitCnt+1) begin
						      Header[11-BitCnt] = Hitlist[0];
							  Hitlist = {1'b0,Hitlist[2048:1]};
						  end
						  $display("    Event tag=%d",Header[11:5]);
						  nChips = Header[3:0];
						  $display("    Number of chips reporting=%d",nChips);
						  for (chip=0; chip<nChips; chip=chip+1) begin
						      for (BitCnt=0; BitCnt<12; BitCnt=BitCnt+1) begin
						          Header[11-BitCnt] = Hitlist[0];
							      Hitlist = {1'b0,Hitlist[2048:1]};
						      end	
							  nClust = Header[9:6];
                              $display("      Chip %d, Address=%d, # clusters=%d",chip,Header[3:0],nClust);	
                              for (clust=0; clust<nClust; clust=clust+1) begin
							      for (BitCnt=0; BitCnt<12; BitCnt=BitCnt+1) begin
						              Cluster[11-BitCnt] = Hitlist[0];
							          Hitlist = {1'b0,Hitlist[2048:1]};
						          end	
								  $display("        Cluster %d: number of strips=%d, first strip=%d",clust,Cluster[11:6]+1,Cluster[5:0]);
                              end							  
						  end
					  end else if (ByteList[0] == 8'b11110001) begin
					      $display("    Command echo: command count = %d",{ByteList[1],ByteList[2]});
						  $display("                  command code  = %h",{ByteList[3]});
                      end else begin
					      $display("%g\t Register dump: total number of bytes=%d",$time,nBytes);
					  end
					  for (BCnt = 0; BCnt < nBytes; BCnt = BCnt +1) begin
					      $display("    Byte number %d = %h, %b",BCnt,ByteList[BCnt],ByteList[BCnt]);						  
					  end
			      end
		endcase
	end
end
endmodule

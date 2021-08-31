//////////////////////////////////////////////////////////////////////////////////
// Company:   UCSC
// Engineer:  R. Johnson
// 
// Create Date:    7/17/15 
// Project Name:   AESOP-Lite
// Target Devices: Spartan-6
//  
// Description: Top level module for testing the interface by using the Spartan-6 evaluation board
//
//////////////////////////////////////////////////////////////////////////////////
module topTracker(PWREN, Address0, Address1, Address2, MSTRSEL,
    Debug1P, Debug1N, Debug2P, Debug2N, Debug3P, Debug3N,
    INTCLOCKP, INTCLOCKN,
    TRGINP, TRGINN,
    DATAINP, DATAINN,
    DOUTP1, DOUTN1,
    CMDINP1, CMDINN1,
    CMDOUTP, CMDOUTN,
    TRGOUTP, TRGOUTN,
    CLKOUTP, CLKOUTN,
    CLOCKP, CLOCKN,
    CLKPWRP, CLKPWRN,
    SDA, SCL, 
    CLKOUT1P, CLKOUT1N,
    CLKOUT2P, CLKOUT2N,
    CMDOUT1P, CMDOUT1N, CMDOUT2P, CMDOUT2N,
    RSTOUT1P, RSTOUT1N, RSTOUT2P, RSTOUT2N,
    TReq0_P, TReq0_N,
    TReq1_P, TReq1_N,
    TReq2_P, TReq2_N,
    TReq3_P, TReq3_N,
    TReq4_P, TReq4_N,
    TReq5_P, TReq5_N,
    TReq6_P, TReq6_N,
    TReq7_P, TReq7_N,
    TReq8_P, TReq8_N,
    TReq9_P, TReq9_N,
    TReqA_P, TReqA_N,
    TReqB_P, TReqB_N,    
    Data0_P, Data0_N,
    Data1_P, Data1_N,
    Data2_P, Data2_N,
    Data3_P, Data3_N,
    Data4_P, Data4_N,
    Data5_P, Data5_N,
    Data6_P, Data6_N,
    Data7_P, Data7_N,
    Data8_P, Data8_N,
    Data9_P, Data9_N,
    DataA_P, DataA_N,
    DataB_P, DataB_N,
    TACK1P, TACK1N,
    TACK2P, TACK2N
    );

input     Address0, Address1, Address2, MSTRSEL;
output    PWREN;
input     Debug1P, Debug1N;
output    Debug2P, Debug2N;
input     Debug3P, Debug3N;

input    INTCLOCKP, INTCLOCKN;
input    TRGINP, TRGINN;
input    DATAINP, DATAINN;
output   DOUTP1, DOUTN1;
input    CMDINP1, CMDINN1;
output   CMDOUTP, CMDOUTN;
output   TRGOUTP, TRGOUTN;
output   CLKOUTP, CLKOUTN;
input    CLOCKP, CLOCKN;
output   CLKPWRP, CLKPWRN;
inout    SDA, SCL;
output   CLKOUT1P, CLKOUT1N;
output   CLKOUT2P, CLKOUT2N;
output   CMDOUT1P, CMDOUT1N, CMDOUT2P, CMDOUT2N;
output   RSTOUT1P, RSTOUT1N, RSTOUT2P, RSTOUT2N;
output   TACK1P, TACK1N, TACK2P, TACK2N;
    
input TReq0_P, TReq0_N;
input TReq1_P, TReq1_N;
input TReq2_P, TReq2_N;
input TReq3_P, TReq3_N;
input TReq4_P, TReq4_N;
input TReq5_P, TReq5_N;
input TReq6_P, TReq6_N;
input TReq7_P, TReq7_N;
input TReq8_P, TReq8_N;
input TReq9_P, TReq9_N;
input TReqA_P, TReqA_N;
input TReqB_P, TReqB_N;
input Data0_P, Data0_N;
input Data1_P, Data1_N;
input Data2_P, Data2_N;
input Data3_P, Data3_N;
input Data4_P, Data4_N;
input Data5_P, Data5_N;
input Data6_P, Data6_N;
input Data7_P, Data7_N;
input Data8_P, Data8_N;
input Data9_P, Data9_N;
input DataA_P, DataA_N;
input DataB_P, DataB_N;
//input TrgIn_P, TrgIn_N;
//input CmdIn_P, CmdIn_N;    

wire [3:0] BrdAddress;
wire Debug3, Debug4;
assign BrdAddress[0] = Address0;
assign BrdAddress[1] = Address1;
assign BrdAddress[2] = Address2;
assign BrdAddress[3] = MSTRSEL;

reg CmdNext, UART_RX, DOUT_Mux;

parameter delayVal=168;     // 168
parameter delayVal2=250;    // 250

//wire SCLen;
//assign SCLen = 1'b1;

//assign TrigPrimIn = 1'b0;
//IDDR2 Debug1Pbuf (.D(Debug1P), .Q0(TrgInReg), .C0(CLK), .C1(~CLK), .CE(1'b1), .R(1'b0), .S(1'b0));
//IDDR2 Debug1Nbuf (.D(Debug1N), .Q0(extTrg2), .C0(CLK), .C1(~CLK), .CE(1'b1), .R(1'b0), .S(1'b0));
//Daisy chain of trigger primitives
IBUFDS TrigPrimInBuf(.I(Debug1P), .IB(Debug1N), .O(TrigPrimIn));
OBUFDS TrigPrimOutBuf(.O(Debug2P), .OB(Debug2N), .I(TrigPrimOut));
IBUFDS TrigPrimInBuf2(.I(Debug3P), .IB(Debug3N), .O(TrigPrimIn2));   
//assign TrigPrimIn2 = 1'b0;   //Temporary for debugging

//Receive the 10 MHz differential clock from the previous layer (if connected)
IBUFDS CLKINBUF(.I(CLOCKP), .IB(CLOCKN), .O(CLKIN));

//Select which clock source to use, depending on whether this board is the master
BUFGMUX CLKMUX(.I1(CLKEXT), .I0(CLKIN), .S(MSTRSEL), .O(CLK));

//Digital clock manager, derives the 10 MHz local clock from the 20 MHz external clock.  
//Also derives the 1.2 MHz clock for synchronizing DC/DC converters.
//CLK_DLY_CMD is the 10 MHz clock going out to the ASICs
clk_wiz_v3_6 CLKMGR(.CLK_IN1_P(INTCLOCKP),.CLK_IN1_N(INTCLOCKN),.CLK_OUT1(CLKEXT),.CLK_OUT2(CLKPWR_dummy),.RESET(1'b0),.LOCKED(LOCKED));
assign CLKPWR = 1'b0;
assign CLK_DLY = CLK;
assign CLK_DLY2 = CLK;
assign CLK_DLY_CMD = CLK;
assign CLK_DLY_CMD_bar = ~CLK;

//Output clocks going to ASICs
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
        ODDR2_CLK1 (.Q(CLKOUT1), .C0(CLK_DLY_CMD), .C1(CLK_DLY_CMD_bar), .CE(1'b1), .D0(1'b1), .D1(1'b0), .R(1'b0), .S(1'b0));
OBUFDS CLKOUT1_BUFFER (.O(CLKOUT1P), .OB(CLKOUT1N), .I(CLKOUT1));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
        ODDR2_CLK2 (.Q(CLKOUT2), .C0(CLK_DLY_CMD), .C1(CLK_DLY_CMD_bar), .CE(1'b1), .D0(1'b1), .D1(1'b0), .R(1'b0), .S(1'b0));
OBUFDS CLKOUT2_BUFFER (.O(CLKOUT2P), .OB(CLKOUT2N), .I(CLKOUT2));

//Relay the clock to the next higher layer
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
        ODDR2_CLKNXT (.Q(CLKOUT), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(1'b1), .D1(1'b0), .R(1'b0), .S(1'b0));
OBUFDS CLKNXT_BUFFER (.O(CLKOUTP), .OB(CLKOUTN), .I(CLKOUT));

//Send a 1.2 MHz clock to the DC/DC converters
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
        ODDR2_CLKPWR (.Q(CLKPWROUT), .C0(CLKPWR), .C1(~CLKPWR), .CE(1'b1), .D0(1'b1), .D1(1'b0), .R(1'b0), .S(1'b0));
OBUFDS CLKPWR_BUFFER (.O(CLKPWRP), .OB(CLKPWRN), .I(CLKPWROUT));

wire [11:0] TReq, TReqReg;

//LVDS buffers for the Fast-OR trigger request signals from the ASICs
IBUFDS TREQ0_BUFFER    (.I(TReq0_P), .IB(TReq0_N), .O(TReq[0]));
IBUFDS TREQ1_BUFFER    (.I(TReq1_P), .IB(TReq1_N), .O(TReq[1]));
IBUFDS TREQ2_BUFFER    (.I(TReq2_P), .IB(TReq2_N), .O(TReq[2]));
IBUFDS TREQ3_BUFFER    (.I(TReq3_P), .IB(TReq3_N), .O(TReq[3]));
IBUFDS TREQ4_BUFFER    (.I(TReq4_P), .IB(TReq4_N), .O(TReq[4]));
IBUFDS TREQ5_BUFFER    (.I(TReq5_P), .IB(TReq5_N), .O(TReq[5]));
IBUFDS TREQ6_BUFFER    (.I(TReq6_P), .IB(TReq6_N), .O(TReq[6]));
IBUFDS TREQ7_BUFFER    (.I(TReq7_P), .IB(TReq7_N), .O(TReq[7]));
IBUFDS TREQ8_BUFFER    (.I(TReq8_P), .IB(TReq8_N), .O(TReq[8]));
IBUFDS TREQ9_BUFFER    (.I(TReq9_P), .IB(TReq9_N), .O(TReq[9]));
IBUFDS TREQa_BUFFER    (.I(TReqA_P), .IB(TReqA_N), .O(TReq[10]));
IBUFDS TREQb_BUFFER    (.I(TReqB_P), .IB(TReqB_N), .O(TReq[11]));

//Register the TReq signals
IDDR2 Treq0_Reg (.D(TReq[0]), .Q0(TReqReg[0]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq1_Reg (.D(TReq[1]), .Q0(TReqReg[1]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq2_Reg (.D(TReq[2]), .Q0(TReqReg[2]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq3_Reg (.D(TReq[3]), .Q0(TReqReg[3]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq4_Reg (.D(TReq[4]), .Q0(TReqReg[4]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq5_Reg (.D(TReq[5]), .Q0(TReqReg[5]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq6_Reg (.D(TReq[6]), .Q0(TReqReg[6]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq7_Reg (.D(TReq[7]), .Q0(TReqReg[7]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq8_Reg (.D(TReq[8]), .Q0(TReqReg[8]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq9_Reg (.D(TReq[9]), .Q0(TReqReg[9]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treqa_Reg (.D(TReq[10]), .Q0(TReqReg[10]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treqb_Reg (.D(TReq[11]), .Q0(TReqReg[11]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));

//Trigger acknowledge going to ASICs
OBUFDS TACK1_BUFFER (.O(TACK1P), .OB(TACK1N), .I(TACK1DLY));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
        ODDR2_TACK1 (.Q(TACK1LATCH), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(TackASIC), .D1(TackASIC), .R(1'b0), .S(1'b0));
IODELAY2 #(.ODELAY_VALUE(delayVal), .DELAY_SRC("ODATAIN")) TACK1_DELAY (.ODATAIN(TACK1LATCH), .DOUT(TACK1DLY));
OBUFDS TACK2_BUFFER (.O(TACK2P), .OB(TACK2N), .I(TACK2DLY));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
        ODDR2_TACK2 (.Q(TACK2LATCH), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(TackASIC), .D1(TackASIC), .R(1'b0), .S(1'b0));
IODELAY2 #(.ODELAY_VALUE(delayVal), .DELAY_SRC("ODATAIN")) TACK2_DELAY (.ODATAIN(TACK2LATCH), .DOUT(TACK2DLY));

//Send commands to ASICs after aligning with the clock and delaying
OBUFDS CMD1_BUFFER (.O(CMDOUT1P), .OB(CMDOUT1N), .I(CMD1DLY));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
        ODDR2_CMD1 (.Q(CMD1LATCH), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(CMDFAB), .D1(CMDFAB), .R(1'b0), .S(1'b0));
IODELAY2 #(.ODELAY_VALUE(delayVal), .DELAY_SRC("ODATAIN")) CMD1_DELAY (.ODATAIN(CMD1LATCH), .DOUT(CMD1DLY));
OBUFDS CMD2_BUFFER (.O(CMDOUT2P), .OB(CMDOUT2N), .I(CMD2DLY));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
        ODDR2_CMD2 (.Q(CMD2LATCH), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(CMDFAB), .D1(CMDFAB), .R(1'b0), .S(1'b0));
IODELAY2 #(.ODELAY_VALUE(delayVal), .DELAY_SRC("ODATAIN")) CMD2_DELAY (.ODATAIN(CMD2LATCH), .DOUT(CMD2DLY));

//Reset hard reset signals signals to ASICs after aligning with the clock
OBUFDS RST1_BUFFER (.O(RSTOUT1P), .OB(RSTOUT1N), .I(Rst1LATCH));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
        ODDR2_RST1 (.Q(Rst1LATCH), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(RSTchips), .D1(RSTchips), .R(1'b0), .S(1'b0));
OBUFDS RST2_BUFFER (.O(RSTOUT2P), .OB(RSTOUT2N), .I(Rst2LATCH));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
        ODDR2_RST2 (.Q(Rst2LATCH), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(RSTchips), .D1(RSTchips), .R(1'b0), .S(1'b0));
        
//Receive data from the layer above and align it with the clock after delaying
IBUFDS DATAIN_BUFFER (.I(DATAINP), .IB(DATAINN), .O(DATAIN));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(delayVal2), .DELAY_SRC("IDATAIN")) DataIN_DELAY (.IDATAIN(DATAIN), .DATAOUT(DATAInDly));    
IDDR2 DATAIN_REG (.D(DATAInDly), .Q0(DataInReg), .C0(CLK), .C1(~CLK), .CE(1'b1), .R(1'b0), .S(1'b0));

//Send data to the layer below, or to TxD, after aligning with the clock
OBUFDS DATAOUT_BUFFER (.O(DOUTP1), .OB(DOUTN1), .I(DOUT_Reg));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
        ODDR2_DATAOUT (.Q(DOUT_Reg), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(DOUT_Mux), .D1(DOUT_Mux), .R(1'b0), .S(1'b0));

//Receive the LVDS trigger from the layer below or from the trigger system, and align it with the clock after a delay
IBUFDS TRGIN_BUFFER (.I(TRGINP), .IB(TRGINN), .O(TrgIn));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(delayVal2), .DELAY_SRC("IDATAIN")) TrgIN_DELAY (.IDATAIN(TrgIn), .DATAOUT(TrgInDly));    
IDDR2 TRGIN_Reg (.D(TrgInDly), .Q0(TrgInReg), .C0(CLK_DLY2), .C1(~CLK_DLY2), .CE(1'b1), .R(1'b0), .S(1'b0));

//Relay the trigger with 1 clock cycle delay to the next layer
OBUFDS TRGOUT_BUFFER (.O(TRGOUTP), .OB(TRGOUTN), .I(TrigNextLyr));

//Receive commands from the layer below or from RxD and align with the clock after a delay
IBUFDS CMDIN_BUFFER (.I(CMDINP1), .IB(CMDINN1), .O(CmdIn));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(delayVal2), .DELAY_SRC("IDATAIN")) CmdIN_DELAY (.IDATAIN(CmdIn), .DATAOUT(CmdInDly));    
IDDR2 CMDIN_Reg (.D(CmdInDly), .Q0(CmdInReg), .C0(CLK_DLY2), .C1(~CLK_DLY2), .CE(1'b1), .R(1'b0), .S(1'b0));

//Relay the command stream to the next higher layer
OBUFDS CMDOUT_BUFFER(.O(CMDOUTP), .OB(CMDOUTN), .I(CmdNext));

//i2c port
IOBUF #(.DRIVE(12), .IOSTANDARD("I2C"), .SLEW("SLOW") ) IOBUF_SDA (.O(SDAin), .IO(SDA), .I(SDAout), .T(~SDAen));
//IOBUF #(.DRIVE(12), .IOSTANDARD("I2C"), .SLEW("SLOW") ) IOBUF_SCL (.O(SCLinDum), .IO(SCL), .I(SCLout), .T(~SCLen));
OBUFT #(.DRIVE(12), .SLEW("SLOW") ) OBUFT_SCL (.O(SCL), .I(SCLout), .T(~SCLen));

wire [11:0] Data, DataDly, DataReg;

// LVDS buffers for data coming in from the ASICs
IBUFDS Data0_BUFFER    (.I(Data0_P), .IB(Data0_N), .O(Data[0]));
IBUFDS Data1_BUFFER    (.I(Data1_P), .IB(Data1_N), .O(Data[1]));
IBUFDS Data2_BUFFER    (.I(Data2_P), .IB(Data2_N), .O(Data[2]));
IBUFDS Data3_BUFFER    (.I(Data3_P), .IB(Data3_N), .O(Data[3]));
IBUFDS Data4_BUFFER    (.I(Data4_P), .IB(Data4_N), .O(Data[4]));
IBUFDS Data5_BUFFER    (.I(Data5_P), .IB(Data5_N), .O(Data[5]));
IBUFDS Data6_BUFFER    (.I(Data6_P), .IB(Data6_N), .O(Data[6]));
IBUFDS Data7_BUFFER    (.I(Data7_P), .IB(Data7_N), .O(Data[7]));
IBUFDS Data8_BUFFER    (.I(Data8_P), .IB(Data8_N), .O(Data[8]));
IBUFDS Data9_BUFFER    (.I(Data9_P), .IB(Data9_N), .O(Data[9]));
IBUFDS DataA_BUFFER    (.I(DataA_P), .IB(DataA_N), .O(Data[10]));
IBUFDS DataB_BUFFER    (.I(DataB_P), .IB(DataB_N), .O(Data[11]));

// Register the data from the ASICs after a delay
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(delayVal), .DELAY_SRC("IDATAIN")) Data0_DELAY (.IDATAIN(Data[0]), .DATAOUT(DataDly[0]));
IDDR2 Data0_Reg (.D(DataDly[0]), .Q0(DataReg[0]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(delayVal), .DELAY_SRC("IDATAIN")) Data1_DELAY (.IDATAIN(Data[1]), .DATAOUT(DataDly[1]));
IDDR2 Data1_Reg (.D(DataDly[1]), .Q0(DataReg[1]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(delayVal), .DELAY_SRC("IDATAIN")) Data2_DELAY (.IDATAIN(Data[2]), .DATAOUT(DataDly[2]));
IDDR2 Data2_Reg (.D(DataDly[2]), .Q0(DataReg[2]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(delayVal), .DELAY_SRC("IDATAIN")) Data3_DELAY (.IDATAIN(Data[3]), .DATAOUT(DataDly[3]));
IDDR2 Data3_Reg (.D(DataDly[3]), .Q0(DataReg[3]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(delayVal), .DELAY_SRC("IDATAIN")) Data4_DELAY (.IDATAIN(Data[4]), .DATAOUT(DataDly[4]));
IDDR2 Data4_Reg (.D(DataDly[4]), .Q0(DataReg[4]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(delayVal), .DELAY_SRC("IDATAIN")) Data5_DELAY (.IDATAIN(Data[5]), .DATAOUT(DataDly[5]));
IDDR2 Data5_Reg (.D(DataDly[5]), .Q0(DataReg[5]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(delayVal), .DELAY_SRC("IDATAIN")) Data6_DELAY (.IDATAIN(Data[6]), .DATAOUT(DataDly[6]));
IDDR2 Data6_Reg (.D(DataDly[6]), .Q0(DataReg[6]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(delayVal), .DELAY_SRC("IDATAIN")) Data7_DELAY (.IDATAIN(Data[7]), .DATAOUT(DataDly[7]));
IDDR2 Data7_Reg (.D(DataDly[7]), .Q0(DataReg[7]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(delayVal), .DELAY_SRC("IDATAIN")) Data8_DELAY (.IDATAIN(Data[8]), .DATAOUT(DataDly[8]));
IDDR2 Data8_Reg (.D(DataDly[8]), .Q0(DataReg[8]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(delayVal), .DELAY_SRC("IDATAIN")) Data9_DELAY (.IDATAIN(Data[9]), .DATAOUT(DataDly[9]));
IDDR2 Data9_Reg (.D(DataDly[9]), .Q0(DataReg[9]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(delayVal), .DELAY_SRC("IDATAIN")) Data10_DELAY (.IDATAIN(Data[10]), .DATAOUT(DataDly[10]));
IDDR2 DataA_Reg (.D(DataDly[10]), .Q0(DataReg[10]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(delayVal), .DELAY_SRC("IDATAIN")) Data11_DELAY (.IDATAIN(Data[11]), .DATAOUT(DataDly[11]));
IDDR2 DataB_Reg (.D(DataDly[11]), .Q0(DataReg[11]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));

//Main program for the Tracker FPGA

reg CmdIn1;
wire [7:0] TxD_data, RxD_data;
AESOP_TKR TrackerMain(.Debug1(dummy1),.Debug2(dummy2),.Debug3(dummy3),.Debug4(dummy4),.Debug5(Debug3P),.Debug6(Debug3N),
          .ResetExt(1'b0), .SysCLK(CLK), .TxD_start(TxD_start), .TxD_data(TxD_data), .TxD_busy(TxD_busy), .RxD_data_ready(RxD_data_ready), .RxD_data(RxD_data),
          .TrigExt(TrgInReg), .HardReset(RSTchips), .BrdAddress(BrdAddress), .ASICpower(PWREN), .CmdIn(CmdIn1), .CmdNextLyr(CmdRelay), .DataIn1(DataInReg), .DataOut(DataNext),
          .ASICdata(DataReg), .TReq(TReqReg), .TrigPrimIn1(TrigPrimIn), .TrigPrimIn2(TrigPrimIn2), .TrigPrimOut(TrigPrimOut), .CmdASIC(CMDFAB), .Tack(TackASIC),
          .SDAin(SDAin), .SDAout(SDAout), .SDAen(SDAen), .SCLout(SCLout), .SCLen(SCLen), .TrigNextLyr(TrigNextLyr));

//Instantiate the UART receiver and transmitter
async_receiver pcUART_serial_rx(.clk(CLK), .RxD(UART_RX), .RxD_data_ready(RxD_data_ready), .RxD_data(RxD_data));
async_transmitter pcUART_serial_tx(.clk(CLK), .TxD(UART_TX), .TxD_start(TxD_start), .TxD_data(TxD_data), .TxD_busy(TxD_busy));

always @ (MSTRSEL or UART_TX or CmdInReg or CmdRelay or DataNext or CmdInReg) begin
    if (MSTRSEL) begin
        DOUT_Mux = UART_TX;
        UART_RX = CmdInReg;
        CmdNext = CmdRelay;
		  CmdIn1 = 1'b0;
    end else begin
        DOUT_Mux = DataNext;
        UART_RX = 1'b0;
        CmdNext = CmdInReg;
		  CmdIn1 = CmdInReg;
    end
end

endmodule

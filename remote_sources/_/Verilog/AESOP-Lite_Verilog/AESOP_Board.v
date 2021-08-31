// Simulate one AESOP tracker board
// This is only used in iverilog simulations, not in the FPGA on the actual tracker board
// R. Johnson   8/22/2015

`include "Chip_rtl\pCT_Chip.v"

module AESOP_Board(ResetExt, SysCLK, TxD_start, TxD_data, TxD_busy, RxD_data_ready, RxD_data, TrigExt, TrigPrimIn, TrigPrimIn2, TrigPrimOut, BrdAddress, CmdIn, CmdNextLyr, DataIn, DataOut, TrigNextLyr,
                    DataIn0,DataIn1,DataIn2,DataIn3,DataIn4,DataIn5,DataIn6,DataIn7,DataIn8,DataIn9,DataInA,DataInB);

input [63:0] DataIn0,DataIn1,DataIn2,DataIn3,DataIn4,DataIn5,DataIn6,DataIn7,DataIn8,DataIn9,DataInA,DataInB;
input ResetExt;
input SysCLK;
output TxD_start;
output [7:0] TxD_data;
input TxD_busy;
input RxD_data_ready;
input [7:0] RxD_data;
input TrigExt;
input [3:0] BrdAddress;
input CmdIn;
input TrigPrimIn2;
output CmdNextLyr;
input DataIn;
output DataOut;
output TrigNextLyr;
input TrigPrimIn;
output TrigPrimOut;

wire [11:0] DataP,TReq;
reg [11:0] Data;
reg TrigNextLyr, CmdNextLyr;

always @ (posedge SysCLK) begin
    TrigNextLyr <= TrgNxt;
    CmdNextLyr <= CmdNext;
    //if (CmdNext & BrdAddress[3]) $display("%g\t CmdNext from Master",$time);
end

AESOP_TKR AESOP_TKR_U (.ResetExt(ResetExt), .SysCLK(SysCLK), .TxD_start(TxD_start), .TxD_data(TxD_data), .TxD_busy(TxD_busy), .RxD_data_ready(RxD_data_ready), .RxD_data(RxD_data),
          .TrigExt(TrigExt), .BrdAddress(BrdAddress), .ASICpower(ASICpower), .CmdIn(CmdIn), .CmdNextLyr(CmdNext), .DataIn1(DataIn), .DataOut(DataOut),
          .ASICdata(Data), .TReq(TReq), .TrigPrimIn1(TrigPrimIn), .TrigPrimOut(TrigPrimOut), .HardReset(ResetH), .CmdASIC(Cmd), .Tack(TackASIC),
          .SDAin(SDAin), .SDAout(SDAout), .SDAen(SDAen), .SCLout(SCLout), .SCLen(SCLen), .TrigNextLyr(TrgNxt), .TrigPrimIn2(TrigPrimIn2));


always @ (ASICpower) $display("%g\t AESOP_Board %h: ASIC power set to %b",$time,BrdAddress,ASICpower);

//always @ (posedge TackASIC) $display("%g\t AESOP_Board %h: trigger acknowledge signal sent to the asics",$time,BrdAddress);

always @ (posedge Clock) begin
    Data[0] <= DataP[0];
    Data[1] <= DataP[1];
    Data[2] <= DataP[2];
    Data[3] <= DataP[3];
    Data[4] <= DataP[4];
    Data[5] <= DataP[5];
    Data[6] <= DataP[6];
    Data[7] <= DataP[7];
    Data[8] <= DataP[8];
    Data[9] <= DataP[9];
    Data[10] <= DataP[10];
    Data[11] <= DataP[11];
    if (ResetH) $display("%g\t AESOP_board %h:  hard reset",$time,BrdAddress);
    if (DataP != 0 && BrdAddress==2) $display("%g\t %h DataP in AESOP_board.v=%b",$time,BrdAddress,DataP);
end     

//RTL version of the tracker ASIC.  Has some small differences from the final logic design, but it
//simulates much faster.
assign Clock = SysCLK;
wire [1:0] Tack;
assign Tack[0] = TackASIC;
assign Tack[1] = TackASIC;
pCT_Chip Chip0(DataIn0,Tack[0],Cmd,Clock,DataP[0],TReq[0],5'b0000,ResetH,BrdAddress);
pCT_Chip Chip1(DataIn1,Tack[0],Cmd,Clock,DataP[1],TReq[1],5'b0001,ResetH,BrdAddress);
pCT_Chip Chip2(DataIn2,Tack[0],Cmd,Clock,DataP[2],TReq[2],5'b0010,ResetH,BrdAddress);
pCT_Chip Chip3(DataIn3,Tack[0],Cmd,Clock,DataP[3],TReq[3],5'b0011,ResetH,BrdAddress);
pCT_Chip Chip4(DataIn4,Tack[0],Cmd,Clock,DataP[4],TReq[4],5'b0100,ResetH,BrdAddress);
pCT_Chip Chip5(DataIn5,Tack[0],Cmd,Clock,DataP[5],TReq[5],5'b0101,ResetH,BrdAddress);
pCT_Chip Chip6(DataIn6,Tack[1],Cmd,Clock,DataP[6],TReq[6],5'b0110,ResetH,BrdAddress);
pCT_Chip Chip7(DataIn7,Tack[1],Cmd,Clock,DataP[7],TReq[7],5'b0111,ResetH,BrdAddress);
pCT_Chip Chip8(DataIn8,Tack[1],Cmd,Clock,DataP[8],TReq[8],5'b1000,ResetH,BrdAddress);
pCT_Chip Chip9(DataIn9,Tack[1],Cmd,Clock,DataP[9],TReq[9],5'b1001,ResetH,BrdAddress);
pCT_Chip ChipA(DataInA,Tack[1],Cmd,Clock,DataP[10],TReq[10],5'b1010,ResetH,BrdAddress);
pCT_Chip ChipB(DataInB,Tack[1],Cmd,Clock,DataP[11],TReq[11],5'b1011,ResetH,BrdAddress);
          
endmodule

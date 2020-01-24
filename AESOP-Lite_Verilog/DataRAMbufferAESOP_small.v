// This program monitors the data output from a pCTFE64 test board FPGA and writes the data into a 65kByte RAM
// buffer (block RAM) whenever it encouters a data packet indicated by a start bit.  One state machine
// monitors the data stream to calculate when the data packet starts and finishes to turn on and off
// the writing to RAM.  Another state machine writes a byte to RAM every time it accumulates 8 bits.
// A third state machine can be activated to dump the data out of the RAM one byte at a time, intended
// to go to a UART for slow transmission to the PC.  The first two bytes sent out give the number of
// data bytes to follow, encoded as a 16-bit integer, MSB sent first.  Once the RAM is read out it is
// reset such that further data to come will overwrite the old data.
// R. Johnson  10/22/2012
// This version does not try to parse the incoming data stream.  It simply looks for a start bit to begin
// filling the RAM, and it turns off the RAM filling if a long string of zeroes is encountered.
// 8/25/2015   Repurpose this code for AESOP-Lite, to run in the Spartan-6.  Revise to stop recording if
// more than 8 zero bytes are received, and then to back up the pointer to the end of the data, so that
// the zeroes are not transmitted.  Also reduced the memory size.
//
module DataRAMbufferAESOP(TxD_start, TxD_data, DMPDone, DMPStart, TxD_busy, SerialData, Clock, Reset, Done, Error);
input Reset;                 //System initialization reset 
input Clock;                 //Fast system clock
output TxD_start;            //Signal start of UART output
output [7:0] TxD_data;       //UART output byte
input TxD_busy;              //UART transmitter busy signal
input SerialData;            //Serial data stream coming in
input DMPStart;              //Signal to start dumping the RAM buffer to the UART
output DMPDone;              //Signal that the RAM dump is complete
output Done;                 //Signal 1 clock long when done capturing data
output [2:0] Error;          //Set if an overflow error occurs

reg DMPDone, TxD_start;
reg [7:0] TxD_data;
reg [2:0] Error;

// State machine to write the input stream into block RAM.
// Look for a start bit, and then start storing bytes.
// Mark the first byte of any string of zero bytes.  If more
// than 8 zero bytes occur in a row, stop storing data, set
// the end pointer back to the first zero byte, and send a
// done signal.
//Definition of states:
parameter [3:0] QR0= 4'b0001;  //Wait for a start signal
parameter [3:0] QR1= 4'b0010;  //Shift in 8 bits
parameter [3:0] QR2= 4'b0100;  //Write a byte to RAM
parameter [3:0] QR3= 4'b1000;  //Wait for data to be dumped

reg [3:0] StateRAM, NextStateRAM;
reg RAMStart, RAMStop;  //Control signals from outside this process
reg [7:0] EndData, WriteAdr;
reg [7:0] RAMbyte;  //One byte to write to memory
wire [7:0] doa;     //RAM data out from port a
reg [2:0] CntByte;  //Counter for filling one byte
reg ena;            //Read enable memory port a
reg wea;            //Memory write enable
reg [2:0] CntZero;  //Count sequential bytes that are all zeroes
reg Done;

always @ (posedge Clock) begin
//    if (StateRAM == QR0 && NextStateRAM == QR1) $display("time StateRAM SerialData WriteAdr CntZero CntByte RAMbyte       EndData Done");
//    if (StateRAM != QR0) $display("%g\t %b     %b      %d       %d      %d      %b     %d      %b",$time, StateRAM, SerialData, WriteAdr, CntZero, CntByte, RAMbyte, EndData, Done);
end

always @ (StateRAM or RAMStart or RAMStop or CntByte or WriteAdr or SerialData or CntZero or RAMbyte or DMPDone)
begin
  case (StateRAM)
    QR0: begin
            if (SerialData) NextStateRAM = QR1;  //Waiting here for a start bit
            else NextStateRAM = QR0;
            ena = 0;
            wea = 0;
         end
    QR1: begin
            if (WriteAdr >= 8'hFA) NextStateRAM = QR3;  //Avoid overflow and overwriting of the RAM
            else if (CntByte == 7) NextStateRAM = QR2;
            else NextStateRAM = QR1;
            ena = 0;
            wea = 0;
         end
    QR2: begin
            if (RAMbyte == 0 && CntZero >= 7) NextStateRAM = QR3;
            else NextStateRAM = QR1;
            ena = 1;
            wea = 1;      //Write the byte into memory on the next clock edge
         end
    QR3: begin
            if (DMPDone) NextStateRAM = QR0;
            else NextStateRAM = QR3;
            ena = 0;
            wea = 0;
         end
    default: begin NextStateRAM = QR0; ena = 0; wea = 0; end
  endcase
end

always @ (posedge Clock)
begin
  if (Reset) begin
    Error <= 0;
    StateRAM <= QR0;
    WriteAdr <= 0;
    Done <= 1'b0;
  end else begin
    StateRAM <= NextStateRAM;
//    if (StateRAM == QR0 && NextStateRAM == QR1) $display("time RAM-In StateRAM WriteAdr CntByte CntZero Done RAMbyte SerialData");
//    if (StateRAM != QR0) $display("%g\t RAM-In %b      %h      %d       %d     %b   %b   %b ",$time,StateRAM,WriteAdr,CntByte,CntZero,Done,RAMbyte,SerialData);
    case (StateRAM)
        QR0: begin
                if (SerialData && WriteAdr > 0) Error[0] <= 1'b1;  // Write address failed to be reset
                CntByte <= 0;
                CntZero <= 0;
             end
        QR1: begin
                if (WriteAdr >= 8'hFA) begin        // RAM buffer overflow
                    Done <= 1'b1;
                    Error[1] <= 1'b1;
                end
                RAMbyte <= {RAMbyte[6:0],SerialData};
                CntByte <= CntByte + 1;
             end
        QR2: begin
                RAMbyte <= {RAMbyte[6:0],SerialData};
                if (RAMbyte == 0) begin
                    if (CntZero == 0) begin
                        EndData <= WriteAdr;
                        WriteAdr <= WriteAdr + 1;
                    end else if (CntZero >= 7) begin
                        WriteAdr <= EndData;
                        Done <= 1'b1;
                    end else begin
                        WriteAdr <= WriteAdr + 1;
                    end
                    CntZero <= CntZero + 1;
                end else begin
                    CntZero <= 0;
                    WriteAdr <= WriteAdr + 1;
                end
                CntByte <= CntByte + 1;                
             end
        QR3: begin
                Done <= 1'b0;
                if (SerialData) Error[2] <= 1'b1;   // More data coming in before the last has been dumped to the UART
                if (DMPDone) WriteAdr <= 0;
             end
    endcase
  end
end


//State machine to dump the RAM to the UART
//Definition of states:
parameter [6:0] QD0= 7'b0000001;  //Wait for a start signal
parameter [6:0] QD1= 7'b0000010;  //Wait for the Tx to be ready
parameter [6:0] QD2= 7'b0000100;  //Write the first (most significant) byte of the header
parameter [6:0] QD3= 7'b0001000;  //Wait for the Tx to be ready
parameter [6:0] QD6= 7'b0010000;  //Wait for the Tx to be ready
parameter [6:0] QD7= 7'b0100000;  //Write a byte from RAM to Tx
parameter [6:0] QD8= 7'b1000000;  //Signal done

reg enb;
reg [7:0] ReadAdr;
wire [7:0] dob;
reg [6:0] StateDMP, NextStateDMP;
reg [2:0] CntDly;

always @ (StateDMP or CntDly or DMPStart or TxD_busy or ReadAdr or WriteAdr or dob)
begin
    case (StateDMP)
        QD0: begin
                if (DMPStart) NextStateDMP = QD1;
                else NextStateDMP = QD0;
                enb = 0;
                TxD_data = dob;
             end
        QD1: begin
                enb = 0;
                TxD_data = WriteAdr;
                if (TxD_busy) NextStateDMP = QD1;
                else NextStateDMP = QD2;
             end
        QD2: begin
                enb = 0;
                TxD_data = WriteAdr;
                if (CntDly == 4) NextStateDMP=QD3;
                else NextStateDMP=QD2;
             end
        QD3: begin
                TxD_data = WriteAdr;
                if (WriteAdr != 0) begin
                    if (TxD_busy) begin
                        NextStateDMP = QD3;
                        enb = 0;
                    end else begin
                        NextStateDMP = QD7;
                        enb = 1;    // get the next byte from memory
                    end
                end else begin  // empty event!
                    NextStateDMP = QD8;
                    enb = 0;
                end
             end
        QD6: begin
                TxD_data = dob;
                if (TxD_busy) begin
                    NextStateDMP = QD6;
                    enb = 0;
                end else begin 
                    NextStateDMP = QD7;
                    enb = 1;    // get the next byte from memory
                end
             end
        QD7: begin
                enb = 0;
                if (CntDly == 3) begin
                  if (ReadAdr == WriteAdr-1) NextStateDMP = QD8;
                  else NextStateDMP = QD6;
                end else NextStateDMP = QD7;
                TxD_data = dob;
             end
        QD8: begin
                enb = 0;
                NextStateDMP = QD0;
                TxD_data = dob;
             end
        default: begin
                    enb = 0;
                    NextStateDMP = QD0;
                    TxD_data = dob;
                 end
    endcase
end

always @ (posedge Clock)
begin
    if (Reset) begin
        StateDMP <= QD0;
        ReadAdr <= 0;
        TxD_start <= 1'b0;
    end else begin
 //       if (DMPStart) $display("%g\t RAMbuffer StateDMP    TxD_busy TxD_start     ReadAdr    TxD_data   WriteAdr ena enb StateRAM",$time);
 //       if (StateDMP != QD0) $display("%g\t RAMbuffer %b         %b      %b            %h         %b  %b   %b  %b    %b",$time,StateDMP,TxD_busy,TxD_start,ReadAdr,TxD_data,WriteAdr,ena,enb,StateRAM);
        StateDMP <= NextStateDMP;
        case (StateDMP)
            QD0: begin
                    DMPDone <= 1'b0;
                    TxD_start <= 1'b0;
                 end
            QD1: begin
                    CntDly <= 0;
                    if (!TxD_busy) TxD_start <= 1'b1;
                 end
            QD2: begin
                    CntDly <= CntDly + 1;
                 end
            QD3: begin
                    TxD_start <= 1'b0;
                    CntDly <= 0;
                 end
            QD6: begin
                    TxD_start <= 1'b0;
                    CntDly <= 0;
                 end
            QD7: begin
                   if (!TxD_busy) begin
                       TxD_start <= 1'b1;
                   end
                   if (TxD_start) CntDly <= CntDly + 1;
                   if (CntDly == 3) begin
                      ReadAdr <= ReadAdr + 1;
                      CntDly <= 0;
                   end
                 end
            QD8: begin
                    TxD_start <= 1'b0;
                    DMPDone <= 1'b1;
                    ReadAdr <= 0;
                 end
        endcase
    end
end

DataOutRAM DataOutRAM_U(Clock, Clock, ena, enb, wea, WriteAdr, ReadAdr, RAMbyte, doa, dob);   //Instantiate the memory here

endmodule
    
//Dual ported RAM with one write port:
module DataOutRAM (clka, clkb, ena, enb, wea, addra, addrb, dia, doa, dob);
input clka, clkb;
input wea;                  //Write enable
input ena, enb;             //Primary and secondary read enables
input [7:0] addra, addrb;  //Addresses for the primary and secondary ports
input [7:0] dia;               //Input data register to write to memory
output [7:0] doa, dob;         //Output registers for the two ports
reg [7:0] RAM [255:0];     //Memory array
reg [7:0] dob, doa;

always @(posedge clka)
begin
    if (ena)
    begin
        if (wea) begin
            RAM[addra]<=dia;  
            doa <= dia;
        end else doa <= RAM[addra];        //Write first, then read
        //$display("%g\t RAM:  write %b to address %h",$time,dia,addra);
    end
end

always @(posedge clkb)
begin
    if (enb)
    begin
        dob <= RAM[addrb];
        //$display("%g\t RAM: read %b from address %h",$time,RAM[addrb],addrb);
    end
end
endmodule

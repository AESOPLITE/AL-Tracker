// Routine called when the AESOP-Lite ASICs have been powered on to put them into
// a better default state than their native power-up default.
// In fact, the idea is to program each board with the best operational state
// prior to flight, so that no ASIC configuration is needed upon a reboot.
// R. Johnson  6/17/2016
// Increased output drive in configuration register to 10   8/4/2016
// Add the trigger mask to the initialization   2/7/2017
// Set the trigger masks and raised some thresholds according to Brians scans  10/17/2017
// Raised all of the thresholds to 5 times noise 12/4/2017
// Final optimized thresholds and masks for flight  1/23/2018
// Added F board values and increased the ASIC current drive  3/19/2020
module TKRinitial(Clock, Reset, Go, CMD, Done, Version);

input Clock;           // 10 MHz system clock
input Reset;           // The Reset must be activated before the first Go signal
input Go;              // 1-clock signal to start sending out the command stream
output CMD;            // Output command stream to be directed to the ASICs
output Done;           // 1-clock signal that the process has been completed
output [7:0] Version;  // Identifies the board being configured

reg Done;
reg CMD;

reg [7:0] Version;
reg [18:0] ConFigReg;   // Default configuration register setting

reg [7:0] ThrDAC0;      // Default threshold DAC settings
reg [7:0] ThrDAC1;
reg [7:0] ThrDAC2;
reg [7:0] ThrDAC3;
reg [7:0] ThrDAC4;
reg [7:0] ThrDAC5;
reg [7:0] ThrDAC6;
reg [7:0] ThrDAC7;
reg [7:0] ThrDAC8;
reg [7:0] ThrDAC9;
reg [7:0] ThrDACa;
reg [7:0] ThrDACb;

reg [63:0] DataMSK0;     // Default data mask settings
reg [63:0] DataMSK1;
reg [63:0] DataMSK2;
reg [63:0] DataMSK3;
reg [63:0] DataMSK4;
reg [63:0] DataMSK5;
reg [63:0] DataMSK6;
reg [63:0] DataMSK7;
reg [63:0] DataMSK8;
reg [63:0] DataMSK9;
reg [63:0] DataMSKa;
reg [63:0] DataMSKb;

reg [63:0] TrigMSK0;     // Default data mask settings
reg [63:0] TrigMSK1;
reg [63:0] TrigMSK2;
reg [63:0] TrigMSK3;
reg [63:0] TrigMSK4;
reg [63:0] TrigMSK5;
reg [63:0] TrigMSK6;
reg [63:0] TrigMSK7;
reg [63:0] TrigMSK8;
reg [63:0] TrigMSK9;
reg [63:0] TrigMSKa;
reg [63:0] TrigMSKb;

reg [11:0] PrtyDAC;      // Parity bits for the 12 DAC settings
reg [11:0] PrtyMSK;      // Parity bits for the 12 data mask settings
reg [11:0] PrtyMSKT;     // Parity bits for the 12 trigger mask settings
reg PrtyCfg;             // Parity bit for the configuration register setting

parameter [8:0] Wait = 9'b000000001;     // Wait for the Go signal
parameter [8:0] CNFG = 9'b000000010;     // Load the configuration registers of all chips
parameter [8:0] Wat2 = 9'b000000100;     // Pause before sending the next command
parameter [8:0] DACW = 9'b000001000;     // Load the threshold DAC register
parameter [8:0] Wat3 = 9'b000010000;     // Pause
parameter [8:0] MSKW = 9'b000100000;     // Load the data mask registers
parameter [8:0] Wat4 = 9'b001000000;     // Pause
parameter [8:0] MSKT = 9'b010000000;     // Load the trigger mask registers
parameter [8:0] ENDS = 9'b100000000;     // Done

reg [8:0] State, NextState;
reg [6:0] Cnt;
reg [3:0] CntWt;
reg [3:0] CntChip;
reg [10:0] CmdHead;
reg [20:0] CmdDAC;
reg [76:0] CmdMSK;

always @ (State or Cnt or Go or CmdHead or CmdDAC or CntChip or CntWt or CmdMSK) begin
    case (State) 
        Wait: begin
                  if (Go) NextState = CNFG;
                  else NextState = Wait;
                  CMD = 1'b0;
              end
        CNFG: begin
                  CMD = CmdHead[10];
                  if (Cnt == 29) NextState = Wat2;
                  else NextState = CNFG;
              end
        Wat2: begin
                  if (CntWt == 4'hf) NextState = DACW;
                  else NextState = Wat2;
                  CMD = 1'b0;
              end
        DACW: begin
                  if (Cnt == 20) begin
                      if (CntChip == 11) NextState = Wat3;
                      else NextState = Wat2;
                  end else NextState = DACW;
                  CMD = CmdDAC[20];
              end
        Wat3: begin
                  if (CntWt == 4'hf) NextState = MSKW;
                  else NextState = Wat3;
                  CMD = 1'b0;
              end
        MSKW: begin
                  CMD = CmdMSK[76];
                  if (Cnt == 76) begin
                      if (CntChip == 11) NextState = Wat4;
                      else NextState = Wat3;
                  end else NextState = MSKW;
              end
		Wat4: begin
                  if (CntWt == 4'hf) NextState = MSKT;
                  else NextState = Wat4;
                  CMD = 1'b0;		
		      end
	    MSKT: begin 
                  CMD = CmdMSK[76];
                  if (Cnt == 76) begin
                      if (CntChip == 11) NextState = ENDS;
                      else NextState = Wat4;
                  end else NextState = MSKT;
		      end
        ENDS: begin
                  NextState = Wait;
                  CMD = 1'b0;
              end  
        default: begin
                     NextState = Wait;
                     CMD = 1'b0;
                 end
    endcase
end

always @ (posedge Clock) begin
  if (Reset) begin
     State <= Wait;   
     ConFigReg <= 19'b1010111001000111000;   // All chips on all boards need to have the same configuration register setting
     PrtyCfg <= 1'b1;                        // Don't forget to set the parity bits when changing register defaults
     
	 // Each chip in general will have a different threshold DAC setting, for S/N optimization
     // Board A
/*
     Version <= 8'h41;
     ThrDAC0 <= {1'b0,7'd22};                
     ThrDAC1 <= {1'b0,7'd25};
     ThrDAC2 <= {1'b0,7'd22};
     ThrDAC3 <= {1'b0,7'd21};
     ThrDAC4 <= {1'b0,7'd25};
     ThrDAC5 <= {1'b0,7'd25};
     ThrDAC6 <= {1'b0,7'd25};
     ThrDAC7 <= {1'b0,7'd22};
     ThrDAC8 <= {1'b0,7'd26};
     ThrDAC9 <= {1'b0,7'd26};
     ThrDACa <= {1'b0,7'd23};
     ThrDACb <= {1'b0,7'd24};
     PrtyDAC <= 12'b001111111111; 
*/
 
     // Board B
 /*    Version <= 8'h42;
     ThrDAC0 <= {1'b0,7'd20};                
     ThrDAC1 <= {1'b0,7'd20};
     ThrDAC2 <= {1'b0,7'd21};
     ThrDAC3 <= {1'b0,7'd22};
     ThrDAC4 <= {1'b0,7'd22};
     ThrDAC5 <= {1'b0,7'd24};
     ThrDAC6 <= {1'b0,7'd26};
     ThrDAC7 <= {1'b0,7'd24};
     ThrDAC8 <= {1'b0,7'd23};
     ThrDAC9 <= {1'b0,7'd24};
     ThrDACa <= {1'b0,7'd24};
     ThrDACb <= {1'b0,7'd23};
     PrtyDAC <= 12'b000001011100;
 */    
     // Board C
 /*    Version <= 8'h43;
     ThrDAC0 <= {1'b0,7'd21};                
     ThrDAC1 <= {1'b0,7'd22};
     ThrDAC2 <= {1'b0,7'd22};
     ThrDAC3 <= {1'b0,7'd21};
     ThrDAC4 <= {1'b0,7'd23};
     ThrDAC5 <= {1'b0,7'd25};
     ThrDAC6 <= {1'b0,7'd24};
     ThrDAC7 <= {1'b0,7'd22};
     ThrDAC8 <= {1'b0,7'd22};
     ThrDAC9 <= {1'b0,7'd22};
     ThrDACa <= {1'b0,7'd23};
     ThrDACb <= {1'b0,7'd21};
     PrtyDAC <= 12'b101110101111;     
*/
     // Board D
/*     Version <= 8'h44;
     ThrDAC0 <= {1'b0,7'd22};                
     ThrDAC1 <= {1'b0,7'd22};
     ThrDAC2 <= {1'b0,7'd20};
     ThrDAC3 <= {1'b0,7'd20};
     ThrDAC4 <= {1'b0,7'd23};
     ThrDAC5 <= {1'b0,7'd22};
     ThrDAC6 <= {1'b0,7'd23};
     ThrDAC7 <= {1'b0,7'd21};
     ThrDAC8 <= {1'b0,7'd24};
     ThrDAC9 <= {1'b0,7'd23};
     ThrDACa <= {1'b0,7'd22};
     ThrDACb <= {1'b0,7'd23};
     PrtyDAC <= 12'b010010100011;
*/
     // Board E
 /*    Version <= 8'h45;
     ThrDAC0 <= {1'b0,7'd24};                
     ThrDAC1 <= {1'b0,7'd21};
     ThrDAC2 <= {1'b0,7'd23};
     ThrDAC3 <= {1'b0,7'd21};
     ThrDAC4 <= {1'b0,7'd22};
     ThrDAC5 <= {1'b0,7'd21};
     ThrDAC6 <= {1'b0,7'd22};
     ThrDAC7 <= {1'b0,7'd25};
     ThrDAC8 <= {1'b0,7'd22};
     ThrDAC9 <= {1'b0,7'd22};
     ThrDACa <= {1'b0,7'd23};
     ThrDACb <= {1'b0,7'd25};
     PrtyDAC <= 12'b101111111010; 
*/  
/*
     // Board F, from scan Feb_3_2020
     Version <= 8'h46;
     ThrDAC0 <= {1'b0,7'd24};                
     ThrDAC1 <= {1'b0,7'd22};
     ThrDAC2 <= {1'b0,7'd21};
     ThrDAC3 <= {1'b0,7'd22};
     ThrDAC4 <= {1'b0,7'd24};
     ThrDAC5 <= {1'b0,7'd23};
     ThrDAC6 <= {1'b0,7'd23};
     ThrDAC7 <= {1'b0,7'd22};
     ThrDAC8 <= {1'b0,7'd21};
     ThrDAC9 <= {1'b0,7'd23};
     ThrDACa <= {1'b0,7'd23};
     ThrDACb <= {1'b0,7'd26};
     PrtyDAC <= 12'b100110001110; 
*/  
     // Board G
     Version <= 8'h47;
     ThrDAC0 <= {1'b0,7'd22};                
     ThrDAC1 <= {1'b0,7'd22};
     ThrDAC2 <= {1'b0,7'd21};
     ThrDAC3 <= {1'b0,7'd20};
     ThrDAC4 <= {1'b0,7'd22};
     ThrDAC5 <= {1'b0,7'd24};
     ThrDAC6 <= {1'b0,7'd20};
     ThrDAC7 <= {1'b0,7'd24};
     ThrDAC8 <= {1'b0,7'd21};
     ThrDAC9 <= {1'b0,7'd23};
     ThrDACa <= {1'b0,7'd23};
     ThrDACb <= {1'b0,7'd22};
     PrtyDAC <= 12'b100100010111; 

     // Board H

     /*Version <= 8'h48;
     ThrDAC0 <= {1'b0,7'd23};                
     ThrDAC1 <= {1'b0,7'd23};
     ThrDAC2 <= {1'b0,7'd23};
     ThrDAC3 <= {1'b0,7'd25};
     ThrDAC4 <= {1'b0,7'd26};
     ThrDAC5 <= {1'b0,7'd23};
     ThrDAC6 <= {1'b0,7'd25};
     ThrDAC7 <= {1'b0,7'd24};
     ThrDAC8 <= {1'b0,7'd23};
     ThrDAC9 <= {1'b0,7'd23};
     ThrDACa <= {1'b0,7'd25};
     ThrDACb <= {1'b0,7'd24};
     PrtyDAC <= 12'b010001011000;  
*/
/*
     // Board I
     Version <= 8'h49;
     ThrDAC0 <= {1'b0,7'd22};                
     ThrDAC1 <= {1'b0,7'd23};
     ThrDAC2 <= {1'b0,7'd22};
     ThrDAC3 <= {1'b0,7'd20};
     ThrDAC4 <= {1'b0,7'd23};
     ThrDAC5 <= {1'b0,7'd23};
     ThrDAC6 <= {1'b0,7'd22};
     ThrDAC7 <= {1'b0,7'd24};
     ThrDAC8 <= {1'b0,7'd24};
     ThrDAC9 <= {1'b0,7'd22};
     ThrDACa <= {1'b0,7'd24};
     ThrDACb <= {1'b0,7'd22};
     PrtyDAC <= 12'b101001000101;     
*/ 
     DataMSK0 <= 64'hffffffffffffffff;   // Here is where individual channels can be masked out from data
     DataMSK1 <= 64'hffffffffffffffff;
     DataMSK2 <= 64'hffffffffffffffff;
     DataMSK3 <= 64'hffffffffffffffff;
     DataMSK4 <= 64'hffffffffffffffff;
     DataMSK5 <= 64'hffffffffffffffff;
     DataMSK6 <= 64'hffffffffffffffff;
     DataMSK7 <= 64'hffffffffffffffff;
     DataMSK8 <= 64'hffffffffffffffff;
     DataMSK9 <= 64'hffffffffffffffff;
     DataMSKa <= 64'hffffffffffffffff;
     DataMSKb <= 64'hffffffffffffffff;
     PrtyMSK <= 12'b000000000000;
	 
// Here is where individual channels can be masked out from trigger
//On all boards we mask the first and last channel, since they tend to be noisy and are not important
	 
     // Board C trigger mask. Chip 9, channel 17 is noisy
/*
	 TrigMSK0 <= 64'hfffffffffffffffe;   
     TrigMSK1 <= 64'hffffffffffffffff;
     TrigMSK2 <= 64'hffffffffffffffff;
     TrigMSK3 <= 64'hffffffffffffffff;
     TrigMSK4 <= 64'hffffffffffffffff;
     TrigMSK5 <= 64'hffffffffffffffff;
     TrigMSK6 <= 64'hffffffffffffffff;
     TrigMSK7 <= 64'hffffffffffffffff;
     TrigMSK8 <= 64'hffffffffffffffff;
     TrigMSK9 <= 64'hFFFFBFFFFFFFFFFF;
     TrigMSKa <= 64'hffffffffffffffff;
     TrigMSKb <= 64'h7fffffffffffffff;
     PrtyMSKT <= 12'b101000000001;
*/      
	 // Board E trigger mask.  Chip 5, channels 25 and 26 are very noisy
/*
     TrigMSK0 <= 64'hfffffffffffffffe;   
     TrigMSK1 <= 64'hffffffffffffffff;
     TrigMSK2 <= 64'hffffffffffffffff;
     TrigMSK3 <= 64'hffffffffffffffff;
     TrigMSK4 <= 64'hffffffffffffffff;
     TrigMSK5 <= 64'hffffff9fffffffff;
     TrigMSK6 <= 64'hffffffffffffffff;
     TrigMSK7 <= 64'hffffffffffffffff;
     TrigMSK8 <= 64'hffffffffffffffff;
     TrigMSK9 <= 64'hffffffffffffffff;
     TrigMSKa <= 64'hffffffffffffffff;
     TrigMSKb <= 64'h7fffffffffffffff;
     PrtyMSKT <= 12'b100000000001; 
*/	
/*
     // Board I trigger mask.  Chip 8 channel 52 is very noisy.
     TrigMSK0 <= 64'hfffffffffffffffe;   
     TrigMSK1 <= 64'hffffffffffffffff;
     TrigMSK2 <= 64'hffffffffffffffff;
     TrigMSK3 <= 64'hffffffffffffffff;
     TrigMSK4 <= 64'hffffffffffffffff;
     TrigMSK5 <= 64'hffffffffffffffff;
     TrigMSK6 <= 64'hffffffffffffffff;
     TrigMSK7 <= 64'hffffffffffffffff;
     TrigMSK8 <= 64'hfffffffffffff7ff;
     TrigMSK9 <= 64'hffffffffffffffff;
     TrigMSKa <= 64'hffffffffffffffff;
     TrigMSKb <= 64'h7fffffffffffffff;
     PrtyMSKT <= 12'b100100000001;
*/	
     // Board H trigger mask, chip  6, channels 29,30,37,53 are very noisy
/*
     TrigMSK0 <= 64'hfffffffffffffffe;   
     TrigMSK1 <= 64'hffffffffffffffff;
     TrigMSK2 <= 64'hffffffffffffffff;
     TrigMSK3 <= 64'hffffffffffffffff;
     TrigMSK4 <= 64'hffffffffffffffff;
     TrigMSK5 <= 64'hffffffffffffffff;
     TrigMSK6 <= 64'hfffffff9fffffbff;
     TrigMSK7 <= 64'hffffffffffffffff;
     TrigMSK8 <= 64'hffffffffffffffff;
     TrigMSK9 <= 64'hffffffffffffffff;
     TrigMSKa <= 64'hffffffffffffffff;
     TrigMSKb <= 64'h7fffffffffffffff;
     PrtyMSKT <= 12'b100001000001;
*/  
     //Other boards, mask off only the first and last channels:
 
     TrigMSK0 <= 64'hfffffffffffffffe;   
     TrigMSK1 <= 64'hffffffffffffffff;
     TrigMSK2 <= 64'hffffffffffffffff;
     TrigMSK3 <= 64'hffffffffffffffff;
     TrigMSK4 <= 64'hffffffffffffffff;
     TrigMSK5 <= 64'hffffffffffffffff;
     TrigMSK6 <= 64'hffffffffffffffff;
     TrigMSK7 <= 64'hffffffffffffffff;
     TrigMSK8 <= 64'hffffffffffffffff;
     TrigMSK9 <= 64'hffffffffffffffff;
     TrigMSKa <= 64'hffffffffffffffff;
     TrigMSKb <= 64'h7fffffffffffffff;
     PrtyMSKT <= 12'b100000000001;

	 
  end else begin
     if (Go || State != Wait) $display("%g\t TKRinitial, State=%b, Cnt=%d, CntChip=%d, Done=%b, CMD=%b, CntWt=%d, CmdDAC=%b",$time,State,Cnt,CntChip,Done,CMD,CntWt,CmdDAC);
     State <= NextState;
     case (State)
         Wait: begin
                   Cnt <= 0;
                   CmdHead <= {10'b1111111011,PrtyCfg};
                   CntChip <= 0;
                   Done <= 1'b0;
               end
         CNFG: begin
                   CmdHead <= {CmdHead[9:0],ConFigReg[18]};
                   ConFigReg <= {ConFigReg[17:0],CmdHead[10]};
                   Cnt <= Cnt + 1;
                   CntWt <= 0;
               end
         Wat2: begin
                   Cnt <= 0;
                   case (CntChip)
                       4'h0: CmdDAC <= {2'b10,CntChip,4'b1010,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyDAC[0],ThrDAC0,2'b00};
                       4'h1: CmdDAC <= {2'b10,CntChip,4'b1010,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyDAC[1],ThrDAC1,2'b00};
                       4'h2: CmdDAC <= {2'b10,CntChip,4'b1010,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyDAC[2],ThrDAC2,2'b00};
                       4'h3: CmdDAC <= {2'b10,CntChip,4'b1010,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyDAC[3],ThrDAC3,2'b00};
                       4'h4: CmdDAC <= {2'b10,CntChip,4'b1010,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyDAC[4],ThrDAC4,2'b00};
                       4'h5: CmdDAC <= {2'b10,CntChip,4'b1010,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyDAC[5],ThrDAC5,2'b00};
                       4'h6: CmdDAC <= {2'b10,CntChip,4'b1010,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyDAC[6],ThrDAC6,2'b00};
                       4'h7: CmdDAC <= {2'b10,CntChip,4'b1010,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyDAC[7],ThrDAC7,2'b00};
                       4'h8: CmdDAC <= {2'b10,CntChip,4'b1010,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyDAC[8],ThrDAC8,2'b00};
                       4'h9: CmdDAC <= {2'b10,CntChip,4'b1010,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyDAC[9],ThrDAC9,2'b00};
                       4'ha: CmdDAC <= {2'b10,CntChip,4'b1010,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyDAC[10],ThrDACa,2'b00};
                       4'hb: CmdDAC <= {2'b10,CntChip,4'b1010,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyDAC[11],ThrDACb,2'b00};
                   endcase
                   CntWt <= CntWt + 1;
               end
         DACW: begin
                   CmdDAC <= {CmdDAC[19:0],1'b0};
                   Cnt <= Cnt + 1;
                   if (Cnt == 20) begin
                       if (CntChip == 11) CntChip <= 0;
                       else CntChip <= CntChip + 1;
                   end
                   CntWt <= 0;
               end
         Wat3: begin
                   Cnt <= 0;
                   case (CntChip)
                       4'h0: CmdMSK <= {2'b10,CntChip,4'b1100,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSK[0],DataMSK0,2'b00};
                       4'h1: CmdMSK <= {2'b10,CntChip,4'b1100,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSK[1],DataMSK1,2'b00};
                       4'h2: CmdMSK <= {2'b10,CntChip,4'b1100,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSK[2],DataMSK2,2'b00};
                       4'h3: CmdMSK <= {2'b10,CntChip,4'b1100,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSK[3],DataMSK3,2'b00};
                       4'h4: CmdMSK <= {2'b10,CntChip,4'b1100,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSK[4],DataMSK4,2'b00};
                       4'h5: CmdMSK <= {2'b10,CntChip,4'b1100,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSK[5],DataMSK5,2'b00};
                       4'h6: CmdMSK <= {2'b10,CntChip,4'b1100,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSK[6],DataMSK6,2'b00};
                       4'h7: CmdMSK <= {2'b10,CntChip,4'b1100,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSK[7],DataMSK7,2'b00};
                       4'h8: CmdMSK <= {2'b10,CntChip,4'b1100,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSK[8],DataMSK8,2'b00};
                       4'h9: CmdMSK <= {2'b10,CntChip,4'b1100,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSK[9],DataMSK9,2'b00};
                       4'ha: CmdMSK <= {2'b10,CntChip,4'b1100,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSK[10],DataMSKa,2'b00};
                       4'hb: CmdMSK <= {2'b10,CntChip,4'b1100,CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSK[11],DataMSKb,2'b00};
                   endcase
                   CntWt <= CntWt + 1;
               end  	   
         MSKW: begin
                   CmdMSK <= {CmdMSK[75:0],1'b0};
                   Cnt <= Cnt + 1;
                   if (Cnt == 76) begin
                       CntChip <= CntChip + 1;
                   end
                   CntWt <= 0;
               end 
         Wat4: begin
                   Cnt <= 0;
                   case (CntChip)
                       4'h0: CmdMSK <= {2'b10,CntChip,4'b1101,!CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSKT[0],TrigMSK0,2'b00};
                       4'h1: CmdMSK <= {2'b10,CntChip,4'b1101,!CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSKT[1],TrigMSK1,2'b00};
                       4'h2: CmdMSK <= {2'b10,CntChip,4'b1101,!CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSKT[2],TrigMSK2,2'b00};
                       4'h3: CmdMSK <= {2'b10,CntChip,4'b1101,!CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSKT[3],TrigMSK3,2'b00};
                       4'h4: CmdMSK <= {2'b10,CntChip,4'b1101,!CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSKT[4],TrigMSK4,2'b00};
                       4'h5: CmdMSK <= {2'b10,CntChip,4'b1101,!CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSKT[5],TrigMSK5,2'b00};
                       4'h6: CmdMSK <= {2'b10,CntChip,4'b1101,!CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSKT[6],TrigMSK6,2'b00};
                       4'h7: CmdMSK <= {2'b10,CntChip,4'b1101,!CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSKT[7],TrigMSK7,2'b00};
                       4'h8: CmdMSK <= {2'b10,CntChip,4'b1101,!CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSKT[8],TrigMSK8,2'b00};
                       4'h9: CmdMSK <= {2'b10,CntChip,4'b1101,!CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSKT[9],TrigMSK9,2'b00};
                       4'ha: CmdMSK <= {2'b10,CntChip,4'b1101,!CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSKT[10],TrigMSKa,2'b00};
                       4'hb: CmdMSK <= {2'b10,CntChip,4'b1101,!CntChip[0]^CntChip[1]^CntChip[2]^CntChip[3]^PrtyMSKT[11],TrigMSKb,2'b00};
                   endcase
                   CntWt <= CntWt + 1;
               end		
	     MSKT: begin
                   CmdMSK <= {CmdMSK[75:0],1'b0};
                   Cnt <= Cnt + 1;
                   if (Cnt == 76) begin
                       CntChip <= CntChip + 1;
                   end	
		       end
         ENDS: begin
                   Done <= 1'b1;
               end         
     endcase
  end
end

endmodule


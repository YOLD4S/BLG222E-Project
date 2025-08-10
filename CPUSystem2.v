`timescale 1ns / 1ps

module CPUSystem (
    input wire Clock,
    input wire Reset,
    input reg[11:0] T
);
    wire ALU_WF;
    wire IR_Write;              /////
    wire Mem_WR;
    wire Mem_CS;                /////
    wire IR_LH;                 /////
    wire MuxDSel;
    wire DR_E;
    wire [1:0] MuxCSel;
    wire [1:0] ARF_OutCSel;
    wire [1:0] ARF_OutDSel;
    wire [1:0] ARF_FunSel;
    wire [1:0] MuxASel;
    wire [1:0] MuxBSel;
    wire [1:0] DR_FunSel;
    wire [2:0] RF_OutASel;
    wire [2:0] RF_OutBSel;
    wire [2:0] RF_FunSel;
    wire [2:0] ARF_RegSel;
    wire [3:0] RF_RegSel;
    wire [3:0] RF_ScrSel;
    wire [4:0] ALU_FunSel;

    wire T_Reset;
    wire [7:0] Address;          /////
    wire [5:0] Opcode;           /////
    wire [2:0] DestReg;          /////
    wire [2:0] SrcReg1;          /////
    wire [2:0] SrcReg2;          /////
    wire [1:0] RegSel;           /////




    assign IR_LH = T[1];
    assign IR_Write = T[0] && T[1];
    assign Opcode = IROut[15:10];
    assign DestReg = IROut[9:7];
    assign SrcReg1 = IROut[6:4];
    assign SrcReg2 = IROut[3:1];
    assign RegSel = IROut[9:8];
    assign Address = IROut[7:0];







    ArithmeticLogicUnitSystem ALUSys (
        .ALU_WF(ALU_WF),
        .IR_Write(IR_Write),
        .Mem_WR(Mem_WR),
        .Mem_CS(Mem_CS),
        .IR_LH(IR_LH),
        .MuxDSel(MuxDSel),
        .Clock(Clock),
        .DR_E(DR_E),
        .MuxCSel(MuxCSel),
        .ARF_OutCSel(ARF_OutCSel),
        .ARF_OutDSel(ARF_OutDSel),
        .ARF_FunSel(ARF_FunSel),
        .MuxASel(MuxASel),
        .MuxBSel(MuxBSel),
        .DR_FunSel(DR_FunSel),
        .RF_OutASel(RF_OutASel),
        .RF_OutBSel(RF_OutBSel),
        .RF_FunSel(RF_FunSel),
        .ARF_RegSel(ARF_RegSel),
        .RF_RegSel(RF_RegSel),
        .RF_ScrSel(RF_ScrSel),
        .ALU_FunSel(ALU_FunSel)
    );
    // Reset functionality
    always @(*) begin
        if (Reset) begin
            CPUSys.T_Reset = 1;
            CPUSys.ALUSys.ARF.PC.Q = 16'h0000;
            CPUSys.ALUSys.ARF.AR.Q = 16'h0000;
            CPUSys.ALUSys.ARF.SP.Q = 16'h0000;
            CPUSys.ALUSys.IR.IROut = 16'h0000;
            CPUSys.ALUSys.DR.DROut = 32'h00000000;
            CPUSys.ALUSys.ALU.FlagsOut = 4'b0000;
            CPUSys.RF.R1.Q = 32'h00000000;
            CPUSys.RF.R2.Q = 32'h00000000;
            CPUSys.RF.R3.Q = 32'h00000000;
            CPUSys.RF.R4.Q = 32'h00000000;
            CPUSys.RF.S1.Q = 32'h00000000;
            CPUSys.RF.S2.Q = 32'h00000000;
            CPUSys.RF.S3.Q = 32'h00000000;
            CPUSys.RF.S4.Q = 32'h00000000;
        end
    end
endmodule


module ControlUnit (

);


endmodule



// module SequenceCounter (
//     input wire Clock,
//     input wire Reset,
//     output reg [11:0] T
// );
//     always @(posedge Clock or posedge Reset) begin
//         if (Reset) begin
//             T <= 12'b0;
//         end else begin
//             T <= T + 1;
//         end
//     end
// );


// endmodule

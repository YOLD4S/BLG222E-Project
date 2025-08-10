`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12.04.2025 15:39:59
// Design Name: 
// Module Name: ArithmeticLogicUnitSystem
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module ArithmeticLogicUnitSystem(
    input [2:0] RF_OutASel,
    input [2:0] RF_OutBSel, 
    input [2:0] RF_FunSel,   
    input [3:0] RF_RegSel,
    input [3:0] RF_ScrSel,    
    input [4:0] ALU_FunSel,
    input ALU_WF,           
    input [1:0] ARF_OutCSel, 
    input [1:0] ARF_OutDSel, 
    input [1:0] ARF_FunSel,
    input [2:0] ARF_RegSel,   
    input IR_LH,
    input IR_Write,       
    input Mem_WR,
    input Mem_CS,          
    input [1:0] MuxASel,
    input [1:0] MuxBSel,         
    input [1:0] MuxCSel,
    input MuxDSel,
    input [1:0] DR_FunSel,
    input DR_E,               
    input Clock         
    );

    wire [31:0] OutA, OutB, ALUOut, DROut;
    wire [15:0] OutC, OutD, IROut, Address;
    wire [7:0] MemOut;
    wire [3:0] FlagsOut;

    reg [31:0] MuxAOut, MuxBOut, MuxDOut;
    reg [7:0] MuxCOut;

    assign Address = OutD;

    RegisterFile RF(.Clock(Clock), .I(MuxAOut), .RegSel(RF_RegSel), .ScrSel(RF_ScrSel), .FunSel(RF_FunSel), .OutASel(RF_OutASel), .OutBSel(RF_OutBSel), .OutA(OutA), .OutB(OutB));
    AddressRegisterFile ARF(.Clock(Clock), .I(MuxBOut), .RegSel(ARF_RegSel), .FunSel(ARF_FunSel), .OutCSel(ARF_OutCSel), .OutDSel(ARF_OutDSel), .OutC(OutC), .OutD(OutD));
    ArithmeticLogicUnit ALU(.Clock(Clock), .A(MuxDOut), .B(OutB), .FunSel(ALU_FunSel), .WF(ALU_WF), .ALUOut(ALUOut), .FlagsOut(FlagsOut));
    Memory MEM(.Clock(Clock), .Address(OutD), .Data(MuxCOut), .WR(Mem_WR), .CS(Mem_CS), .MemOut(MemOut));
    DataRegister DR(.Clock(Clock), .E(DR_E), .FunSel(DR_FunSel), .I(MemOut), .DROut(DROut));
    InstructionRegister IR(.Clock(Clock), .Write(IR_Write), .LH(IR_LH), .I(MemOut), .IROut(IROut));

    always @(*) begin
        case (MuxASel)
            2'b00: MuxAOut = ALUOut;
            2'b01: MuxAOut = {16'b0, OutC};
            2'b10: MuxAOut = DROut;
            2'b11: MuxAOut = {24'b0, IROut[7:0]};
        endcase

        case (MuxBSel)
            2'b00: MuxBOut = ALUOut;
            2'b01: MuxBOut = {16'b0, OutC};
            2'b10: MuxBOut = DROut;
            2'b11: MuxBOut = {24'b0, IROut[7:0]};
        endcase

        case (MuxCSel)
            2'b00: MuxCOut = ALUOut[7:0];
            2'b01: MuxCOut = ALUOut[15:8];
            2'b10: MuxCOut = ALUOut[23:16];
            2'b11: MuxCOut = ALUOut[31:24];
        endcase

        if (MuxDSel) MuxDOut = {16'b0, OutC};
        else MuxDOut = OutA;
    end

endmodule

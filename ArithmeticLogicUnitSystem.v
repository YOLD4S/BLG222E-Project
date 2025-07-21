`timescale 1ns / 1ps

module ArithmeticLogicUnitSystem (
    input wire ALU_WF, IR_Write, Mem_WR, Mem_CS, IR_LH,
    input wire MuxDSel, Clock, DR_E,
    input wire [1:0] MuxCSel, ARF_OutCSel, ARF_OutDSel,ARF_FunSel,
    input wire [1:0] MuxASel, MuxBSel, DR_FunSel,
    input wire [2:0] RF_OutASel, RF_OutBSel, RF_FunSel, ARF_RegSel,
    input wire [3:0] RF_RegSel, RF_ScrSel,
    input wire [4:0] ALU_FunSel
);
    wire [31:0] MuxAOut, MuxBOut, RF_outA, RF_outB, MuxDOut, DROut;
    wire [15:0] ARF_outC, ARF_outD, ALU_out, IROut;
    wire [7:0] MuxCOut, MemOut;
    wire [3:0] ALU_FlagsOut;

    RegisterFile RF (
        .OutASel(RF_OutASel), .OutBSel(RF_OutBSel), 
        .FunSel(RF_FunSel), .RegSel(RF_RegSel),
        .ScrSel(RF_ScrSel), .Clock(Clock),
        .OutA(RF_outA), .OutB(RF_outB),
        .I(MuxAOut)
    );

    ArithmeticLogicUnit ALU (
        .A(MuxDOut),
        .B(RF_outB),
        .FunSel(ALU_FunSel),
        .WF(ALU_WF),
        .Clock(Clock),
        .ALUOut(ALU_out),
        .FlagsOut(ALU_FlagsOut)
    );

    ArithmeticRegisterFile ARF (
        .Clock(Clock),
        .I(MuxBOut),
        .RegSel(ARF_RegSel),
        .FunSel(ARF_FunSel),
        .OutCSel(ARF_OutCSel),
        .OutDSel(ARF_OutDSel),
        .OutC(ARF_outC),
        .OutD(ARF_outD)
    );

    DataRegister DR (
        .Clock(Clock),
        .E(DR_E),
        .FunSel(DR_FunSel),
        .I(MemOut),
        .DROut(DROut)
    );

    InstructionRegister IR (
        .Clock(Clock),
        .Write(IR_Write),
        .LH(IR_LH),
        .I(MemOut),
        .IROut(IROut)
    );

    Memory Mem (
        .Address(ARF_outD),
        .Data(MuxCOut),
        .WR(Mem_WR),
        .CS(Mem_CS),
        .Clock(Clock),
        .MemOut(MemOut)
    );

    Mux4to1_32bit MuxA (
        .in0(ALU_out), .in1({16'b0, ARF_outC}), .in2(DROut), .in3(IROut[7:0]),
        .sel(MuxASel), .out(MuxAOut)
    );

    Mux4to1_32bit MuxB (
        .in0(ALU_out), .in1({16'b0, ARF_outC}), .in2(DROut), .in3(IROut[7:0]),
        .sel(MuxBSel), .out(MuxBOut)
    );

    Mux4to1_8bit MuxC (
        .in0(ALU_out[7:0]), .in1(ALU_out[15:8]), .in2(ALU_out[23:16]), .in3(ALU_out[31:24]),
        .sel(MuxCSel), .out(MuxCOut)
    );

    Mux2to1_32bit MuxD (
        .in0(RF_outA), .in1({16'b0, ARF_outC}),
        .sel(MuxDSel), .out(MuxDOut)
    );

endmodule


module Mux4to1_32bit (
    input wire [31:0] in0, in1, in2, in3,
    input wire [1:0] sel,
    output wire [31:0] out
    );
    assign out = (sel == 2'b00) ? in0 :
                 (sel == 2'b01) ? in1 :
                 (sel == 2'b10) ? in2 : in3;
endmodule


module Mux2to1_32bit (
    input wire [31:0] in0, in1,
    input wire sel,
    output wire [31:0] out
    );
    assign out = (sel == 1'b0) ? in0 : in1;
endmodule


module Mux4to1_8bit (
    input wire [7:0] in0, in1, in2, in3,
    input wire [1:0] sel,
    output wire [7:0] out
    );
    assign out = (sel == 2'b00) ? in0 :
                 (sel == 2'b01) ? in1 :
                 (sel == 2'b10) ? in2 : in3;
endmodule
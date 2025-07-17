`timescale 1ns / 1ps

module ArithmeticLogicUnitSystem (
    input wire [2:0] RF_OutASel, RF_OutBSel, RF_FunSel,
    input wire [3:0] RF_RegSel, RF_ScrSel,
    input wire [4:0] ALU_FunSel,
    input wire ALU_WF,
    input wire [1:0] ARF_OutCSel, ARF_OutDSel, ARF_FunSel,
    input wire [4:0] ARF_RegSel,
    input wire IR_LH,
    input wire IR_Write, Mem_WR, Mem_CS,
    input wire [1:0] MuxASel, MuxBSel, DR_FunSel,
    input wire MuxCSel, MuxDSel,
    input wire Clock, DR_E
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
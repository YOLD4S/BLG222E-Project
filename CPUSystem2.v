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
    wire [63:0] D;
    6to64Decoder decoder (.in(Opcode), .out(D));

    wire a,b,c,d,e, rrr;
    assign a = Opcode[5];
    assign b = Opcode[4];
    assign c = Opcode[3];
    assign d = Opcode[2];
    assign e = Opcode[1];
    assign rrr = ((a && ~b && c) || (a && b && ~c) || (a && ~b && d));


    assign IR_LH = T[1];
    assign IR_Write = T[0] && T[1];
    assign Opcode = IROut[15:10];

    assign DestReg = rrr ? IROut[9:7] : 3'b000;
    assign SrcReg1 = rrr ? IROut[6:4] : 3'b000;
    assign SrcReg2 = rrr ? IROut[3:1] : 3'b000;
    assign RegSel = IROut[9:8];    //????
    assign Address = IROut[7:0];   //????

    assign Mem_CS = 0;
    assign Mem_WR = ;





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


module 6to64Decoder (
    input wire [5:0] in,
    output wire [63:0] out
    );
    case (in)
        6'h00: out = 64'h1;
        6'h01: out = 64'h2;
        6'h02: out = 64'h4;
        6'h03: out = 64'h8;
        6'h04: out = 64'h10;
        6'h05: out = 64'h20;
        6'h06: out = 64'h40;
        6'h07: out = 64'h80;
        6'h08: out = 64'h100;
        6'h09: out = 64'h200;
        6'h0A: out = 64'h400;
        6'h0B: out = 64'h800;
        6'h0C: out = 64'h1000;
        6'h0D: out = 64'h2000;
        6'h0E: out = 64'h4000;
        6'h0F: out = 64'h8000;
        6'h10: out = 64'h10000;
        6'h11: out = 64'h20000;
        6'h12: out = 64'h40000;
        6'h13: out = 64'h80000;
        6'h14: out = 64'h100000;
        6'h15: out = 64'h200000;
        6'h16: out = 64'h400000;
        6'h17: out = 64'h800000;
        6'h18: out = 64'h1000000;
        6'h19: out = 64'h2000000;
        6'h1A: out = 64'h4000000;
        6'h1B: out = 64'h8000000;
        6'h1C: out = 64'h10000000;
        6'h1D: out = 64'h20000000;
        6'h1E: out = 64'h40000000;
        6'h1F: out = 64'h80000000;
        6'h20: out = 64'h100000000;
        6'h21: out = 64'h200000000;
        6'h22: out = 64'h400000000;
        6'h23: out = 64'h800000000;
    endcase
endmodule
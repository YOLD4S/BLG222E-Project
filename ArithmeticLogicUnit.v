`timescale 1ns / 1ps

module ArithmeticLogicUnit (
    input wire [32:0] A,
    input wire [32:0] B,
    input wire [4:0] FunSel,
    input wire WF,
    input wire Clock,
    output wire [32:0] ALUOut,
    output reg [3:0] FlagsOut
);
    wire Z, C, N, O = FlagsOut;
    wire widthSelect = FunSel[4];
    wire a,b,c,d = FunSel[3:0];

    wire Z_en = WF;
    wire C_en = WF & (~a&b | b&~d | b&c);
    wire N_en = WF & ~(a&b&~c&d);
    wire O_en = WF & (~a&b&~c + ~a&b&~d);

    wire C_in, carrier, C_out;
    wire rightFunSel [3:0] = (FunSel == 5'b11101) ? 4'b1100 : FunSel[3:0];

    halfALU leftALU (
        .C_in(C_in),
        .A(A[15:0]),
        .B(B[15:0]),
        .FunSel(FunSel[3:0]),
        .WF(WF),
        .Clock(Clock),
        .ALUOut(ALUOut[15:0]),
        .FlagsOut(flags_L),
        .C_out(carrier)
    );
    halfALU rightALU (
        .C_in(carrier),
        .A(A[31:16]),
        .B(B[31:16]),
        .FunSel(rightFunSel),
        .WF(WF),
        .Clock(Clock),
        .ALUOut(ALUOut[31:16]),
        .FlagsOut(flags_R),
        .C_out(C_out)
    );
    assign ALUOut[32] = widthSelect ? {leftALU.ALUOut, rightALU.ALUOut} : {{16{rightALU.ALUOut[15]}}, rightALU.ALUOut};

    always @(posedge Clock) begin
        Z <= Z_en & (ALUOut == 0);
        N <= N_en & (widthSelect ? ALUOut[31] : ALUOut[15]);
        C <= C_en & C_out;
        O <= O_en & ((A[31] == B[31]) && (ALUOut[31] != A[31]));
    end

endmodule


module halfALU (
    input wire L_in,
    input wire C_in,
    input wire [15:0] A,
    input wire [15:0] B,
    input wire [3:0] FunSel,  // 5 yerine 4 bitlik cunku MSB islemin 32 ya da 16 bitlik oldugunu ifade eder
    input wire WF,
    input wire Clock,
    output reg [15:0] ALUOut,
    output wire C_out,
    output wire R_out
);

    assign R_out = ALUOut[0];
    always @(posedge Clock) begin
        case (FunSel)
            4'b0000: ALUOut <= A;
            4'b0001: ALUOut <= B;
            4'b0010: ALUOut <= ~A;
            4'b0011: ALUOut <= ~B;
            4'b0100: {C_out, ALUOut} <= A + B + C_in;     //    A + B
            4'b0101: {C_out, ALUOut} <= A + B + C_in; //A + B + C  // ustteki ile ayni duruyor. Gerekli durumda sagdaki yarim ALUnin C_in girisine gerekli inputu Ana ALU icerisinde verecegeim
            4'b0110: {C_out, ALUOut} <= A - B;
            4'b0111: {C_out, ALUOut} <= A & B;
            4'b1000: {C_out, ALUOut} <= A | B;
            4'b1001: {C_out, ALUOut} <= A ^ B;
            4'b1010: {C_out, ALUOut} <= ~(A & B);
            4'b1011: {C_out, ALUOut} <= {A, C_in}; // LSL A
            4'b1100: {ALUOut, R_out} <= {L_in, A}; // LSR A
            4'b1101: ALUOut <= {A[15], A[15:1]}; // ASR A
            4'b1110: {C, ALUOut} <= {A, C}; // CSL A
            4'b1111: {ALUOut, C} <= {C, A}; // CSR A
        endcase
    end 
endmodule
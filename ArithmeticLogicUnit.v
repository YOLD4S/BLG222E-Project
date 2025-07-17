`timescale 1ns / 1ps

module ArithmeticLogicUnit (
    input wire [31:0] A,
    input wire [31:0] B,
    input wire [4:0] FunSel,
    input wire WF,
    input wire Clock,
    output wire [31:0] ALUOut,
    output reg [3:0] FlagsOut // {Z, C, N, O}
);
    wire widthSelect = FunSel[4];
    wire [3:0] f = FunSel[3:0];

    wire Z_en = WF;
    wire C_en = WF & (~f[3]&f[2]&~f[1] | f[2]&~f[0] | f[3]&f[1]&f[0]);
    wire N_en = WF & ~(f[0]&~f[1]&f[2]&f[3]);
    wire O_en = WF & (~f[3]&f[2]&~f[1] | ~f[3]&f[2]&~f[0]); // a'bc' + a'bd'

    wire C;

    assign {C, ALUOut} = (f == 4'b0000) ? {1'b0, A} : 
                 (f == 4'b0001) ? {1'b0, B} :
                 (f == 4'b0010) ? {1'b0, ~A} :
                 (f == 4'b0011) ? {1'b0, ~B} :
                 (f == 4'b0100) ? A + B :
                 (f == 4'b0101) ? A + B + FlagsOut[2] : // A + B + C
                 (f == 4'b0110) ? A - B :
                 (f == 4'b0111) ? {1'b0, A & B} :
                 (f == 4'b1000) ? {1'b0, A | B} :
                 (f == 4'b1001) ? {1'b0, A ^ B} :
                 (f == 4'b1010) ? {1'b0, ~(A & B)} :
                 (f == 4'b1011) ? {A[31:0], 1'b0} : // LSL
                 (f == 4'b1100) ? {A[0], 1'b0, A[31:1]} : // LSR
                 (f == 4'b1101) ? {1'b0, A[31], A[31:1]} : // ASR
                 (f == 4'b1110) ? {A[31:0], FlagsOut[2]} : // CSL
                 {A[0], FlagsOut[2], A[31:1]}; // CSR

    always @(posedge Clock) begin
        if (Z_en)
            FlagsOut[3] <= (ALUOut == 0);
        if (C_en)
            FlagsOut[2] <= widthSelect ? C : (A[26] ^ B[26] ^ ALUOut[26]);
        if (N_en)
            FlagsOut[1] <= ALUOut[31];
        if (O_en)
            FlagsOut[0] <= FunSel[1] ? ((A[31] != B[31]) && (B[31] == ALUOut[31])) : ((A[31] == B[31]) && (ALUOut[31] != A[31]));
    end

endmodule
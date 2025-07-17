// `timescale 1ns / 1ps

// module ArithmeticLogicUnit (
//     input wire [32:0] A,
//     input wire [32:0] B,
//     input wire [4:0] FunSel,
//     input wire WF,
//     input wire Clock,
//     output wire [32:0] ALUOut,
//     output reg [3:0] FlagsOut // Z, C, N, O
// );

//     wire C_in, C_carrier, C_out;
//     wire L_in, LR_carrier, R_out;

//     wire a,b,c,d;
//     assign {a,b,c,d} = FunSel[3:0];
//     wire Z_en = WF;
//     wire C_en = WF & (~a&b | b&~d | b&c);
//     wire N_en = WF & ~(a&b&~c&d);
//     wire O_en = WF & (~a&b&~c + ~a&b&~d);
//     wire widthSelect = FunSel[4];
//     wire [3:0] rightFunSel = (FunSel == 5'b11101) ? 4'b1100 : FunSel[3:0];

//     assign C_in = (~a&d | c&~d) ? FlagsOut[1] : 1'b0;
//     assign L_in = d ? FlagsOut[1] : 1'b0;

//     halfALU leftALU (
//         .C_in(C_carrier),
//         .A(A[31:16]),
//         .B(B[31:16]),
//         .FunSel(FunSel[3:0]),
//         .ALUOut(ALUOut[31:16]),
//         .FlagsOut(flags_L),
//         .C_out(C_out),
//         .L_in(L_in),
//         .R_out(LR_carrier)
//     );
//     halfALU rightALU (
//         .C_in(C_in),
//         .A(A[15:0]),
//         .B(B[15:0]),
//         .FunSel(rightFunSel),
//         .ALUOut(ALUOut[15:00]),
//         .FlagsOut(flags_R),
//         .C_out(C_carrier),
//         .L_in(LR_carrier),
//         .R_out(R_out)
//     );
//     assign ALUOut[32] = widthSelect ? {leftALU.ALUOut, rightALU.ALUOut} : {{16{rightALU.ALUOut[15]}}, rightALU.ALUOut};

//     always @(posedge Clock) begin
//         if (Z_en)
//             FlagsOut[0] <= (ALUOut == 0);
//         else
//             FlagsOut[0] <= FlagsOut[0];
//         if (C_en)
//             FlagsOut[1] <= widthSelect ? C_out : C_carrier;
//         else
//             FlagsOut[1] <= FlagsOut[1];
//         if (N_en)
//             FlagsOut[2] <= ALUOut[31];
//         else
//             FlagsOut[2] <= FlagsOut[2];
//         if (O_en)
//             FlagsOut[3] <= FunSel[1] ? ((A[31] != B[31]) && (B[31] == ALUOut[31])) : ((A[31] == B[31]) && (ALUOut[31] != A[31]));
//         else
//             FlagsOut[3] <= FlagsOut[3];

//     end

// endmodule


// module halfALU (
//     input wire L_in,
//     input wire C_in,
//     input wire [15:0] A,//
//     input wire [15:0] B,//
//     input wire [3:0] FunSel, 
//     input wire Clock,//
//     output wire [15:0] ALUOut,
//     output wire C_out,//
//     output wire R_out,//
//     output wire [3:0] FlagsOut // Z, C, N, O
// );

//     assign R_out = ALUOut[0];
//     assign FlagsOut[0] = (ALUOut == 0); // Z flag
//     assign FlagsOut[1] = C_out; // C flag
//     assign FlagsOut[2] = (ALUOut[15] ==   1); // N flag
//     assign FlagsOut[3] = FunSel[1] ? (A[15] != B[15]) && (ALUOut[15] != A[15]) : ((A[15] == B[15]) && (ALUOut[15] != A[15])); // O flag                                                              


//     assign C_out = (FunSel[3:0] == 4'b0100 || FunSel[3:0] == 4'b0101 || FunSel[3:0] == 4'b0110) ? (A + B + C_in > 16'hFFFF) : 
//     (FunSel[3:0] == 4'b1011 || FunSel[3:0] == 4'b1110) ? ({A, C_in} > 16'hFFFF) : 1'b0; // Carry out condition
//     assign R_out = A[0]; // Right shift out

//     assign ALUOut = (FunSel == 4'b0000) ? A : 
//                  (FunSel == 4'b0001) ? B : 
//                  (FunSel == 4'b0010) ? ~A : 
//                  (FunSel == 4'b0011) ? ~B : 
//                  (FunSel == 4'b0100) ? A + B + C_in : 
//                  (FunSel == 4'b0101) ? A + B + C_in : // A + B + C
//                  (FunSel == 4'b0110) ? A - B : 
//                  (FunSel == 4'b0111) ? A & B : 
//                  (FunSel == 4'b1000) ? A | B : 
//                  (FunSel == 4'b1001) ? A ^ B : 
//                  (FunSel == 4'b1010) ? ~(A & B) : 
//                  (FunSel == 4'b1011) ? {L_in, A[15:1]} : // LSL
//                  (FunSel == 4'b1100) ? {A[15], A[15:1]} : // LSR
//                  (FunSel == 4'b1101) ? {A[15], A[15:1]} : // ASR
//                  (FunSel == 4'b1110) ? {L_in, A[15:1]} : // CSL
//                  {A[15], A[15:1]}; // CSR

// endmodule

`timescale 1ns / 1ps

module ArithmeticLogicUnit (
    input wire [32:0] A,
    input wire [32:0] B,
    input wire [4:0] FunSel,
    input wire WF,
    input wire Clock,
    output wire [32:0] ALUOut,
    output reg [3:0] FlagsOut // Z, C, N, O
);
    assign {Z,C,N,O} = FlagsOut;
    wire a,b,c,d;
    assign {a,b,c,d} = FunSel[3:0];
    wire Z_en = WF;
    wire C_en = WF & (~a&b | b&~d | b&c);
    wire N_en = WF & ~(a&b&~c&d);
    wire O_en = WF & (~a&b&~c + ~a&b&~d);
    wire widthSelect = FunSel[4];

    wire real_A = widthSelect ? A : {{16{A[31]}}, A[15:0]};
    wire real_B = widthSelect ? B : {{16{B[31]}}, B[15:0]};

    assign ALUOut = (FunSel == 4'b0000) ? A : 
                 (FunSel == 4'b0001) ? B :
                 (FunSel == 4'b0010) ? ~A :
                 (FunSel == 4'b0011) ? ~B :
                 (FunSel == 4'b0100) ? A + B :
                 (FunSel == 4'b0101) ? A + B + C : // A + B + C
                 (FunSel == 4'b0110) ? A - B :
                 (FunSel == 4'b0111) ? A & B :
                 (FunSel == 4'b1000) ? A | B :
                 (FunSel == 4'b1001) ? A ^ B :
                 (FunSel == 4'b1010) ? ~(A & B) :
                 (FunSel == 4'b1011) ? {A[31:1], 1'b0} : // LSL
                 (FunSel == 4'b1100) ? {1'b0, A[31:1]} : // LSR
                 (FunSel == 4'b1101) ? {A[31], A[31:1]} : // ASR
                 (FunSel == 4'b1110) ? {C, A[31:1]} : // CSL
                 {A[31], A[31:1]}; // CSR

    always @(posedge Clock) begin
        if (Z_en)
            FlagsOut[3] <= (ALUOut == 3);
        else
            FlagsOut[3] <= FlagsOut[3];
        if (C_en)
            FlagsOut[2] <= widthSelect ? C_out : C_carrier;
        else
            FlagsOut[2] <= FlagsOut[2];
        if (N_en)
            FlagsOut[1] <= ALUOut[31];
        else
            FlagsOut[1] <= FlagsOut[1];
        if (O_en)
            FlagsOut[0] <= FunSel[1] ? ((A[31] != B[31]) && (B[31] == ALUOut[31])) : ((A[31] == B[31]) && (ALUOut[31] != A[31]));
        else
            FlagsOut[3] <= FlagsOut[3];

    end

endmodule
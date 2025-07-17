`timescale 1ns / 1ps


module ArithmeticLogicUnit (
    input wire [31:0] A,
    input wire [31:0] B,
    input wire [4:0] FunSel,   // Function selector
    input wire WF,             // Write flags enable
    input wire Clock,          // Clock input
    output reg [31:0] ALUOut,
    output reg [3:0] FlagsOut  // Z, C, N, O
);

    reg carry;
    reg overflow;
    reg negative;
    reg zero;

    always @(posedge Clock) begin
        carry = 0;
        overflow = 0;
        zero = 0;
        negative = 0;

        case (FunSel)
            // 16-bit operations
            5'b00000: ALUOut = {16'b0, A[15:0]};              // Pass A (lower 16)
            5'b00001: ALUOut = {16'b0, B[15:0]};              // Pass B (lower 16)
            5'b00010: ALUOut = {16'b0, ~A[15:0]};             // NOT A
            5'b00011: ALUOut = {16'b0, ~B[15:0]};             // NOT B
            5'b00100: begin
                ALUOut = {16'b0, A[15:0] + B[15:0]};
                carry = (A[15:0] + B[15:0]) > 16'hFFFF;
                overflow = (A[15] == B[15]) && (ALUOut[15] != A[15]);
            end
            5'b00101: begin
                ALUOut = {16'b0, A[15:0] + B[15:0] + FlagsOut[2]};
                carry = (A[15:0] + B[15:0] + FlagsOut[2]) > 16'hFFFF;
                overflow = (A[15] == B[15]) && (ALUOut[15] != A[15]);
            end
            5'b00110: begin
                ALUOut = {16'b0, A[15:0] - B[15:0]};
                carry = (A[15:0] < B[15:0]);
                overflow = (A[15] != B[15]) && (ALUOut[15] != A[15]);
            end
            5'b00111: ALUOut = {16'b0, A[15:0] & B[15:0]};
            5'b01000: ALUOut = {16'b0, A[15:0] | B[15:0]};
            5'b01001: ALUOut = {16'b0, A[15:0] ^ B[15:0]};
            5'b01010: ALUOut = {16'b0, ~(A[15:0] & B[15:0])};
            5'b01011: begin ALUOut = {16'b0, A[14:0], 1'b0}; carry = A[15]; end // LSL
            5'b01100: begin ALUOut = {16'b0, 1'b0, A[15:1]}; carry = A[0]; end  // LSR
            5'b01101: ALUOut = {16'b0, A[15], A[15:1]};                         // ASR
            5'b01110: begin ALUOut = {16'b0, A[14:0], FlagsOut[2]}; carry = A[15]; end // CSL
            5'b01111: begin ALUOut = {16'b0, FlagsOut[2], A[15:1]}; carry = A[0]; end  // CSR

            // 32-bit operations
            5'b10000: ALUOut = A;
            5'b10001: ALUOut = B;
            5'b10010: ALUOut = ~A;
            5'b10011: ALUOut = ~B;
            5'b10100: begin
                ALUOut = A + B;
                carry = (A + B) > 32'hFFFFFFFF;
                overflow = (A[31] == B[31]) && (ALUOut[31] != A[31]);
            end
            5'b10101: begin
                ALUOut = A + B + FlagsOut[2];
                carry = (A + B + FlagsOut[2]) > 32'hFFFFFFFF;
                overflow = (A[31] == B[31]) && (ALUOut[31] != A[31]);
            end
            5'b10110: begin
                ALUOut = A - B;
                carry = (A < B);
                overflow = (A[31] != B[31]) && (ALUOut[31] != A[31]);
            end
            5'b10111: ALUOut = A & B;
            5'b11000: ALUOut = A | B;
            5'b11001: ALUOut = A ^ B;
            5'b11010: ALUOut = ~(A & B);
            5'b11011: begin ALUOut = A << 1; carry = A[31]; end // LSL
            5'b11100: begin ALUOut = A >> 1; carry = A[0]; end  // LSR
            5'b11101: ALUOut = {A[31], A[31:1]};                // ASR
            5'b11110: begin ALUOut = {A[30:0], FlagsOut[2]}; carry = A[31]; end // CSL
            5'b11111: begin ALUOut = {FlagsOut[2], A[31:1]}; carry = A[0]; end  // CSR

            default: ALUOut = 32'b0;
        endcase

        zero = (ALUOut == 0);
        negative = ALUOut[31];

        if (WF) begin
            FlagsOut[3] <= zero;
            FlagsOut[2] <= carry;
            FlagsOut[1] <= negative;
            FlagsOut[0] <= overflow;
        end
    end
endmodule















// module ArithmeticLogicUnit (
//     input wire [31:0] A,
//     input wire [31:0] B,
//     input wire [4:0] FunSel,
//     input wire WF,
//     input wire Clock,
//     output wire [31:0] ALUOut,
//     output reg [3:0] FlagsOut // {Z, C, N, O}
// );
//     wire widthSelect = FunSel[4];
//     wire [3:0] f = FunSel[3:0];

//     wire Z_en = WF;
//     wire C_en = WF & (~f[0]&f[1] | f[1]&~f[3] | f[1]&f[2]);
//     wire N_en = WF & ~(f[0]&f[1]&~f[2]&f[3]);
//     wire O_en = WF & (~f[0]&f[1]&~f[2] + ~f[0]&f[1]&~f[3]);
    
//     wire C_out;

//     // wire [31:0] real_A = widthSelect ? A : {{16{A[15]}}, A[15:0]};
//     // wire [31:0] real_B = widthSelect ? B : {{16{B[15]}}, B[15:0]};

//     assign {C_out, ALUOut} = (FunSel == 4'b0000) ? {1'b0, A} : 
//                  (FunSel == 4'b0001) ? {1'b0, B} :
//                  (FunSel == 4'b0010) ? {1'b0, ~A} :
//                  (FunSel == 4'b0011) ? {1'b0, ~B} :
//                  (FunSel == 4'b0100) ? A + B :
//                  (FunSel == 4'b0101) ? A + B + FlagsOut[2] : // A + B + C
//                  (FunSel == 4'b0110) ? A - B :
//                  (FunSel == 4'b0111) ? {1'b0, A & B} :
//                  (FunSel == 4'b1000) ? {1'b0, A | B} :
//                  (FunSel == 4'b1001) ? {1'b0, A ^ B} :
//                  (FunSel == 4'b1010) ? {1'b0, ~(A & B)} :
//                  (FunSel == 4'b1011) ? {A[31:0], 1'b0} : // LSL
//                  (FunSel == 4'b1100) ? {A[0], 1'b0, A[31:1]} : // LSR
//                  (FunSel == 4'b1101) ? {1'b0, A[31], A[31:1]} : // ASR
//                  (FunSel == 4'b1110) ? {A[31:0], FlagsOut[2]} : // CSL
//                  {A[0], FlagsOut[2], A[31:1]}; // CSR

//     always @(posedge Clock) begin
//         if (Z_en)
//             FlagsOut[3] <= (ALUOut == 0);
//         if (C_en)
//             FlagsOut[2] <= widthSelect ? C_out : (A[26] ^ B[26] ^ ALUOut[26]);
//         if (N_en)
//             FlagsOut[1] <= ALUOut[31];
//         if (O_en)
//             FlagsOut[0] <= FunSel[1] ? ((A[31] != B[31]) && (B[31] == ALUOut[31])) : ((A[31] == B[31]) && (ALUOut[31] != A[31]));
//     end

// endmodule










// `timescale 1ns / 1ps

// module ArithmeticLogicUnit (
//     input wire [31:0] A,
//     input wire [31:0] B,
//     input wire [4:0] FunSel,
//     input wire WF,
//     input wire Clock,
//     output wire [31:0] ALUOut,
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
//     assign ALUOut[31] = widthSelect ? {leftALU.ALUOut, rightALU.ALUOut} : {{16{rightALU.ALUOut[15]}}, rightALU.ALUOut};

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
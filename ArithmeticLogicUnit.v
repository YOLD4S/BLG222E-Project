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

    wire C_in, C_carrier, C_out;
    wire L_in, LR_carrier, R_out;

    wire a,b,c,d;
    assign {a,b,c,d} = FunSel[3:0];
    wire Z_en = WF;
    wire C_en = WF & (~a&b | b&~d | b&c);
    wire N_en = WF & ~(a&b&~c&d);
    wire O_en = WF & (~a&b&~c + ~a&b&~d);
    wire widthSelect = FunSel[4];
    wire [3:0] rightFunSel = (FunSel == 5'b11101) ? 4'b1100 : FunSel[3:0];

    assign C_in = (~a&d | c&~d) ? FlagsOut[1] : 1'b0;
    assign L_in = d ? FlagsOut[1] : 1'b0;

    halfALU leftALU (
        .C_in(C_carrier),
        .A(A[15:0]),
        .B(B[15:0]),
        .FunSel(FunSel[3:0]),
        .Clock(Clock),
        .ALUOut(ALUOut[15:0]),
        .FlagsOut(flags_L),
        .C_out(C_out),
        .L_in(L_in),
        .R_out(LR_carrier)
    );
    halfALU rightALU (
        .C_in(C_in),
        .A(A[31:16]),
        .B(B[31:16]),
        .FunSel(rightFunSel),
        .Clock(Clock),
        .ALUOut(ALUOut[31:16]),
        .FlagsOut(flags_R),
        .C_out(C_carrier),
        .L_in(LR_carrier),
        .R_out(R_out)
    );
    assign ALUOut[32] = widthSelect ? {leftALU.ALUOut, rightALU.ALUOut} : {{16{rightALU.ALUOut[15]}}, rightALU.ALUOut};

    always @(posedge Clock) begin
        FlagsOut[0] <= Z_en & (ALUOut == 0);
        FlagsOut[2] <= N_en & (widthSelect ? ALUOut[31] : ALUOut[15]);
        FlagsOut[1] <= C_en & C_out;
        FlagsOut[3] <= O_en & ((A[31] == B[31]) && (ALUOut[31] != A[31]));
    end

endmodule


module halfALU (
    input wire L_in,
    input wire C_in,
    input wire [15:0] A,//
    input wire [15:0] B,//
    input wire [3:0] FunSel, 
    input wire Clock,//
    output reg [15:0] ALUOut,
    output wire C_out,//
    output wire R_out,//
    output wire [3:0] FlagsOut // Z, C, N, O
);

    assign R_out = ALUOut[0];
    assign FlagsOut[0] = (ALUOut == 0); // Z flag
    assign FlagsOut[1] = C_out; // C flag
    assign FlagsOut[2] = (ALUOut[15] ==   1); // N flag
    assign FlagsOut[3] = FunSel[1] ? (A[15] != B[15]) && (ALUOut[15] != A[15]) : ((A[15] == B[15]) && (ALUOut[15] != A[15])); // O flag                                                              


    assign C_out = (FunSel[3:0] == 4'b0100 || FunSel[3:0] == 4'b0101 || FunSel[3:0] == 4'b0110) ? (A + B + C_in > 16'hFFFF) : 
    (FunSel[3:0] == 4'b1011 || FunSel[3:0] == 4'b1110) ? ({A, C_in} > 16'hFFFF) : 1'b0; // Carry out condition
    assign R_out = A[0]; // Right shift out

    always @(posedge Clock) begin
        case (FunSel)
            4'b0000: ALUOut <= A;
            4'b0001: ALUOut <= B;
            4'b0010: ALUOut <= ~A;
            4'b0011: ALUOut <= ~B;
            4'b0100: ALUOut <= A + B + C_in;     //    A + B
            4'b0101: ALUOut <= A + B + C_in; //A + B + C  // ustteki ile ayni duruyor. Gerekli durumda sagdaki yarim ALUnin C_in girisine gerekli inputu Ana ALU icerisinde verecegeim
            4'b0110: ALUOut <= A - B;
            4'b0111: ALUOut <= A & B;
            4'b1000: ALUOut <= A | B;
            4'b1001: ALUOut <= A ^ B;
            4'b1010: ALUOut <= ~(A & B);
            4'b1011: ALUOut <= {A, C_in}; // LSL A
            4'b1100: ALUOut <= {L_in, A[15:1]}; // LSR A
            4'b1101: ALUOut <= {A[15], A[15:1]}; // ASR A        ///.     Needs to be solved
            4'b1110: ALUOut <= {A, C_in}; // CSL A
            4'b1111: ALUOut <= {L_in, A[15:1]}; // CSR A.      // Combinatorial logic for L_in should be handled outside this module
        endcase
    end
endmodule


// L_in logic ekle (ana modulde)
// ASR duzelt (Ana modulde sag yarima LSR sinyali gondereerek olabilir)
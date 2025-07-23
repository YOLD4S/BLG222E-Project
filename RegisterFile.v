`timescale 1ns / 1ps

module RegisterFile(
    input   wire        Clock     ,
    input   wire[31:0]  I  ,
    input   wire[2:0]   OutASel ,
    input   wire[2:0]   OutBSel ,
    input   wire[2:0]   FunSel  ,
    input   wire[3:0]   RegSel  ,
    input   wire[3:0]   ScrSel  ,
    
    output  wire[31:0]  OutA     ,
    output  wire[31:0]  OutB
);
    wire ER1, ER2, ER3, ER4, ES1, ES2, ES3, ES4;

    wire[31:0] Q_R1;
    wire[31:0] Q_R2;
    wire[31:0] Q_R3;
    wire[31:0] Q_R4;
    wire[31:0] Q_S1;
    wire[31:0] Q_S2;
    wire[31:0] Q_S3;
    wire[31:0] Q_S4;

    assign {ER1, ER2, ER3, ER4} = RegSel;
    assign {ES1, ES2, ES3, ES4} = ScrSel;
    
    Register32bit R1(.I(I), .E(ER1), .FunSel(FunSel), .Clock(Clock), .Q(Q_R1));
    Register32bit R2(.I(I), .E(ER2), .FunSel(FunSel), .Clock(Clock), .Q(Q_R2));
    Register32bit R3(.I(I), .E(ER3), .FunSel(FunSel), .Clock(Clock), .Q(Q_R3));
    Register32bit R4(.I(I), .E(ER4), .FunSel(FunSel), .Clock(Clock), .Q(Q_R4));

    Register32bit S1(.I(I), .E(ES1), .FunSel(FunSel), .Clock(Clock), .Q(Q_S1));
    Register32bit S2(.I(I), .E(ES2), .FunSel(FunSel), .Clock(Clock), .Q(Q_S2));
    Register32bit S3(.I(I), .E(ES3), .FunSel(FunSel), .Clock(Clock), .Q(Q_S3));
    Register32bit S4(.I(I), .E(ES4), .FunSel(FunSel), .Clock(Clock), .Q(Q_S4));
    
    // Output selection for OutA and OutB
    assign OutA = (OutASel == 3'b000) ? Q_R1 :
                  (OutASel == 3'b001) ? Q_R2 :
                  (OutASel == 3'b010) ? Q_R3 :
                  (OutASel == 3'b011) ? Q_R4 :
                  (OutASel == 3'b100) ? Q_S1 :
                  (OutASel == 3'b101) ? Q_S2 :
                  (OutASel == 3'b110) ? Q_S3 : Q_S4;
    assign OutB = (OutBSel == 3'b000) ? Q_R1 :
                  (OutBSel == 3'b001) ? Q_R2 :
                  (OutBSel == 3'b010) ? Q_R3 :
                  (OutBSel == 3'b011) ? Q_R4 :
                  (OutBSel == 3'b100) ? Q_S1 :
                  (OutBSel == 3'b101) ? Q_S2 :
                  (OutBSel == 3'b110) ? Q_S3 : Q_S4;

endmodule
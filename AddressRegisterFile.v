`timescale 1ns / 1ps

module AddressRegisterFile (
    input wire Clock,
    input wire [31:0] I, // Bu kisim odev dosyasinda ve simulasyonda 32 bit olarak tanimlanmis. Mantiken 16 olmasi lazim ama
    input wire [2:0] RegSel,
    input wire [1:0] FunSel,
    input wire [1:0] OutCSel,
    input wire [1:0] OutDSel,
    output wire [15:0] OutC,
    output wire [15:0] OutD
);
  wire PC_E, AR_E, SP_E;
  wire [15:0] Q_PC, Q_AR, Q_SP;

  assign PC_E = RegSel[2];
  assign SP_E = RegSel[1];
  assign AR_E = RegSel[0];

  Register16bit PC (
      .I(I[15:0]),
      .E(PC_E),
      .FunSel(FunSel),
      .Clock(Clock),
      .Q(Q_PC)
  );
  Register16bit AR (
      .I(I[15:0]),
      .E(AR_E),
      .FunSel(FunSel),
      .Clock(Clock),
      .Q(Q_AR)
  );
  Register16bit SP (
      .I(I[15:0]),
      .E(SP_E),
      .FunSel(FunSel),
      .Clock(Clock),
      .Q(Q_SP)
  );

  assign OutC = (OutCSel == 2'b00) ? Q_PC : (OutCSel == 2'b01) ? Q_SP : Q_AR;

  assign OutD = (OutDSel == 2'b00) ? Q_PC : (OutDSel == 2'b01) ? Q_SP : Q_AR;

endmodule

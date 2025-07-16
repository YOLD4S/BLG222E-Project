module AddressRegisterFile(
    input wire Clock,
    input wire [15:0] I, // Bu kisim odev dosyasinda ve simulasyonda 32 bit olarak tanimlanmis
    input wire [2:0] RegSel,
    input wire [1:0] FunSel,
    input wire [1:0] OutCSel,
    input wire [1:0] OutDSel,
    output wire [15:0] OutC,
    output wire [15:0] OutD
);
    wire PC_E, AR_E, SP_E;
    wire [15:0] Q_PC, Q_AR, Q_SP;

    Register16bit PC(.I(I), .E(PC_E), .FunSel(FunSel), .Clock(Clock), .Q(Q_PC));
    Register16bit AR(.I(I), .E(AR_E), .FunSel(FunSel), .Clock(Clock), .Q(Q_AR));
    Register16bit SP(.I(I), .E(SP_E), .FunSel(FunSel), .Clock(Clock), .Q(Q_SP));

    assign PC_E = RegSel[0];
    assign AR_E = RegSel[1];
    assign SP_E = RegSel[2];

    assign OutC = (OutCSel == 2'b00) ? PC.Q :
                  (OutCSel == 2'b01) ? SP.Q : AR.Q;
                  
    assign OutD = (OutDSel == 2'b00) ? PC.Q :
                  (OutDSel == 2'b01) ? SP.Q : AR.Q;

    // always @(posedge Clock) begin
    //     if (we) begin
    //         reg_file[addr] <= data_in;
    //     end
    //     OutC <= reg_file[OutCSel];
    //     OutD <= reg_file[OutDSel];
    // end
endmodule
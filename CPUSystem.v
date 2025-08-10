`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Abdullah Akcan
// Project Name: BLG222E Project 2 Simulation
//////////////////////////////////////////////////////////////////////////////////

module r_selector (
    input [1:0] in,
    output reg [3:0] out
    );

    always @(*) begin
        case (in)
            2'b00: out = 4'b1000;
            2'b01: out = 4'b0100;
            2'b10: out = 4'b0010;
            2'b11: out = 4'b0001;
            default: out = 4'b0000; 
        endcase
    end
endmodule

module r_rf_selector (
    input [2:0] in,
    output reg [3:0] rf_out,
    output reg [2:0] arf_out
    );
    
    always @(*) begin
        case (in)
            3'b000: arf_out = 3'b100;   // PC
            3'b001: arf_out = 3'b010;   // SP
            3'b010: arf_out = 3'b001;   // AR
            3'b011: arf_out = 3'b001;   // AR
            3'b100: rf_out = 4'b1000;   // R1
            3'b101: rf_out = 4'b0100;   // R2
            3'b110: rf_out = 4'b0010;   // R3
            3'b111: rf_out = 4'b0001;   // R4
            default: begin
                rf_out = 4'b0000;
                arf_out = 3'b000;
            end
        endcase
    end
endmodule

module CPUSystem (
    input Clock,
    input Reset,
    output reg [11:0] T
    );
    
    reg [2:0] RF_OutASel, RF_OutBSel, RF_FunSel, ARF_RegSel;
    reg [3:0] RF_RegSel, RF_ScrSel;
    reg [4:0] ALU_FunSel;
    reg [1:0] ARF_OutCSel, ARF_OutDSel, ARF_FunSel;
    reg ALU_WF, IR_LH, IR_Write, Mem_WR, Mem_CS;         
    reg [1:0] MuxASel, MuxBSel, MuxCSel;
    reg MuxDSel;
    reg [1:0] DR_FunSel;
    reg DR_E; 
    wire Z, C, N, O;

    wire [5:0] Opcode;
    wire [1:0] RegSel;
    wire [7:0] Address;
    wire [2:0] DestReg, SrcReg1, SrcReg2;

    reg T_Reset;

    ArithmeticLogicUnitSystem ALUSys(
        .RF_OutASel(RF_OutASel),   .RF_OutBSel(RF_OutBSel), 
        .RF_FunSel(RF_FunSel),     .RF_RegSel(RF_RegSel),
        .RF_ScrSel(RF_ScrSel),     .ALU_FunSel(ALU_FunSel),
        .ALU_WF(ALU_WF),           .ARF_OutCSel(ARF_OutCSel), 
        .ARF_OutDSel(ARF_OutDSel), .ARF_FunSel(ARF_FunSel),
        .ARF_RegSel(ARF_RegSel),   .IR_LH(IR_LH),
        .IR_Write(IR_Write),       .Mem_WR(Mem_WR),
        .Mem_CS(Mem_CS),           .MuxASel(MuxASel),
        .MuxBSel(MuxBSel),         .MuxCSel(MuxCSel),
        .Clock(Clock),         .DR_FunSel(DR_FunSel),
        .DR_E(DR_E),               .MuxDSel(MuxDSel) 
    ); 

    assign {Z,C,N,O} = ALUSys.ALU.FlagsOut;
    
    assign Opcode = ALUSys.IR.IROut[15:10];
    assign RegSel = ALUSys.IR.IROut[9:8];
    assign Address = ALUSys.IR.IROut[7:0];
    assign DestReg = ALUSys.IR.IROut[9:7];
    assign SrcReg1 = ALUSys.IR.IROut[6:4];
    assign SrcReg2 = ALUSys.IR.IROut[3:1];

    wire [3:0] r_sel;
    wire [3:0] rf_dest;
    wire [3:0] rf_src1;
    wire [3:0] rf_src2;
    wire [2:0] arf_dest;
    wire [2:0] arf_src1;
    wire [2:0] arf_src2;

    r_selector rsel (.in(RegSel), .out(r_sel));
    r_rf_selector destsel (.in(DestReg), .rf_out(rf_dest), .arf_out(arf_dest));
    r_rf_selector src1sel (.in(SrcReg1), .rf_out(rf_src1), .arf_out(arf_src1));
    r_rf_selector src2sel (.in(SrcReg2), .rf_out(rf_src2), .arf_out(arf_src2));

    task ClearRegisters;
        begin
            ALUSys.RF.R1.Q = 32'h0;
            ALUSys.RF.R2.Q = 32'h0;
            ALUSys.RF.R3.Q = 32'h0;
            ALUSys.RF.R4.Q = 32'h0;
            ALUSys.RF.S1.Q = 32'h0;
            ALUSys.RF.S2.Q = 32'h0;
            ALUSys.RF.S3.Q = 32'h0;
            ALUSys.RF.S4.Q = 32'h0;
            ALUSys.ARF.PC.Q = 16'h0;
            ALUSys.ARF.AR.Q = 16'h0;
            ALUSys.ARF.SP.Q = 16'h00FF;
            ALUSys.IR.IROut = 16'h0;
            ALUSys.DR.DROut = 32'h0;
            ALUSys.ALU.FlagsOut = 4'b0000;
        end
    endtask

    task DisableAll;
        begin
            RF_RegSel = 4'b0000;
            RF_ScrSel = 4'b0000;
            ARF_RegSel = 3'b000;
            IR_Write = 0;
            ALU_WF = 0;
            Mem_CS = 1;
            Mem_WR = 0;
            DR_E = 0;
        end
    endtask

    task ResetT;
        begin
            T_Reset = 1;
        end
    endtask

    always @(posedge Clock or negedge Reset) begin
        if (!Reset || T_Reset) begin
            T <= 12'b000000000001; // Reset T to 1
            T_Reset <= 0;
        end
        else // shift < T one bit to the left
            T <= {T[10:0], T[11]};
    end

    always @(*) begin 
        if (!Reset) begin
            DisableAll();
            ClearRegisters();
        end
        DisableAll();   // to prevent any wrong operation

        if (T[0]) begin // IR[7:0] <- M[PC]
            ARF_OutDSel = 2'b00; // PC
            Mem_CS = 1'b0;       // Enable memory
            Mem_WR = 1'b0;       // Memory read 
            IR_Write = 1'b1;     // Enable writing 
            IR_LH = 1'b0;        // Load LSB
            // PC+1
            ARF_RegSel = 3'b100; // PC
            ARF_FunSel = 2'b01;  // Increment
        end

        if (T[1]) begin // IR[15:8] <- M[PC]
            ARF_OutDSel = 2'b00; 
            Mem_CS = 1'b0;       
            Mem_WR = 1'b0;       
            IR_Write = 1'b1;     
            IR_LH = 1'b1;        
            // PC <= PC+1 
            ARF_RegSel = 3'b100; // Enable PC
            ARF_FunSel = 2'b01;  // Increment
        end

        else begin
            case (Opcode)
                6'b000000: begin // BRA   PC <- VALUE
                    if (T[2]) begin
                        MuxBSel = 2'b11;     // IROut[7:0]
                        ARF_RegSel = 3'b100; // PC
                        ARF_FunSel = 2'b10;  // Load
                        ResetT();
                    end
                end

                6'b000001: begin // BNE    IF Z=0 THEN PC <- VALUE
                    if (T[2]) begin
                        if (Z == 0) begin 
                            MuxBSel = 2'b11;     // IROut[7:0]
                            ARF_RegSel = 3'b100; // PC
                            ARF_FunSel = 2'b10;  // Load
                        end
                        ResetT();
                    end     
                end

                6'b000010: begin // BEQ   IF Z=1 THEN PC <- VALUE
                    if (T[2]) begin
                        if (Z == 1) begin
                            MuxBSel = 2'b11;     // IROut[7:0]
                            ARF_RegSel = 3'b100; // PC
                            ARF_FunSel = 2'b10;  // Load
                        end
                        ResetT();
                    end
                end

                6'b000011: begin // POPL    SP ← SP + 1, Rx ← M[SP] (16-bit)
    if (T[2]) begin
        ARF_RegSel = 3'b010;    // SP
        ARF_FunSel = 2'b01;     // increment
    end
    else if (T[3]) begin  // DR[15:8] <- M[SP] (MSB first) | SP <- SP + 1
        ARF_RegSel = 3'b010;    // SP
        ARF_FunSel = 2'b01;     // increment
        ARF_OutDSel = 2'b01;    // SP
        Mem_CS = 0; 
        Mem_WR = 0;
        DR_E = 1;
        DR_FunSel = 2'b10;      // left shift and load (MSB goes to upper bits)
    end
    else if (T[4]) begin  // DR[7:0] <- M[SP] (LSB second)
        ARF_OutDSel = 2'b01;    // SP
        Mem_CS = 0; 
        Mem_WR = 0;
        DR_E = 1;
        DR_FunSel = 2'b01;      // load (LSB goes to lower bits)
    end
    else if (T[5]) begin  // Rx <- DR
        MuxASel = 2'b10;   // DROut
        RF_RegSel = r_sel; 
        RF_FunSel = 3'b101;
        ResetT();
    end   
end

                6'b000100: begin // PSHL    M[SP] ← Rx, SP ← SP – 1 (16-bit)
    if (T[2]) begin
        RF_OutASel = {1'b0, RegSel};    // Rx
        MuxDSel = 0;                    // OutA
        ALU_FunSel = 5'b00000;          // A
        ALU_WF = 1;
        MuxCSel = 2'b01;                // MSB first (big-endian)

        ARF_OutDSel = 2'b01;            // SP
        Mem_CS = 0;
        Mem_WR = 1;

        ARF_RegSel = 3'b010;    
        ARF_FunSel = 2'b00; // decrement SP
    end    
    if (T[3]) begin
        RF_OutASel = {1'b0, RegSel};    // Rx
        MuxDSel = 0;                    // OutA
        ALU_FunSel = 5'b00000;          // A
        ALU_WF = 1;
        MuxCSel = 2'b00;                // LSB second (big-endian)

        ARF_OutDSel = 2'b01;            // SP
        Mem_CS = 0;
        Mem_WR = 1;

        ARF_RegSel = 3'b010;    
        ARF_FunSel = 2'b00; // decrement SP

        ResetT();
    end      
end

                6'b000101: begin // POPH    SP ← SP + 1, Rx ← M[SP] (32-bit)
    if (T[2]) begin
        ARF_RegSel = 3'b010;    // SP
        ARF_FunSel = 2'b01;     // increment
    end
    else if (T[3]) begin  // DR[31:24] <- M[SP] (MSB first) | SP <- SP + 1
        ARF_RegSel = 3'b010;    // SP
        ARF_FunSel = 2'b01;     // increment
        ARF_OutDSel = 2'b01;    // SP
        Mem_CS = 0; 
        Mem_WR = 0;
        DR_E = 1;
        DR_FunSel = 2'b10;      // left shift and load
    end
    else if (T[4]) begin  // DR[23:16] <- M[SP] | SP <- SP + 1
        ARF_RegSel = 3'b010;    // SP
        ARF_FunSel = 2'b01;     // increment
        ARF_OutDSel = 2'b01;    // SP
        Mem_CS = 0; 
        Mem_WR = 0;
        DR_E = 1;
        DR_FunSel = 2'b10;      // left shift and load
    end
    else if (T[5]) begin  // DR[15:8] <- M[SP] | SP <- SP + 1
        ARF_RegSel = 3'b010;    // SP
        ARF_FunSel = 2'b01;     // increment
        ARF_OutDSel = 2'b01;    // SP
        Mem_CS = 0; 
        Mem_WR = 0;
        DR_E = 1;
        DR_FunSel = 2'b10;      // left shift and load
    end
    else if (T[6]) begin  // DR[7:0] <- M[SP] (LSB last)
        ARF_OutDSel = 2'b01;    // SP
        Mem_CS = 0; 
        Mem_WR = 0;
        DR_E = 1;
        DR_FunSel = 2'b01;      // load (no shift for last byte)
    end
    else if (T[7]) begin  // Rx <- DR
        MuxASel = 2'b10;        // DROut
        RF_RegSel = r_sel; 
        RF_FunSel = 3'b010;     // load
        ResetT();
    end     
end

                6'b000110: begin // PSHH    M[SP] ← Rx, SP ← SP – 1 (32-bit)
    if (T[2]) begin
        RF_OutASel = {1'b0, RegSel};    // Rx
        MuxDSel = 0;                    // OutA
        ALU_FunSel = 5'b10000;          // A
        ALU_WF = 1;
        MuxCSel = 2'b11;                // MSB first (bits 31:24)

        ARF_OutDSel = 2'b01;            // SP
        Mem_CS = 0;
        Mem_WR = 1;

        ARF_RegSel = 3'b010;    
        ARF_FunSel = 2'b00; // decrement SP
    end    
    if (T[3]) begin
        RF_OutASel = {1'b0, RegSel};    // Rx
        MuxDSel = 0;                    // OutA
        ALU_FunSel = 5'b10000;          // A
        ALU_WF = 1;
        MuxCSel = 2'b10;                // Second byte (bits 23:16)

        ARF_OutDSel = 2'b01;            // SP
        Mem_CS = 0;
        Mem_WR = 1;

        ARF_RegSel = 3'b010;    
        ARF_FunSel = 2'b00; // decrement SP
    end
    if (T[4]) begin
        RF_OutASel = {1'b0, RegSel};    // Rx
        MuxDSel = 0;                    // OutA
        ALU_FunSel = 5'b10000;          // A
        ALU_WF = 1;
        MuxCSel = 2'b01;                // Third byte (bits 15:8)

        ARF_OutDSel = 2'b01;            // SP
        Mem_CS = 0;
        Mem_WR = 1;

        ARF_RegSel = 3'b010;    
        ARF_FunSel = 2'b00; // decrement SP
    end
    if (T[5]) begin
        RF_OutASel = {1'b0, RegSel};    // Rx
        MuxDSel = 0;                    // OutA
        ALU_FunSel = 5'b10000;          // A
        ALU_WF = 1;
        MuxCSel = 2'b00;                // LSB last (bits 7:0)

        ARF_OutDSel = 2'b01;            // SP
        Mem_CS = 0;
        Mem_WR = 1;

        ARF_RegSel = 3'b010;    
        ARF_FunSel = 2'b00; // decrement SP

        ResetT();
    end
end

                6'b000111: begin // CALL    M[SP] <- PC, SP <- SP – 1, PC <- VALUE (16 bit)
    if (T[2]) begin
        ARF_OutCSel = 2'b00;    // PC
        MuxDSel = 1;            // OutC
        ALU_FunSel = 5'b00000;  // A
        ALU_WF = 1;
        MuxCSel = 2'b01;        // MSB first (big-endian)

        ARF_OutDSel = 2'b01;    // SP
        Mem_CS = 0;
        Mem_WR = 1;

        ARF_RegSel = 3'b010;    
        ARF_FunSel = 2'b00; // decrement SP
    end    
    if (T[3]) begin
        ARF_OutCSel = 2'b00;    // PC
        MuxDSel = 1;            // OutC
        ALU_FunSel = 5'b00000;  // A
        ALU_WF = 1;
        MuxCSel = 2'b00;        // LSB second (big-endian)

        ARF_OutDSel = 2'b01;    // SP
        Mem_CS = 0;
        Mem_WR = 1;

        ARF_RegSel = 3'b010;    
        ARF_FunSel = 2'b00; // decrement SP
    end 
    if (T[4]) begin
        MuxBSel = 2'b11; 
        ARF_RegSel = 3'b100;    // PC
        ARF_FunSel = 2'b10;     // load
        ResetT();
    end 
end


                6'b001000: begin // RET     SP <- SP + 1, PC <- M[SP] (16 bit)
    if (T[2]) begin
        ARF_RegSel = 3'b010;  // SP
        ARF_FunSel = 2'b01;   // increment
    end
    else if (T[3]) begin  // DR[15:8] <- M[SP] (MSB first) | SP <- SP + 1
        ARF_RegSel = 3'b010;  // SP
        ARF_FunSel = 2'b01;   // increment
        ARF_OutDSel = 2'b01;   // SP
        Mem_CS = 0; 
        Mem_WR = 0;
        DR_E = 1;
        DR_FunSel = 2'b10;  // left shift and load
    end
    else if (T[4]) begin  // DR[7:0] <- M[SP] (LSB second)
        ARF_OutDSel = 2'b01;   // SP
        Mem_CS = 0; 
        Mem_WR = 0;
        DR_E = 1;
        DR_FunSel = 2'b01;  // load
    end
    else if (T[5]) begin  // PC <- DR
        MuxBSel = 2'b10;        // DROut
        ARF_RegSel = 3'b100;    // PC
        ARF_FunSel = 2'b10;     // load
        ResetT();
    end
end

                6'b001001: begin // INC     // DSTREG <- SREG1 + 1
                    if (T[2]) begin
                        if (SrcReg1[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end
                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b010;     // load
                    end
                    else if (T[3]) begin
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b001;     // increment
                    end

                    else if (T[4]) begin    
                        RF_OutASel = 3'b100;    // S1
                        MuxDSel = 0;
                        ALU_WF = 1;
                        if (SrcReg1[2])
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        else 
                            ALU_FunSel = 5'b00000;  // A (16-bit)

                        if (DestReg[2]) begin
                            MuxASel = 2'b00;   // ALUOut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b00;  // ALUOut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                end
                6'b001010: begin // DEC   DSTREG <- SREG1 – 1
                    if (T[2]) begin
                        if (SrcReg1[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b010;     // load
                    end
                    else if (T[3]) begin
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b000;     // decrement
                    end

                    else if (T[4]) begin
                        RF_OutASel = 3'b100;    // S1
                        MuxDSel = 0;
                        ALU_WF = 1;
                        if (SrcReg1[2])
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        else 
                            ALU_FunSel = 5'b00000;  // A (16-bit)

                        if (DestReg[2]) begin
                            MuxASel = 2'b00;   // ALUOut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b00;  // ALUOut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                end
                6'b001011: begin // LSL   DSTREG <- LSL SREG1
                    if (T[2]) begin
                        if (SrcReg1[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b11011;  // LSL A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b01011;  // LSL A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[3]) begin
                        RF_OutASel = 3'b100;    // S1
                        MuxDSel = 0;
                        // ALU_WF = 1; // burada da açılmalı mı acaba?
                        if (SrcReg1[2])
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        else 
                            ALU_FunSel = 5'b00000;  // A (16-bit)

                        if (DestReg[2]) begin
                            MuxASel = 2'b00;   // ALUOut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b00;  // ALUOut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                end
                6'b001100: begin // LSR   DSTREG <- LSR SREG1
                    if (T[2]) begin
                        if (SrcReg1[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b11100;  // LSR A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b01100;  // LSR A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[3]) begin
                        RF_OutASel = 3'b100;    // S1
                        MuxDSel = 0;
                        // ALU_WF = 1; // burada da açılmalı mı acaba?
                        if (SrcReg1[2])
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        else 
                            ALU_FunSel = 5'b00000;  // A (16-bit)

                        if (DestReg[2]) begin
                            MuxASel = 2'b00;   // ALUOut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b00;  // ALUOut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                end
                6'b001101: begin // ASR   DSTREG <- ASR SREG1
                    if (T[2]) begin
                        if (SrcReg1[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b11101;  // ASR A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b01101;  // ASR A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[3]) begin
                        RF_OutASel = 3'b100;    // S1
                        MuxDSel = 0;
                        // ALU_WF = 1; // burada da açılmalı mı acaba?
                        if (SrcReg1[2])
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        else 
                            ALU_FunSel = 5'b00000;  // A (16-bit)

                        if (DestReg[2]) begin
                            MuxASel = 2'b00;   // ALUOut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b00;  // ALUOut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                end
                6'b001110: begin // CSL   DSTREG <- CSL SREG1
                    if (T[2]) begin
                        if (SrcReg1[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b11110;  // CSL A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b01110;  // CSL A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[3]) begin
                        RF_OutASel = 3'b100;    // S1
                        MuxDSel = 0;
                        // ALU_WF = 1; // burada da açılmalı mı acaba?
                        if (SrcReg1[2])
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        else 
                            ALU_FunSel = 5'b00000;  // A (16-bit)

                        if (DestReg[2]) begin
                            MuxASel = 2'b00;   // ALUOut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b00;  // ALUOut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                end
                6'b001111: begin // CSR   DSTREG <- CSR SREG1
                    if (T[2]) begin
                        if (SrcReg1[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b11111;  // CSR A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b01111;  // CSR A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[3]) begin
                        RF_OutASel = 3'b100;    // S1
                        MuxDSel = 0;
                        // ALU_WF = 1; // burada da açılmalı mı acaba?
                        if (SrcReg1[2])
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        else 
                            ALU_FunSel = 5'b00000;  // A (16-bit)

                        if (DestReg[2]) begin
                            MuxASel = 2'b00;   // ALUOut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b00;  // ALUOut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                end
                6'b010000: begin // NOT   DSTREG <- NOT SREG1
                    if (T[2]) begin
                        if (SrcReg1[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10010;  // NOT A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00010;  // NOT A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[3]) begin
                        RF_OutASel = 3'b100;    // S1
                        MuxDSel = 0;
                        // ALU_WF = 1; // burada da açılmalı mı acaba?
                        if (SrcReg1[2])
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        else 
                            ALU_FunSel = 5'b00000;  // A (16-bit)

                        if (DestReg[2]) begin
                            MuxASel = 2'b00;   // ALUOut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b00;  // ALUOut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                end
                6'b010001: begin // AND     DSTREG <- SREG1 AND SREG2
                    if (T[2]) begin   // S1 <- SREG1
                        if (SrcReg1[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[3]) begin   // S2 <- SREG2
                        if (SrcReg2[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg2[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg2[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b0100;    // S2
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[4]) begin // DSTREG <- S1 (operation) S2
                        RF_OutASel = 3'b100;    // S1
                        RF_OutBSel = 3'b101;    // S2
                        MuxDSel = 0;
                        ALU_WF = 1; 
                        ALU_FunSel = 5'b10111;  // A AND B (32-bit)

                        if (DestReg[2]) begin
                            MuxASel = 2'b00;   // ALUOut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b00;  // ALUOut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                end
                6'b010010: begin // ORR     DSTREG <- SREG1 OR SREG2
                    if (T[2]) begin   // S1 <- SREG1
                        if (SrcReg1[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[3]) begin   // S2 <- SREG2
                        if (SrcReg2[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg2[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg2[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b0100;    // S2
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[4]) begin // DSTREG <- S1 (operation) S2
                        RF_OutASel = 3'b100;    // S1
                        RF_OutBSel = 3'b101;    // S2
                        MuxDSel = 0;
                        ALU_WF = 1; 
                        ALU_FunSel = 5'b11000;  // A OR B (32-bit)

                        if (DestReg[2]) begin
                            MuxASel = 2'b00;   // ALUOut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b00;  // ALUOut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                    
                end
                6'b010011: begin // XOR     DSTREG <- SREG1 XOR SREG2
                    if (T[2]) begin   // S1 <- SREG1
                        if (SrcReg1[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[3]) begin   // S2 <- SREG2
                        if (SrcReg2[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg2[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg2[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b0100;    // S2
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[4]) begin // DSTREG <- S1 (operation) S2
                        RF_OutASel = 3'b100;    // S1
                        RF_OutBSel = 3'b101;    // S2
                        MuxDSel = 0;
                        ALU_WF = 1; 
                        ALU_FunSel = 5'b11001;  // A XOR B (32-bit)

                        if (DestReg[2]) begin
                            MuxASel = 2'b00;   // ALUOut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b00;  // ALUOut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                    
                end
                6'b010100: begin // NAND    DSTREG <- SREG1 NAND SREG2
                    if (T[2]) begin   // S1 <- SREG1
                        if (SrcReg1[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[3]) begin   // S2 <- SREG2
                        if (SrcReg2[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg2[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg2[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b0100;    // S2
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[4]) begin // DSTREG <- S1 (operation) S2
                        RF_OutASel = 3'b100;    // S1
                        RF_OutBSel = 3'b101;    // S2
                        MuxDSel = 0;
                        ALU_WF = 1; 
                        ALU_FunSel = 5'b11010;  // A NAND B (32-bit)

                        if (DestReg[2]) begin
                            MuxASel = 2'b00;   // ALUOut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b00;  // ALUOut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                    
                end
                6'b010101: begin // ADD     DSTREG <- SREG1 + SREG2
                    if (T[2]) begin   // S1 <- SREG1
                        if (SrcReg1[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[3]) begin   // S2 <- SREG2
                        if (SrcReg2[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg2[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg2[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b0100;    // S2
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[4]) begin // DSTREG <- S1 (operation) S2
                        RF_OutASel = 3'b100;    // S1
                        RF_OutBSel = 3'b101;    // S2
                        MuxDSel = 0;
                        ALU_WF = 1; 
                        ALU_FunSel = 5'b10100;  // A + B (32-bit)

                        if (DestReg[2]) begin
                            MuxASel = 2'b00;   // ALUOut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b00;  // ALUOut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                end

                6'b010110: begin // ADC     DSTREG <- SREG1 + SREG2 + CARRY
                    if (T[2]) begin   // S1 <- SREG1
                        if (SrcReg1[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[3]) begin   // S2 <- SREG2
                        if (SrcReg2[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg2[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg2[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b0100;    // S2
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[4]) begin // DSTREG <- S1 (operation) S2
                        RF_OutASel = 3'b100;    // S1
                        RF_OutBSel = 3'b101;    // S2
                        MuxDSel = 0;
                        ALU_WF = 1; 
                        ALU_FunSel = 5'b10101;  // A + B + Carry (32-bit)

                        if (DestReg[2]) begin
                            MuxASel = 2'b00;   // ALUOut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b00;  // ALUOut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                end

                6'b010111: begin // SUB     DSTREG <- SREG1 - SREG2
                    if (T[2]) begin   // S1 <- SREG1
                        if (SrcReg1[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b1000;    // S1
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[3]) begin   // S2 <- SREG2
                        if (SrcReg2[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg2[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg2[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end

                        ALU_WF = 1;
                        MuxASel = 2'b00;
                        RF_ScrSel = 4'b0100;    // S2
                        RF_FunSel = 3'b010;     // load
                    end
                    if (T[4]) begin // DSTREG <- S1 (operation) S2
                        RF_OutASel = 3'b100;    // S1
                        RF_OutBSel = 3'b101;    // S2
                        MuxDSel = 0;
                        ALU_WF = 1; 
                        ALU_FunSel = 5'b10110;  // A - B (32-bit)

                        if (DestReg[2]) begin
                            MuxASel = 2'b00;   // ALUOut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b00;  // ALUOut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                end

                6'b011000: begin // MOV     DSTREG <- SREG1
                    if (T[2]) begin
                        if (SrcReg1[2]) begin   // RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;  // A (32-bit)
                        end
                        else begin   // ARF registers
                            ARF_OutCSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;  // A (16-bit)
                        end

                        if (DestReg[2]) begin
                            MuxASel = 2'b00;   // ALUOut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b00;  // ALUOut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                end

                6'b011001: begin // MOVL   Rx[7:0] <- IMMEDIATE (8-bit)
                    if (T[2]) begin
                        MuxASel = 2'b11;   // IROut[7:0]
                        RF_RegSel = r_sel;
                        RF_FunSel = 3'b100;  // Load
                        ResetT();
                    end
                end

                6'b011010: begin // MOVSH   Rx[31-8] <- Rx[23-0] (8-bit Left Shift) | Rx[7-0] <- IMMEDIATE (8-bit)
                    if (T[2]) begin
                        MuxASel = 2'b11;   // IROut[7:0]
                        RF_RegSel = r_sel;
                        RF_FunSel = 3'b110;
                        ResetT();
                    end
                end

                6'b011011: begin // LDARL   DSTREG <- M[AR] (16-bit)
                    if (T[2]) begin  // // DR[7:0] <- M[AR] | AR <- AR + 1
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b01;
                        
                    end
                    else if (T[3]) begin  // DR[7:0] <- M[AR] 
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b10;  // left shift and load
                    end
                    else if (T[4]) begin
                        if (DestReg[2]) begin
                            MuxASel = 2'b10;   // DROut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b10;  // DROut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                end

                6'b011100: begin // LDARH   DSTREG <- M[AR] (32-bit)
                    if (T[2]) begin  // // DR[7:0] <- M[AR] | AR <- AR + 1
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b01;
                    end
                    else if (T[3]) begin  // // DR[7:0] <- M[AR] | AR <- AR + 1
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b10;
                    end
                    else if (T[4]) begin  // // DR[7:0] <- M[AR] | AR <- AR + 1
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b10;
                    end
                    else if (T[5]) begin  // DR[7:0] <- M[AR] 
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b10;  // left shift and load
                    end
                    else if (T[6]) begin
                        if (DestReg[2]) begin
                            MuxASel = 2'b10;   // DROut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b10;  // DROut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                end

                6'b011101: begin // STAR    M[AR] <- SREG1
                    if (T[2]) begin
                        if (SrcReg1[2]) begin// RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;
                            ALU_WF = 1;
                            MuxCSel = 2'b11;
                        end
                        else begin  // ARF registers
                            ARF_RegSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;
                            ALU_WF = 1;
                            MuxCSel = 2'b01;
                        end

                        ARF_OutDSel = 2'b10; // AR
                        Mem_CS = 0;
                        Mem_WR = 1;   // write active
                        ARF_RegSel = 3'b001;
                        ARF_FunSel = 2'b01;   // increment AR
                    end
                    if (T[3]) begin
                        if (SrcReg1[2]) begin// RF registers
                            RF_OutASel = {1'b0, SrcReg1[1:0]};
                            MuxDSel = 0;
                            ALU_FunSel = 5'b10000;
                            ALU_WF = 1;
                            MuxCSel = 2'b10;
                            ARF_RegSel = 3'b001;
                            ARF_FunSel = 2'b01;   // increment AR
                        end
                        else begin  // ARF registers
                            ARF_RegSel = SrcReg1[1:0];
                            MuxDSel = 1;
                            ALU_FunSel = 5'b00000;
                            ALU_WF = 1;
                            MuxCSel = 2'b00;
                            ResetT();   // 2 cycle is enough for arf registers (16-bit)
                        end

                        ARF_OutDSel = 2'b10; // AR
                        Mem_CS = 0;
                        Mem_WR = 1;   // write active
                    end
                    if (T[4]) begin
                        RF_OutASel = {1'b0, SrcReg1[1:0]};
                        MuxDSel = 0;
                        ALU_FunSel = 5'b10000;
                        ALU_WF = 1;
                        MuxCSel = 2'b01;

                        ARF_RegSel = 3'b001;
                        ARF_FunSel = 2'b01;   // increment AR
                        ARF_OutDSel = 2'b10; // AR
                        Mem_CS = 0;
                        Mem_WR = 1;   // write active
                    end
                    if (T[5]) begin
                        RF_OutASel = {1'b0, SrcReg1[1:0]};
                        MuxDSel = 0;
                        ALU_FunSel = 5'b10000;
                        ALU_WF = 1;
                        MuxCSel = 2'b00;
                        ARF_OutDSel = 2'b10; // AR
                        Mem_CS = 0;
                        Mem_WR = 1;   // write active
                        ResetT();
                    end
                end

                6'b011110: begin // LDAL   Rx[15:0] <- M[ADDRESS]
                    if (T[2]) begin   // AR <- ADDRESS
                        MuxBSel = 2'b11;
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b10;  // load

                    end
                    else if (T[3]) begin  // // DR[7:0] <- M[AR] | AR <- AR + 1
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b01;
                        
                    end
                    else if (T[4]) begin  // DR[7:0] <- M[AR] 
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b10;  // left shift and load
                        
                    end
                    else if (T[5]) begin  // Rx <- DR
                        MuxASel = 2'b10;   // DROut
                        RF_RegSel = r_sel; 
                        RF_FunSel = 3'b101;
                        ResetT();
                    end
                end

                6'b011111: begin // LDAH    Rx[31:0] <- M[ADDRESS]
                    if (T[2]) begin   // AR <- ADDRESS
                        MuxBSel = 2'b11;
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b10;  // load

                    end
                    else if (T[3]) begin  // // DR[7:0] <- M[AR] | AR <- AR + 1
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b01;
                    end
                    else if (T[4]) begin  // // DR[7:0] <- M[AR] | AR <- AR + 1
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b10;
                    end
                    else if (T[5]) begin  // // DR[7:0] <- M[AR] | AR <- AR + 1
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b10;
                    end
                    else if (T[6]) begin  // DR[7:0] <- M[AR] 
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b10;  // left shift and load
                    end
                    else if (T[7]) begin  // Rx <- DR
                        MuxASel = 2'b10;   // DROut
                        RF_RegSel = r_sel; 
                        RF_FunSel = 3'b010;
                        ResetT();
                    end
                end

                6'b100000: begin // STA     M[ADDRESS] <- Rx
                    if (T[2]) begin   // AR <- ADDRESS (IROut[7:0])
                        MuxBSel = 2'b11;
                        ARF_RegSel = 3'b001; // AR
                        ARF_FunSel = 2'b10;  // load
                    end
                    
                    if (T[3]) begin
                        RF_OutASel = {1'b0, RegSel};
                        MuxDSel = 0;
                        ALU_FunSel = 5'b10000;   // A
                        ALU_WF = 1;
                        MuxCSel = 2'b11;
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 1;
                    end
                    if (T[4]) begin
                        RF_OutASel = {1'b0, RegSel};
                        MuxDSel = 0;
                        ALU_FunSel = 5'b10000;   // A
                        ALU_WF = 1;
                        MuxCSel = 2'b10;
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 1;
                    end
                    if (T[5]) begin
                        RF_OutASel = {1'b0, RegSel};
                        MuxDSel = 0;
                        ALU_FunSel = 5'b10000;   // A
                        ALU_WF = 1;
                        MuxCSel = 2'b01;
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 1;
                    end
                    if (T[6]) begin
                        RF_OutASel = {1'b0, RegSel};
                        MuxDSel = 0;
                        ALU_FunSel = 5'b10000;   // A
                        ALU_WF = 1;
                        MuxCSel = 2'b00;
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 1;
                        ResetT();
                    end
                end

                6'b100001: begin // LDDRL   DR <- M[AR] (16-bit)
                    if (T[2]) begin  // // DR[7:0] <- M[AR] | AR <- AR + 1
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b01;
                        
                    end
                    else if (T[3]) begin  // DR[7:0] <- M[AR] 
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b10;  // left shift and load
                        ResetT();
                    end
                end

                6'b100010: begin // LDDRH   DR <- M[AR] (32-bit)
                    if (T[2]) begin  // // DR[7:0] <- M[AR] | AR <- AR + 1
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b01;
                    end
                    else if (T[3]) begin  // // DR[7:0] <- M[AR] | AR <- AR + 1
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b10;
                    end
                    else if (T[4]) begin  // // DR[7:0] <- M[AR] | AR <- AR + 1
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b10;
                    end
                    else if (T[5]) begin  // DR[7:0] <- M[AR] 
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 0;
                        DR_E = 1;
                        DR_FunSel = 2'b10;  // left shift and load
                        ResetT();
                    end
                end

                6'b100011: begin // STDR    DSTREG <- DR
                    if (T[2]) begin
                        if (DestReg[2]) begin
                            MuxASel = 2'b10;   // DROut
                            RF_RegSel = rf_dest;
                            RF_FunSel = 3'b010;  // load
                        end
                        else begin
                            MuxBSel = 2'b10;  // DROut
                            ARF_RegSel = arf_dest;
                            ARF_FunSel = 2'b10;  // load
                        end
                        ResetT();
                    end
                end

                6'b100100: begin // STRIM   M[AR+OFFSET] <- Rx (AR is 16-bit register) 
                    if (T[2]) begin  // S1 <- OFFSET
                        MuxASel = 2'b11;
                        RF_ScrSel = 4'b1000; // S1
                        RF_FunSel = 3'b100;  // clear and load
                        
                    end
                    if (T[3]) begin  // S2 <- AR
                        MuxASel = 2'b01;     // OutC
                        ARF_OutCSel = 2'b10; // AR
                        RF_ScrSel = 4'b0100; // S2
                        RF_FunSel = 3'b101;  // clear and load 16-bit
                    end
                    if (T[4]) begin  // AR <- S1 + S2
                        RF_OutASel = 3'b100;  // S1
                        RF_OutBSel = 3'b101;  // S2
                        MuxDSel = 0;
                        ALU_FunSel = 5'b00100; // A + B 16-bit
                        ALU_WF = 1;
                        MuxBSel = 2'b00;       // ALUOut
                        ARF_RegSel = 3'b001;   // AR
                        ARF_FunSel = 2'b10;    // Load
                    end
                    if (T[5]) begin
                        RF_OutASel = {1'b0, RegSel};
                        MuxDSel = 0;
                        ALU_FunSel = 5'b10000;   // A
                        ALU_WF = 1;
                        MuxCSel = 2'b11;
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 1;
                    end
                    if (T[6]) begin
                        RF_OutASel = {1'b0, RegSel};
                        MuxDSel = 0;
                        ALU_FunSel = 5'b10000;   // A
                        ALU_WF = 1;
                        MuxCSel = 2'b10;
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 1;
                    end
                    if (T[7]) begin
                        RF_OutASel = {1'b0, RegSel};
                        MuxDSel = 0;
                        ALU_FunSel = 5'b10000;   // A
                        ALU_WF = 1;
                        MuxCSel = 2'b01;
                        ARF_RegSel = 3'b001;  // AR
                        ARF_FunSel = 2'b01;   // increment
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 1;
                    end
                    if (T[8]) begin
                        RF_OutASel = {1'b0, RegSel};
                        MuxDSel = 0;
                        ALU_FunSel = 5'b10000;   // A
                        ALU_WF = 1;
                        MuxCSel = 2'b00;
                        ARF_OutDSel = 2'b10;   // AR
                        Mem_CS = 0; 
                        Mem_WR = 1;
                        ResetT();
                    end
                end
                default: ResetT(); // Invalid opcode, go to next instruction
            endcase
        end
    end
endmodule
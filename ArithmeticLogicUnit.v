`timescale 1ns / 1ps


module ArithmeticLogicUnit(
    input [31:0] A,
    input [31:0] B,
    input [4:0] FunSel,
    input WF,
    input Clock,
    output reg [31:0] ALUOut,
    output reg [3:0] FlagsOut  // Z C N O
    );

    wire c_in;
    assign c_in = FlagsOut[2];

    // reg [31:0] ALUOut;
    reg [3:0] next_FlagsOut;

    reg [32:0] sum_w_carry_32;
    reg [16:0] sum_w_carry_16;

    always @(*) begin
        if (FunSel[4]) begin    // 32-bit operations
            case (FunSel[3:0])
                4'b0000: begin
                    ALUOut = A;

                    next_FlagsOut[3] = (ALUOut == 16'b0);            // Zero flag
                    next_FlagsOut[1] = ALUOut[31];               // Negative flag

                end
                4'b0001: begin
                    ALUOut = B;

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[1] = ALUOut[31];               // Negative flag

                end
                4'b0010:begin
                    ALUOut = ~A;

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[1] = ALUOut[31];               // Negative flag

                end
                4'b0011:begin
                    ALUOut = ~B;

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[1] = ALUOut[31];               // Negative flag

                end
                4'b0100:begin
                    sum_w_carry_32 = {1'b0, A} + {1'b0, B};
                    ALUOut = sum_w_carry_32[31:0];

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[2] = sum_w_carry_32[32];       // Carry flag
                    next_FlagsOut[1] = ALUOut[31];               // Negative flag
                    next_FlagsOut[0] = (!A[31] && !B[31] && ALUOut[31] ||
                                        A[31] && B[31] && !ALUOut[31] );         // Overflow flag                        

                end
                4'b0101:begin
                    sum_w_carry_32 = {1'b0, A} + {1'b0, B} + c_in;
                    ALUOut = sum_w_carry_32[31:0];

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[2] = sum_w_carry_32[32];       // Carry flag
                    next_FlagsOut[1] = ALUOut[31];               // Negative flag
                    next_FlagsOut[0] = (!A[31] && !B[31] && ALUOut[31] ||
                                        A[31] && B[31] && !ALUOut[31] );         // Overflow flag                        

                end
                4'b0110:begin
                    sum_w_carry_32 = {1'b0, A} + {1'b0, (~B + 1'b1)};
                    ALUOut = sum_w_carry_32[31:0];

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[2] = sum_w_carry_32[32];       // Carry flag
                    next_FlagsOut[1] = ALUOut[31];               // Negative flag
                    next_FlagsOut[0] = (!A[31] && B[31] && ALUOut[31] ||
                                        A[31] && !B[31] && !ALUOut[31] );        // Overflow flag                        

                end
                4'b0111:begin
                    ALUOut = A & B;                             // AND operation

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[1] = ALUOut[31];               // Negative flag

                end
                4'b1000:begin
                    ALUOut = A | B;                             // OR operation

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[1] = ALUOut[31];               // Negative flag

                end
                4'b1001:begin
                    ALUOut = A ^ B;                             // XOR operation

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[1] = ALUOut[31];               // Negative flag

                end
                4'b1010:begin
                    ALUOut = ~(A & B);                          // NAND operation  

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[1] = ALUOut[31];               // Negative flag
       
                end
                4'b1011:begin 
                    ALUOut = {A[30:0], 1'b0};                   // LSL A

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[2] = A[31];                    // Carry flag
                    next_FlagsOut[1] = ALUOut[31];               // Negative flag

                end
                4'b1100:begin
                    ALUOut = {1'b0, A[31:1]};                   // LSR A

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[2] = A[0];                     // Carry flag
                    next_FlagsOut[1] = ALUOut[31];               // Negative flag

                end
                4'b1101:begin
                    ALUOut = {A[31], A[31:1]};                  // ASR A

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag

                end
                4'b1110:begin
                    ALUOut = {A[30:0], c_in};                   // CSL A

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[2] = A[31];                    // Carry flag
                    next_FlagsOut[1] = ALUOut[31];               // Negative flag

                end
                4'b1111:begin
                    ALUOut = {c_in, A[31:1]};                   // CSR A

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[2] = A[0];                     // Carry flag
                    next_FlagsOut[1] = ALUOut[31];               // Negative flag

                end
            endcase
        end

        // 16-bit operations
        else begin                         
            case (FunSel[3:0])
                4'b0000: begin
                    ALUOut = {16'b0, A[15:0]}; 

                    next_FlagsOut[3] = (ALUOut == 16'b0);            // Zero flag
                    
                    next_FlagsOut[1] = ALUOut[15];               // Negative flag
 

                end
                4'b0001: begin
                    ALUOut = {16'b0, B[15:0]};

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    
                    next_FlagsOut[1] = ALUOut[15];               // Negative flag
                        

                end
                4'b0010:begin
                    ALUOut = {16'b0, ~A[15:0]};

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    
                    next_FlagsOut[1] = ALUOut[15];               // Negative flag


                end
                4'b0011:begin
                    ALUOut = {16'b0, ~B[15:0]};

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    
                    next_FlagsOut[1] = ALUOut[15];               // Negative flag
                        

                end
                4'b0100:begin
                    sum_w_carry_16 = {1'b0, A[15:0]} + {1'b0, B[15:0]};
                    ALUOut = {16'b0, sum_w_carry_16[15:0]};

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[2] = sum_w_carry_16[16];       // Carry flag
                    next_FlagsOut[1] = ALUOut[15];               // Negative flag
                    next_FlagsOut[0] = (!A[15] && !B[15] && ALUOut[15] ||
                                        A[15] && B[15] && !ALUOut[15] );         // Overflow flag

                end
                4'b0101:begin
                    sum_w_carry_16 = {1'b0, A[15:0]} + {1'b0, B[15:0]} + c_in;
                    ALUOut = {16'b0, sum_w_carry_16[15:0]};

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[2] = sum_w_carry_16[16];       // Carry flag
                    next_FlagsOut[1] = ALUOut[15];               // Negative flag
                    next_FlagsOut[0] = (!A[15] && !B[15] && ALUOut[15] ||
                                        A[15] && B[15] && !ALUOut[15] );         // Overflow flag

                end
                4'b0110:begin
                    sum_w_carry_16 = {1'b0, A[15:0]} + {1'b0, (~B[15:0] + 1'b1)};
                    ALUOut = {16'b0, sum_w_carry_16[15:0]};

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[2] = sum_w_carry_16[16];       // Carry flag
                    next_FlagsOut[1] = ALUOut[15];               // Negative flag
                    next_FlagsOut[0] = (!A[15] && B[15] && ALUOut[15] ||
                                        A[15] && !B[15] && !ALUOut[15] );        // Overflow flag

                end
                4'b0111:begin
                    ALUOut = {16'b0, A[15:0] & B[15:0]};        // AND operation

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    
                    next_FlagsOut[1] = ALUOut[15];               // Negative flag


                end
                4'b1000:begin
                    ALUOut = {16'b0, A[15:0] | B[15:0]};        // OR operation

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    
                    next_FlagsOut[1] = ALUOut[15];               // Negative flag


                end
                4'b1001:begin
                    ALUOut = {16'b0, A[15:0] ^ B[15:0]};        // XOR operation

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    
                    next_FlagsOut[1] = ALUOut[15];               // Negative flag


                end
                4'b1010:begin
                    ALUOut = {16'b0, ~(A[15:0] & B[15:0])};     // NAND operation

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    
                    next_FlagsOut[1] = ALUOut[15];               // Negative flag


                end
                4'b1011:begin 
                    ALUOut = {16'b0, A[14:0], 1'b0};            // LSL A

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[2] = A[15];                    // Carry flag
                    next_FlagsOut[1] = ALUOut[15];               // Negative flag


                end
                4'b1100:begin
                    ALUOut = {16'b0, 1'b0, A[15:1]};            // LSR A

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[2] = A[0];                     // Carry flag
                    next_FlagsOut[1] = ALUOut[15];               // Negative flag


                end
                4'b1101:begin
                    ALUOut = {16'b0, A[15], A[15:1]};           // ASR A

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    

                end
                4'b1110:begin
                    ALUOut = {16'b0, A[14:0], c_in};            // CSL A

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[2] = A[15];                    // Carry flag
                    next_FlagsOut[1] = ALUOut[15];               // Negative flag


                end
                4'b1111:begin
                    ALUOut = {16'b0, c_in, A[15:1]};            // CSR A

                    next_FlagsOut[3] = (ALUOut == 0);            // Zero flag
                    next_FlagsOut[2] = A[0];                     // Carry flag
                    next_FlagsOut[1] = ALUOut[15];               // Negative flag

 
                end
            endcase
        end
    end

    always @(posedge Clock) begin
        if (WF) begin
            FlagsOut <= next_FlagsOut;
        end
    end

endmodule

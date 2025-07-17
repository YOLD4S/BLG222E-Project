// `timescale 1ns / 1ps

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
//     wire C_en = WF; //& (~f[0]&f[1] | f[1]&~f[3] | f[1]&f[2]);
//     wire N_en = WF & ~(f[0]&f[1]&~f[2]&f[3]);
//     wire O_en = WF; //& (~f[0]&f[1]&~f[2] + ~f[0]&f[1]&~f[3]);
    

//     // wire [31:0] real_A = widthSelect ? A : {{16{A[15]}}, A[15:0]};
//     // wire [31:0] real_B = widthSelect ? B : {{16{B[15]}}, B[15:0]};

//     assign ALUOut = (f == 4'b0000) ? A : 
//                  (f == 4'b0001) ? B :
//                  (f == 4'b0010) ? ~A :
//                  (f == 4'b0011) ? ~B :
//                  (f == 4'b0100) ? A + B :
//                  (f == 4'b0101) ? A + B + FlagsOut[2] : // A + B + C
//                  (f == 4'b0110) ? A - B :
//                  (f == 4'b0111) ? A & B :
//                  (f == 4'b1000) ? A | B :
//                  (f == 4'b1001) ? A ^ B :
//                  (f == 4'b1010) ? ~(A & B) :
//                  (f == 4'b1011) ? {A[31:1], 1'b0} : // LSL
//                  (f == 4'b1100) ? {1'b0, A[31:1]} : // LSR
//                  (f == 4'b1101) ? {A[31], A[31:1]} : // ASR
//                  (f == 4'b1110) ? {FlagsOut[2], A[31:1]} : // CSL
//                  {FlagsOut[2], A[31:1]}; // CSR

//     always @(posedge Clock) begin
//         if (Z_en)
//             FlagsOut[3] <= (ALUOut == 0);
//         if (C_en)
//             FlagsOut[2] <= widthSelect ? (A+B > 32'hFFFFFFFF) : (A[26] ^ B[26] ^ ALUOut[26]);
//         if (N_en)
//             FlagsOut[1] <= ALUOut[31];
//         if (O_en)
//             FlagsOut[0] <= FunSel[1] ? ((A[31] != B[31]) && (B[31] == ALUOut[31])) : ((A[31] == B[31]) && (ALUOut[31] != A[31]));
//     end

// endmodule


`timescale 1ns / 1ps
module ArithmeticLogicUnit(
    input  [31:0] A,
    input  [31:0] B,
    input  [4:0] FunSel,
    input  WF,
    input Clock,
    output reg [31:0] ALUOut,
    output reg [3:0] FlagsOut
    );
	
    wire [15:0] low_a; 
    wire [15:0] low_b;
    reg [16:0] bit17;
    assign low_a = A[15:0];
    assign low_b = B[15:0];
    reg [32:0] bit33 ;    
    reg C = 0;
    reg MSB;

always @(*) begin	
        case (FunSel)
            5'b00000: 
            begin
                ALUOut[31:16] = 16'b0;
                ALUOut[15:0] = low_a;
            end
            5'b00001:
            begin
                ALUOut[31:16] = 16'b0;
                ALUOut[15:0] = low_b;
            end
            5'b00010: 
            begin
                ALUOut[31:16] = 16'b0;
                ALUOut[15:0] = ~low_a;
            end
            5'b00011: 
            begin
                ALUOut[31:16] = 16'b0;
                ALUOut[15:0] = ~low_b;
            end
            5'b00100:
            begin 
                bit17 = low_a + low_b;
                ALUOut = bit17[15:0];
                ALUOut[31:16] = 16'b0;
            end
            5'b00101: 
            begin
                bit17 = low_a + low_b + FlagsOut[2];
                ALUOut = bit17[15:0];
                ALUOut[31:16] = 16'b0;
            end
            5'b00110:
            begin 
                bit17 = low_a + ~low_b + 1;
                ALUOut = bit17[15:0];
                ALUOut[31:16] = 16'b0;
            end
            5'b00111:
            begin
                ALUOut[15:0] = low_a & low_b;
                ALUOut[31:16] = 16'b0;
            end
            5'b01000: 
            begin
                ALUOut[15:0] = low_a | low_b;
                ALUOut[31:16] = 16'b0;
            end
            5'b01001: 
            begin
                ALUOut = {16'b0, low_a ^ low_b};  
            end
            5'b01010: 
            begin
                ALUOut = {16'b0, ~(low_a & low_b)};
            end  
            5'b01011: 
            begin
                C = low_a[15];
                ALUOut[15:0] = low_a << 1; //lsl
                ALUOut[0] = 0;
                ALUOut[31:16] = 16'b0;
            end
            5'b01100: 
            begin    
                C = low_a[0];   // LSR
                ALUOut[15:0] = low_a >> 1; 
                ALUOut[15] = 0;
                ALUOut[31:16] = 16'b0;
            end
            5'b01101: 
            begin
                MSB = low_a[15];    //ASR A
                C = low_a[0];
                ALUOut[15:0] = low_a >>> 1;
                ALUOut[15] = MSB;
                ALUOut[31:16] = 16'b0;
            end
            5'b01110: 
            begin  // CSL A
                ALUOut[15:0] = {low_a[14:0] ,FlagsOut[2]};
                C = low_a[15];
            end
            5'b01111: 
            begin  //CSR A
                ALUOut[15:0] = {FlagsOut[2], low_a[15:1]};
                C = low_a[0]; 
                ALUOut[31:16] = 16'b0;
            end
            5'b10000: 
            begin
                ALUOut = A;
            end  
             
            5'b10001: 
            begin
                ALUOut = B;
            end
    
            5'b10010: 
            begin
                ALUOut = ~A;
            end
            
            5'b10011: 
            begin
                ALUOut = ~B;
            end
            5'b10100: 
            begin
                bit33 = A + B;
    
                ALUOut = bit33[31:0];
                // FLAG UPDATE
    
            end 
            
            5'b10101: 
                begin 
                    bit33 = A + B;
                    bit33 = bit33 + FlagsOut[2]; 
                    ALUOut = bit33[31:0];     
                end
                         
            5'b10110: 
                begin
                    bit33 = A + ~B + 1;
                    ALUOut = bit33[31:0];  
                end
            
            5'b10111: 
            begin      
                ALUOut = A & B;
            end
            5'b11000:
            begin 
                ALUOut = A | B;     
            end
            5'b11001:  
            begin 
                ALUOut = A ^ B;     
            end
            5'b11010:  
            begin 
                ALUOut = ~(A & B);   
            end
            5'b11011:  
            begin
                C = A[31];   //LSL
                ALUOut[31:0] = A << 1; 
                ALUOut[0] = 0;
            end
            5'b11100:
            begin
                C = A[0];
                ALUOut[31:0] = A >> 1; //LSR
                ALUOut[31] = 0;
            end
            5'b11101: 
                begin
                    MSB = A[31];
                    C = A[0];
                    ALUOut[31:0] = A >>> 1; //ASR A
                    ALUOut[31] = MSB;
                end
                
            5'b11110:  
            begin   //CSL
                ALUOut[31:0] = {A[30:0] ,FlagsOut[2]};
                C = A[31];
            end
    
            5'b11111:  //CSR
            begin   
                ALUOut[31:0] = {FlagsOut[2], A[31:1]};
                C = A[0];
            end
            default : ALUOut = 32'b0;
             
        endcase  
    end
    always @(posedge Clock) begin
        if(WF) begin
        case (FunSel)
            5'b00000: 
            begin
                if(ALUOut == 16'b0) 
                  FlagsOut[3] <= 1;
                else FlagsOut[3] <= 0;
                if(ALUOut[15] == 1) 
                  FlagsOut[1] <= 1;
                else FlagsOut[1] <= 1'b0;
            end
            5'b00001:
            begin
                if(ALUOut == 16'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
                if(ALUOut[15] == 1) FlagsOut[1] <= 1; 
             else FlagsOut[1] <= 1'b0;
            end
            5'b00010: 
            begin
                if(ALUOut == 16'b0) FlagsOut[3] <= 1;
                else FlagsOut[3] <= 0;
                if(ALUOut[15] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 1'b0;
            end
            5'b00011: 
            begin
                if(ALUOut == 16'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
                if(ALUOut[15] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
            end
            5'b00100:
            begin 
                if(ALUOut == 16'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
                if(ALUOut[15] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] = 0;
                if(bit17[16] == 1) FlagsOut[2] <= 1; 
                else FlagsOut[2] = 0;
                if(low_a[15] == low_b[7] && low_a[15] == ~ALUOut[15]) FlagsOut[0] <= 1; 
                else FlagsOut[0] <= 0;
            end
            5'b00101: 
            begin
                if(ALUOut == 16'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] = 0;
                if(ALUOut[15] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] = 0;
                if(bit17[16] == 1) FlagsOut[2] <= 1; 
                else FlagsOut[2] = 0;
                if(low_a[15] == low_b[15] && low_a[15] == ~ALUOut[15]) FlagsOut[0] <= 1; 
                else FlagsOut[0] <= 0;
            end
            5'b00110:
            begin 
                if(ALUOut == 16'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] = 0;
                if(ALUOut[15] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
                if( A < B ) FlagsOut[2] <= 1; 
                else FlagsOut[2] <= 0;
                if(low_a[15] == ~low_b[15] && low_b[15] == ALUOut[15]) FlagsOut[0] <= 1; 
                else FlagsOut[0] <= 0;
            end
            5'b00111:
            begin
                if(ALUOut == 16'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
                if(ALUOut[15] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
            end
            5'b01000: 
            begin
                if(ALUOut == 16'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
                if(ALUOut[15] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
                FlagsOut[0] <= 0;
                FlagsOut[2] <= 0;
            end
            5'b01001: 
            begin
                if(ALUOut == 16'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
                if(ALUOut[15] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
            end
            5'b01010:
            begin
                if(ALUOut == 16'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
                if(ALUOut[15] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
            end
            5'b01011: 
            begin
                if(ALUOut == 16'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
                if(ALUOut[15] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
                if(C == 1) FlagsOut[2] <= 1;
                else FlagsOut[2] <= 0;
            end
            5'b01100: 
            begin   
                if(ALUOut == 16'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
                if(ALUOut[15] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] = 0;
                if(C==1) FlagsOut[2] <= 1;
                else FlagsOut[2] <= 0;
            end
           5'b01101: begin // ASR A
                if (ALUOut == 32'b0) FlagsOut[3] <= 1'b1; 
                else FlagsOut[3] <= 1'b0;
            end

            
            5'b01110: 
            begin  // CSL A
                if(ALUOut == 15'b0) FlagsOut[3] = 1;
                else FlagsOut[3] = 0;
                if(ALUOut[15] == 1) FlagsOut[1] = 1; 
                else FlagsOut[1] = 1'b0;
                if(C == 1) FlagsOut[2] = 1;
                else FlagsOut[2] = 0;
            end
            
            5'b01111: 
            begin 
                if(ALUOut == 16'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
                if(ALUOut[15] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 1'b0;  
                if(C == 1) FlagsOut[2] <= 1;
                else FlagsOut[2] <= 0;
            end
            
            5'b10000: 
            begin
                if (ALUOut[31] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
                if(ALUOut == 32'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
            end  
            
            5'b10001: 
            begin
                if (ALUOut[31] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
                if(ALUOut == 32'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
            end
            
            5'b10010: 
            begin
                if (ALUOut[31] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
                if(ALUOut == 32'b0) FlagsOut[3] <= 1;
                else FlagsOut[3] <= 0;
            end
            
            5'b10011: 
            begin
                if (ALUOut[31] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] = 0;
                if(ALUOut == 32'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
            end
            
            5'b10100: 
            begin
                if (bit33[31] == 1) FlagsOut[1] <= 1;
                else FlagsOut[1] <= 0;
                if(bit33 == 32'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] = 0;
                if(A[31] == B[31] && A[31] == ~bit33[31]) FlagsOut[0] <= 1; 
                else FlagsOut[0] <= 0;
                if(bit33[32] == 1) FlagsOut[2] <= 1;
                else FlagsOut[2] <= 0;
            end
            
            5'b10101: 
            begin
                if (bit33[31] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
                if(bit33[31:0] == 32'h0000) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
                if(A[31] == B[31] && A[31] == ~bit33[31]) FlagsOut[0] <= 1; 
                else FlagsOut[0] <= 0;
                if(bit33[32] == 1) FlagsOut[2] <= 1;
                else FlagsOut[2] <= 0;
            end
            
            5'b10110: 
            begin
                if (ALUOut[31] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
                if(ALUOut == 32'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
                if(A[31] == ~B[31] && B[31] == ALUOut[31]) FlagsOut[0] <= 1; 
                else FlagsOut[0] <= 0;
                if(A < B) FlagsOut[2] <= 1; 
                else FlagsOut[2] <= 0;   
            end
            
            5'b10111: 
            begin      
                if (ALUOut[31] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
                if(ALUOut == 32'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
            end
            
            5'b11000:
            begin    
                if (ALUOut[31] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
                if(ALUOut == 32'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
            end
            
            5'b11001:  
            begin     
                if (ALUOut[31] == 1) FlagsOut[1] <= 1;
                else FlagsOut[1] <= 0;
                if(ALUOut == 32'b0) FlagsOut[3] <= 1;
                else FlagsOut[3] <= 0;
            end
            
            5'b11010:  
            begin    
                if (ALUOut[31] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
                if(ALUOut == 32'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
            end
            
            5'b11011:  
            begin
                FlagsOut[2] <= C;
                if (ALUOut[31] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
                if(ALUOut == 32'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
                if(C == 1) FlagsOut[2] <= 1;
                else FlagsOut[2] <= 0;
            end
            
            5'b11100: 
            begin
                if(ALUOut == 32'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
                if(ALUOut[31] == 1) FlagsOut[1] <= 1;
                else FlagsOut[1] <= 0;
                if(C == 1) FlagsOut[2] <= 1;
                else FlagsOut[2] <= 0;
            end
            
            5'b11101: 
            begin
                FlagsOut[3] <= (ALUOut == 32'b0);  
            end
            
            5'b11110: 
            begin
                if(ALUOut == 32'b0) FlagsOut[3] <= 1;
                else FlagsOut[3] <= 0;
                if(ALUOut[31] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
                if(C == 1) FlagsOut[2] <= 1;
                else FlagsOut[2] <= 0;
            end
            
            5'b11111:  
            begin 
                if(ALUOut == 32'b0) FlagsOut[3] <= 1; 
                else FlagsOut[3] <= 0;
                if(ALUOut[31] == 1) FlagsOut[1] <= 1; 
                else FlagsOut[1] <= 0;
                if(C==1) FlagsOut[2] <= 1;
                else FlagsOut[2] <= 0;
            end
            default : FlagsOut<=4'b0;
        endcase
        end
    end
endmodule
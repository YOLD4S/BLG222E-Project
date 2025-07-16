`timescale 1ns / 1ps

module Register32bit (
    input wire Clock,
    input wire E,
    input wire [2:0] FunSel,
    input wire [31:0] I,
    output reg [31:0] Q
);

<<<<<<< HEAD
always @(posedge Clock) begin
=======
always @(posedge clk) begin
    if (E) begin
>>>>>>> b29c4b1a5aa3a0375053dbcdcf08c48ad12e46dd
        case (FunSel)
            3'b000: Q <= Q - 1;                         // Decrement
            3'b001: Q <= Q + 1;                         // Increment
            3'b010: Q <= I;                             // Load input I
            3'b011: Q <= 32'b0;                         // Clear
            3'b100: Q <= {24'b0, I[7:0]};               // Load lower 8 bits of I
            3'b101: Q <= {16'b0, I[15:0]};              // Load lower 16 bits of I
            3'b110: Q <= {Q[23:0], I[7:0]};              // Q (31-8) ← Q (23-0) (8-bit Left Shift)
            3'b111: Q <= {{16{I[15]}}, I[15:0]};          // Sign extend (16bit)
        endcase
    end
end
endmodule
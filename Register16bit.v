`timescale 1ns / 1ps

module Register16bit (
    input wire Clock,
    input wire E,
    input wire [1:0] FunSel,
    input wire [15:0] I,
    output reg [15:0] Q
);

  always @(posedge Clock) begin
    if (E) begin
      case (FunSel)
        2'b00: Q <= Q - 1;  // Decrement
        2'b01: Q <= Q + 1;  // Increment
        2'b10: Q <= I;  // Load input I
        2'b11: Q <= 16'b0;  // Clear
      endcase
    end
  end

endmodule

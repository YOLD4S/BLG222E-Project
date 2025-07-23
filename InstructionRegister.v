`timescale 1ns / 1ps

module InstructionRegister (
    input wire Clock,
    input wire Write,
    input wire LH,
    input wire [7:0] I,
    output reg [15:0] IROut
);

  always @(posedge Clock) begin
    if (Write) begin
      if (LH) begin
        IROut <= {I, IROut[7:0]};
      end else begin
        IROut <= {IROut[15:8], I};
      end
    end
  end

endmodule

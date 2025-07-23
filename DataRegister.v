`timescale 1ns / 1ps

module DataRegister (
    input wire Clock,
    input wire E,
    input wire [1:0] FunSel,
    input wire [7:0] I,
    output reg [31:0] DROut
);

  always @(posedge Clock) begin
    if (E) begin
      case (FunSel)
        2'b00: DROut <= {{24{I[7]}}, I};
        2'b01: DROut <= {24'b0, I};
        2'b10: DROut <= {DROut[23:0], I};
        2'b11: DROut <= {I, DROut[31:8]};
      endcase
    end
  end

endmodule

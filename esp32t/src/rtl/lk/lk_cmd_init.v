import lk_types::*;

module lk_cmd_init_t(
    input  wire  clk,
    input  reg   rx_valid;
    input  reg   rx_valid;
);

always @(*) begin
    out = in;
    if ((in.command == CMD_INIT) && !in.reset) begin
        out.tx_valid = 1'b1;
        out.tx_data = 8'hFF;
        out.command = CMD_IDLE;
    end
end

endmodule

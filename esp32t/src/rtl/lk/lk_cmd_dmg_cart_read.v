import lk_types::*;

module lk_cmd_dmg_cart_read_t(
    input  wire  clk,
    input  bus_t in,
    output bus_t out
);

typedef enum {
    S_EXEC,
    S_TX,
    S_COMPLETE
} state_t;
localparam state_t S_INIT = S_EXEC;
state_t state = S_INIT;

reg [15:0] address;
reg [15:0] remaining;

always @(posedge clk) begin
    out <= in;

    if (in.command != CMD_DMG_CART_READ) begin
        state <= S_INIT;
        address <= in.vars.address;
        remaining <= in.vars.transfer_size;
    end else begin
        case (state)
            S_EXEC: begin
                if (remaining == 0) begin
                    state <= S_COMPLETE;
                end else begin
                    remaining <= remaining - 1;
                    out.cart_request <= 1'b1;
                    out.cart <= cart_t' { address: address, default: '0 };
                    state <= S_TX;
                end
            end
            S_TX: if (in.cart_complete) begin
                address <= address + 1'b1;
                state <= remaining ? S_EXEC : S_COMPLETE;
                out.tx_valid <= 1;
                out.tx_data <= in.cart.data;
            end
            S_COMPLETE: begin
                out.command <= CMD_IDLE;
                out.vars.address <= in.vars.address + in.vars.transfer_size;
            end
            default: state <= S_COMPLETE;
        endcase
    end
end

endmodule
module lk_cmd_dmg_cart_read_t(
    input wire clk,
    input wire en,
    output wire complete,
    input wire [15:0] var_address_in,
    output reg [15:0] var_address_out,
    input wire [15:0] var_transfer_size_in,
    input wire [15:0] var_address,
    input wire [15:0] var_transfer_size,
    output reg cart_req,
    output reg [15:0] cart_a,
    input wire cart_complete
);

typedef enum {
    S_EXEC,
    S_TX,
    S_COMPLETE
} state_t;
localparam state_t S_INIT = S_EXEC;
state_t state = S_INIT;
assign complete = (state == S_COMPLETE);

reg [15:0] address;
reg [15:0] remaining;

always @(posedge clk) begin
    cart_req <= 0;

    if (!en) begin
        cart_a <= ~16'd0;
        state <= S_INIT;
        address <= var_address_in;
        remaining <= var_transfer_size_in;
        var_address_out <= var_address_in + var_transfer_size_in;
    end else begin
        case (state)
            S_EXEC: begin
                if (remaining == 0) begin
                    state <= S_COMPLETE;
                end else begin
                    remaining <= remaining - 1;
                    cart_req <= 1;
                    cart_a <= address;
                    state <= S_TX;
                end
            end
            S_TX: if (cart_complete) begin
                // lk_top directly transmits the cartridge data
                address <= address + 1'b1;
                state <= remaining ? S_EXEC : S_COMPLETE;
            end
            S_COMPLETE: ;
            default: state <= S_COMPLETE;
        endcase
    end
end

endmodule
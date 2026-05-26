import lk_types::*;

module lk_cmd_dmg_cart_read_t(
    input  wire        clk,
    input  wire        enable,
    output reg         complete,

    output reg         tx_valid,
    output reg  [7:0]  tx_data,

    input  wire        cart_req_ready,
    output reg         cart_req_valid,
    output reg  [15:0] cart_req_address,
    input  reg         cart_complete,
    input  reg  [7:0]  cart_complete_data,

    input  wire [15:0] var_address,
    input  wire [15:0] var_transfer_size
);

typedef enum {
    S_INIT,
    S_EXEC,
    S_WAIT,
    S_COMPLETE
} state_t;
state_t state;
always @(posedge clk) complete <= (state == S_COMPLETE);

reg [15:0] address;
reg [15:0] remaining;

reg [15:0] address_next;
reg [15:0] remaining_next;
always @(*) begin
    address_next = address;
    remaining_next = remaining;
    unique case(state)
        S_INIT: begin
            address_next = var_address;
            remaining_next = var_transfer_size;
        end
        S_EXEC: begin
            if (cart_req_ready) begin
                address_next = address + 1'd1;
                remaining_next = remaining - 1'd1;
            end
        end
        default: ;
    endcase
end
always @(posedge clk) begin
    address <= address_next;
    remaining <= remaining_next;
end

reg [15:0] waiting;
reg [15:0] waiting_next;

// Only using for S_EXEC as in other states, we're not going to queue up another request,
// even if the cart request queue is ready
wire signed [1:0] exec_waiting_delta = { 1'b0, cart_req_ready } - { 1'b0, cart_complete };

always @(*) begin
    waiting_next = waiting;
    unique case (state)
        S_INIT: waiting_next = 16'd0;
        S_EXEC: waiting_next = waiting + exec_waiting_delta;
        S_WAIT: if (cart_complete) waiting_next = waiting - 1'd1;
        default: ;
    endcase
end
always @(posedge clk) waiting <= waiting_next;

wire req_valid_next = (state == S_EXEC) && cart_req_ready;

always @(posedge clk) begin
    cart_req_valid <= req_valid_next;
    cart_req_address <= address;
end

reg tx_valid_next;
reg [7:0] tx_data_next;
always @(*) begin
    tx_valid_next = 1'b0;
    tx_data_next = 8'd0;

    if (enable && cart_complete) begin
        tx_valid_next = 1'b1;
        tx_data_next = cart_complete_data;
    end
end
always @(posedge clk) begin
    tx_valid <= tx_valid_next;
    tx_data <= tx_data_next;
end

state_t state_next;
always @(*) begin
    state_next = state;
    if (!enable) begin
        state_next = S_INIT;
    end else begin
        unique case(state)
            S_INIT: state_next = S_EXEC;
            S_EXEC: if (remaining == 0) state_next = S_WAIT;
            S_WAIT: if (waiting == 0) state_next = S_COMPLETE;
            default: ;
        endcase
    end
end
always @(posedge clk) state <= state_next;

endmodule
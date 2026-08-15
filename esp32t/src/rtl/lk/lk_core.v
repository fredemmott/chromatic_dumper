// cart_reader.v
import lk_types::*;
module lk_core #(
)(
    input  wire        clk,
    input  wire        reset,

    // Parallel byte interface (EP3 via usbuvcuart_top)
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    output reg         tx_valid,
    output reg  [7:0]  tx_data,

    output reg         cart_enabled,

    output reg         enqueue_o,
    output cart_req_t  req_o,
    output cart_vars_t vars_o,

    output reg         dequeue_o,
    input wire         cart_complete,
    input wire [7:0]   cart_complete_data
);

reg rx_valid_r;
reg [7:0] rx_data_r;

typedef enum {
    CMD_DISABLE_CART,
    CMD_SET_VARIABLES,
    CMD_ENQUEUE,
    CMD_POLL,
    CMD_PING,
    CMD_DEBUG,
    CMD_IDLE
} command_t;
localparam command_t CMD_COUNT = CMD_IDLE;

command_t command;

logic [CMD_COUNT - 1:0] complete_bus;
wire command_complete = |complete_bus;

typedef enum {
    S_RESET,
    S_INIT_ACK,
    S_EXEC,
    S_IDLE
} state_t;
state_t state;

state_t state_next;
command_t command_next;
always @(*) begin
    state_next = state;
    command_next = CMD_IDLE;
    unique case (state)
        S_RESET: state_next = S_INIT_ACK;
        S_INIT_ACK: state_next = S_IDLE;
        S_IDLE: if (rx_valid_r) begin
            state_next = S_EXEC;
            unique case (rx_data_r[2:0])
                3'h00: command_next = CMD_DISABLE_CART;
                3'h01: command_next = CMD_SET_VARIABLES;
                3'h02: command_next = CMD_ENQUEUE;
                3'h03: command_next = CMD_POLL;
                3'h04: command_next = CMD_PING;
                3'h05: command_next = CMD_DEBUG;
                default: begin
                    state_next = S_IDLE;
                end
            endcase
        end
        S_EXEC: begin
            command_next = command;
            if (command_complete) begin
                command_next = CMD_IDLE;
                state_next = S_IDLE;
            end
        end
        default: ;
    endcase
end

always @(posedge clk) begin
    if (reset) begin
        state <= S_RESET;
        command <= CMD_IDLE;
    end else begin
        state <= state_next;
        command <= command_next;
    end
end

always @(posedge clk) begin
    complete_bus[CMD_DISABLE_CART] <= 1'b0;
    if (reset) begin
        cart_enabled <= 1'b0;
    end else if (command == CMD_DISABLE_CART) begin
        cart_enabled <= 1'b0;
        complete_bus[CMD_DISABLE_CART] <= 1'b1;
    end else if (command == CMD_ENQUEUE) begin
        cart_enabled <= 1'b1;
    end
end

logic PING_tx_valid;
logic [7:0] PING_tx_data;
always @(posedge clk) begin
    complete_bus[CMD_PING] <= 1'b0;
    PING_tx_valid <= 1'b0;
    PING_tx_data <= 8'd0;
    if ((command == CMD_PING) && rx_valid_r) begin
        complete_bus[CMD_PING] <= 1'b1;
        PING_tx_valid <= 1'b1;
        PING_tx_data <= ~rx_data_r;
    end
end

always @(posedge clk) begin
    complete_bus[CMD_SET_VARIABLES] <= 1'b0;
    if (reset) begin
        vars_o <= '{default: 0};
    end else if ((command == CMD_SET_VARIABLES) && rx_valid_r) begin
        complete_bus[CMD_SET_VARIABLES] <= 1'b1;
        vars_o <= '{
            flash_we_pin: rx_data_r[1:0],
            dmg_read_cs_pulse: rx_data_r[2],
            dmg_write_cs_pulse: rx_data_r[3]
        };
    end
end

typedef enum {
    ENQUEUE_RX_COUNT,
    ENQUEUE_RX_BYTE_0,
    ENQUEUE_RX_BYTE_1,
    ENQUEUE_RX_BYTE_2,
    ENQUEUE_RX_BYTE_3,
    ENQUEUE_COMPLETE
} ENQUEUE_state_t;
ENQUEUE_state_t ENQUEUE_state;
logic [7:0] ENQUEUE_remaining;

ENQUEUE_state_t ENQUEUE_state_next;
always @(*) begin
    ENQUEUE_state_next = ENQUEUE_state;
    unique case (ENQUEUE_state)
        ENQUEUE_RX_COUNT: if (rx_valid_r) ENQUEUE_state_next = ENQUEUE_RX_BYTE_0;
        ENQUEUE_RX_BYTE_0: if (rx_valid_r) ENQUEUE_state_next = ENQUEUE_RX_BYTE_1;
        ENQUEUE_RX_BYTE_1: if (rx_valid_r) ENQUEUE_state_next = ENQUEUE_RX_BYTE_2;
        ENQUEUE_RX_BYTE_2: if (rx_valid_r) ENQUEUE_state_next = ENQUEUE_RX_BYTE_3;
        ENQUEUE_RX_BYTE_3: if (rx_valid_r) begin
            if (ENQUEUE_remaining) ENQUEUE_state_next = ENQUEUE_RX_BYTE_0;
            else ENQUEUE_state_next = ENQUEUE_COMPLETE;
        end
        default: ;
    endcase
end

always @(posedge clk) begin
    complete_bus[CMD_ENQUEUE] <= 1'b0;
    if (command != CMD_ENQUEUE) begin
        ENQUEUE_state <= ENQUEUE_RX_COUNT;
    end else begin
        complete_bus[CMD_ENQUEUE] <= (ENQUEUE_state_next == ENQUEUE_COMPLETE);
        ENQUEUE_state <= ENQUEUE_state_next;
    end
end

always @(posedge clk) begin
    enqueue_o <= 1'b0;
    if (command != CMD_ENQUEUE) begin
        ENQUEUE_remaining <= 8'd0;
    end else if (rx_valid_r) begin
        unique case (ENQUEUE_state)
            ENQUEUE_RX_COUNT: ENQUEUE_remaining <= rx_data_r;
            ENQUEUE_RX_BYTE_0: begin
                req_o.address[15:8] <= rx_data_r;
                ENQUEUE_remaining <= ENQUEUE_remaining - 8'd1;
            end
            ENQUEUE_RX_BYTE_1: req_o.address[7:0] <= rx_data_r;
            ENQUEUE_RX_BYTE_2: req_o.data <= rx_data_r;
            ENQUEUE_RX_BYTE_3: begin
                enqueue_o <= 1'b1;
                req_o.is_write <= rx_data_r[0];
                req_o.is_flash <= rx_data_r[1];
                req_o.wait_for_status <= rx_data_r[2];
            end
            default: ;
        endcase
    end
end

logic POLL_tx_valid;
logic [7:0] POLL_tx_data;

logic [15:0] POLL_timeout;
always @(posedge clk) begin
    if ((command != CMD_POLL) || POLL_tx_valid) begin
        POLL_timeout <= 16'hFFFF; // ~ 1ms on a 14.901ns xClk
    end else begin
        POLL_timeout <= POLL_timeout - 1;
    end
end

typedef enum {
    POLL_RX_COUNT,
    POLL_EXEC,
    POLL_COMPLETE
} POLL_state_t;
POLL_state_t POLL_state;
always @(posedge clk) complete_bus[CMD_POLL] <= POLL_state == POLL_COMPLETE;

logic [7:0] POLL_dequeue_remaining;

always @(posedge clk) begin
    POLL_tx_valid <= 1'b0;
    POLL_tx_data <= 8'd0;
    dequeue_o <= 1'b0;
    if (command != CMD_POLL) begin
        POLL_dequeue_remaining <= 8'd0;
        POLL_state <= POLL_RX_COUNT;
    end else if (!POLL_timeout) begin
        POLL_state <= POLL_COMPLETE;
    end else begin
        unique case (POLL_state)
            POLL_RX_COUNT: if (rx_valid_r) begin
                POLL_dequeue_remaining <= rx_data_r;
                POLL_state <= POLL_EXEC;
            end
            POLL_EXEC: begin
                if (POLL_dequeue_remaining == 0) begin
                    POLL_state <= POLL_COMPLETE;
                end else if (cart_complete && !dequeue_o) begin
                    dequeue_o <= 1'b1;
                    POLL_tx_valid <= 1'b1;
                    POLL_tx_data <= cart_complete_data;
                    POLL_dequeue_remaining <= POLL_dequeue_remaining - 8'd1;
                end
            end
            default: ;
        endcase
    end
end

always @(posedge clk) begin
    rx_valid_r <= rx_valid;
    rx_data_r <= rx_data;
end

logic DBG_tx_valid;
logic [7:0] DBG_tx_data;

logic tx_valid_exec;
logic [7:0] tx_data_exec;
always @(*) begin
    tx_valid_exec = 0;
    tx_data_exec = 0;
    unique case (command)
        CMD_POLL: begin
            tx_valid_exec = POLL_tx_valid;
            tx_data_exec = POLL_tx_data;
        end
        CMD_DEBUG: begin
            tx_valid_exec = DBG_tx_valid;
            tx_data_exec = DBG_tx_data;
        end
        CMD_PING: begin
            tx_valid_exec = PING_tx_valid;
            tx_data_exec = PING_tx_data;
        end
        default: if (command_complete) begin
            tx_valid_exec = 1'b1;
            tx_data_exec = 8'h01;
        end
    endcase
end

always @(posedge clk) begin
    tx_valid <= 1'b0;
    tx_data <= 8'd0;

    unique case (state)
        S_INIT_ACK: begin
            tx_valid <= 1'b1;
            tx_data <= 8'hFF;
        end
        S_EXEC: begin
            tx_valid <= tx_valid_exec;
            tx_data <= tx_data_exec;
        end
        default: ;
    endcase
end

reg [15:0] enqueue_count;
reg [15:0] dequeue_count;

reg [2:0] DBG_idx;

always @(posedge clk) begin
    complete_bus[CMD_DEBUG] <= 1'b0;
    DBG_tx_valid <= 1'b0;
    if (reset) begin
        enqueue_count <= '0;
        dequeue_count <= '0;
    end else begin
        if (enqueue_o) enqueue_count <= enqueue_count + 16'b1;
        if (dequeue_o) dequeue_count <= dequeue_count + 16'd1;
        if (command != CMD_DEBUG) begin
            DBG_idx <= 1'b0;
        end else begin
            DBG_idx <= DBG_idx + 3'd1;
            unique case (DBG_idx)
                0: begin
                    DBG_tx_valid <= 1'b1;
                    DBG_tx_data <= enqueue_count[15:8];
                end
                1: begin
                    DBG_tx_valid <= 1'b1;
                    DBG_tx_data <= enqueue_count[7:0];
                end
                2: begin
                    DBG_tx_valid <= 1'b1;
                    DBG_tx_data <= dequeue_count[15:8];
                end
                3: begin
                    DBG_tx_valid <= 1'b1;
                    DBG_tx_data <= dequeue_count[7:0];
                    complete_bus[CMD_DEBUG] <= 1'b1;
                end
                default: ;
            endcase
        end
    end
end

endmodule // cart_reader
`default_nettype wire

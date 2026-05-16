module lk_vars_t(
    input wire clk,
    input wire reset,
    output reg complete,
    output wire var_dmg_read_cs_pulse,
    output wire var_dmg_write_cs_pulse,
    input wire rx_valid,
    input wire [7:0] rx_data,
    output reg tx_valid,
    output reg [7:0] tx_data,
    input wire get_variable_en,
    input wire set_variable_en
);

function [9:0] make_var16_id(
    input [7:0] size,
    input [7:0] key
);
    // Give all sizes are 1, 2, or 4, the lowest bit doesn't actually give us any information...
    // strip it off, then negate it so we can use null as a sentinel
    // size: 1 -> 001 -> 00 -> ~ 11
    // size: 2 -> 010 -> 01 -> ~ 10
    // size: 4 -> 100 -> 10 -> ~ 01
    begin
        make_var16_id = {~size[2:1], key[7:0]};
    end
endfunction

localparam VAR_ID_NONE = 0;

// ============================================================
// Firmware variable indices (match DEVICE_VAR in LK_Device.py)
// ============================================================

localparam VAR_ID_ADDRESS = make_var16_id(8'd4, 8'h00);
localparam VAR_ID_TRANSFER_SIZE = make_var16_id(8'd2, 8'h00);
localparam VAR_ID_STATUS_REGISTER = make_var16_id(8'd2, 8'h03);
localparam VAR_ID_CART_MODE = make_var16_id(8'd1, 8'h00);
localparam VAR_ID_DMG_ACCESS_MODE = make_var16_id(8'd1, 8'h01);
localparam VAR_ID_FLASH_WE_PIN = make_var16_id(8'd1, 8'h04);
localparam VAR_ID_DMG_READ_CS_PULSE = make_var16_id(8'd1, 8'h08);
localparam VAR_ID_DMG_WRITE_CS_PULSE = make_var16_id(8'd1, 8'h09);

localparam FWE_PIN_WR = 1;
localparam FWE_PIN_AUDIO = 2;
localparam FWE_PIN_WR_AND_RESET = 3;

struct packed {
    reg [15:0] address;
    reg [15:0] transfer_size;
    reg [7:0]  status_register;
    reg [7:0]  cart_mode;
    reg [7:0]  dmg_access_mode;
    reg [1:0]  flash_we_pin;
    reg        dmg_read_cs_pulse;
    reg        dmg_write_cs_pulse;
} storage = '{default:0};
assign var_dmg_read_cs_pulse = storage.dmg_read_cs_pulse;
assign var_dmg_write_cs_pulse = storage.dmg_read_cs_pulse;

function [15:0] get_var16(
    reg [7:0] size,
    reg [7:0] key
);
    begin
        get_var16 = 16'd0;
        case(make_var16_id(size, key))
            VAR_ID_ADDRESS: get_var16 = storage.address;
            VAR_ID_TRANSFER_SIZE: get_var16 = storage.transfer_size;
            VAR_ID_STATUS_REGISTER: get_var16[7:0] = storage.status_register;
            VAR_ID_CART_MODE: get_var16[7:0] = storage.cart_mode;
            VAR_ID_DMG_ACCESS_MODE: get_var16[7:0] = storage.dmg_access_mode;
            VAR_ID_FLASH_WE_PIN: get_var16[1:0] = storage.flash_we_pin;
            VAR_ID_DMG_READ_CS_PULSE: get_var16[0] = storage.dmg_read_cs_pulse;
            VAR_ID_DMG_WRITE_CS_PULSE: get_var16[0] = storage.dmg_write_cs_pulse;
            default: ;
        endcase
    end
endfunction

task set_var16(
    reg [7:0] size,
    reg [7:0] key,
    reg [15:0] data
);
    begin
        case (make_var16_id(size, key))
            VAR_ID_ADDRESS: storage.address <= data;
            VAR_ID_TRANSFER_SIZE: storage.transfer_size <= data;
            VAR_ID_STATUS_REGISTER: storage.status_register <= data[7:0];
            VAR_ID_CART_MODE: storage.cart_mode <= data[7:0];
            VAR_ID_DMG_ACCESS_MODE: storage.dmg_access_mode <= data[7:0];
            VAR_ID_FLASH_WE_PIN: storage.flash_we_pin <= data[1:0];
            VAR_ID_DMG_READ_CS_PULSE: storage.dmg_read_cs_pulse <= data[0];
            VAR_ID_DMG_WRITE_CS_PULSE: storage.dmg_write_cs_pulse <= data[0];
            default: ;
        endcase
    end
endtask

reg get_complete;
reg get_tx_valid;
reg [7:0] get_tx_data;

reg set_complete;
reg set_tx_valid;
reg [7:0] set_tx_data;

always_comb begin
    tx_valid = 0;
    tx_data = 0;
    complete = 0;
    if (get_variable_en) begin
        tx_valid = get_tx_valid;
        tx_data = get_tx_data;
        complete = get_complete;
    end else if (set_variable_en) begin
        tx_valid = set_tx_valid;
        tx_data = set_tx_data;
        complete = set_complete;
    end
end

enum {
    GS_RX,
    GS_TX,
    GS_COMPLETE
} get_state = GS_RX;
assign get_complete = (get_state == GS_COMPLETE);

reg [2:0] get_idx = 0;
reg [7:0] get_size;
reg [15:0] get_value;

always @(posedge clk) begin
    get_tx_valid <= 0;
    get_tx_data <= 0;

    if (!get_variable_en) begin
        get_idx <= 0;
        get_state <= GS_RX;
    end else begin
        case (get_state)
            GS_RX: begin
                if (rx_valid) begin
                    get_idx <= get_idx + 1;
                    case (get_idx)
                        0: get_size <= rx_data;
                        1, 2, 3: /* unused MSB */ ;
                        4: begin
                            get_idx <= 0;
                            get_value <= get_var16(get_size, rx_data);
                            get_state <= GS_TX;
                        end
                        default: get_state <= GS_COMPLETE;
                    endcase
                end
            end
            GS_TX: begin
                get_tx_valid <= 1;
                get_idx <= get_idx + 1;
                case(get_idx)
                    0, 1: ;
                    2: get_tx_data <= get_value[15:8];
                    3: begin
                        get_tx_data <= get_value[7:0];
                        get_state <= GS_COMPLETE;
                     end
                    default: get_state <= GS_COMPLETE;
                endcase
            end
            GS_COMPLETE: ;
            default: get_state <= GS_COMPLETE;
        endcase
    end
end

enum {
    SS_RX,
    SS_COMPLETE
} set_state = SS_RX;
assign set_complete = (set_state == SS_COMPLETE);

reg [3:0] set_idx = 0;
reg [7:0] set_size;
reg [7:0] set_key;
reg [7:0] set_value_msb;

always @(posedge clk) begin
    set_tx_valid <= 0;
    set_tx_data <= 0;

    if (reset) begin
        storage <= '{default:0};
    end

    if (!set_variable_en) begin
        set_idx <= 0;
        set_state <= SS_RX;
    end else begin
        case (set_state)
            SS_RX: begin
                if (rx_valid) begin
                    set_idx <= set_idx + 1;
                    case (set_idx)
                        0: set_size <= rx_data;
                        1, 2, 3: /* unused key AGB MSB */ ;
                        4: set_key <= rx_data;
                        5, 6: /* unused value AGB MSB */ ;
                        7: set_value_msb <= rx_data;
                        8: begin
                            set_var16(set_size, set_key, { set_value_msb, rx_data });
                            set_tx_valid <= 1;
                            set_tx_data <= 8'h01;
                            set_state <= SS_COMPLETE;
                        end
                    endcase
                end
            end
            SS_COMPLETE: ;
            default: set_state <= SS_COMPLETE;
        endcase
    end
end

endmodule

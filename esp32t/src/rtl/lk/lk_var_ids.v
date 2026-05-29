package lk_var_ids;

// Given all sizes are 1, 2, or 4, the lowest bit doesn't actually give us any information...
// strip it off, then negate it so we can use null as a sentinel
// size: 1 -> 001 -> 00 -> ~ 11
// size: 2 -> 010 -> 01 -> ~ 10
// size: 4 -> 100 -> 10 -> ~ 01
//
// Given the firmware version we indicate, the highest variable key is h11 == d17 == b1_0001
// only looking at the lowest 5 bits gives us space up to h1F == d31 which seems plenty of breathing room
//
// So, 2 bits for size, 5 bits for key, 7 bits
typedef reg[6:0] var_id_t;
localparam var_id_t VAR_ID_INVALID = 7'b0;
localparam VAR_KEY_MAX = 8'b1_1111;

function var_id_t decode_var_id(
    input [7:0] size,
    input [7:0] key
);
    if (key > VAR_KEY_MAX) return VAR_ID_INVALID;
    return {~size[2:1], key[4:0]};
endfunction

localparam VAR_ID_ADDRESS = decode_var_id(8'd4, 8'h00);
localparam VAR_ID_TRANSFER_SIZE = decode_var_id(8'd2, 8'h00);
localparam VAR_ID_STATUS_REGISTER = decode_var_id(8'd2, 8'h03);
localparam VAR_ID_CART_MODE = decode_var_id(8'd1, 8'h00);
localparam VAR_ID_DMG_ACCESS_MODE = decode_var_id(8'd1, 8'h01);
localparam VAR_ID_FLASH_WE_PIN = decode_var_id(8'd1, 8'h04);
localparam VAR_ID_DMG_READ_CS_PULSE = decode_var_id(8'd1, 8'h08);
localparam VAR_ID_DMG_WRITE_CS_PULSE = decode_var_id(8'd1, 8'h09);

endpackage
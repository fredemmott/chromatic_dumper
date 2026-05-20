import lk_types::*;

module lk_cmd_idle_t(
    input  wire  clk,
    input  bus_t in,
    output bus_t out
);

function command_t parse(input [7:0] rx_data);
    parse = CMD_IDLE;
    case (rx_data)
        // Must match `DEVICE_CMD` in `LK_Device.py`
        8'hA1: parse = CMD_QUERY_FW_INFO;
        8'hA3: parse = CMD_SET_MODE_DMG;
        8'hA5: parse = CMD_SET_VOLTAGE_5V;
        8'hA6: parse = CMD_SET_VARIABLE;
        8'hA7: parse = CMD_SET_FLASH_CMD;
        8'hA8: parse = CMD_SET_ADDR_AS_INPUTS;
        8'hA9: parse = CMD_CLK_TOGGLE;
        8'hAC: parse = CMD_DISABLE_PULLUPS;
        8'hAD: parse = CMD_GET_VARIABLE;
        8'hAE: parse = CMD_GET_VAR_STATE;
        8'hAF: parse = CMD_SET_VAR_STATE;
        8'hB1: parse = CMD_DMG_CART_READ;
        8'hB2: parse = CMD_DMG_CART_WRITE;
        8'hB3: parse = CMD_DMG_CART_WRITE_SRAM;
        8'hB4: parse = CMD_DMG_MBC_RESET;
        8'hB8: parse = CMD_DMG_SET_BANK_CHANGE_CMD;
        8'hBA: parse = CMD_DMG_CART_READ_MEASURE;
        8'hD1: parse = CMD_DMG_FLASH_WRITE_BYTE;
        8'hD3: parse = CMD_FLASH_PROGRAM;
        8'hD4: parse = CMD_CART_WRITE_FLASH_CMD;
        8'hD5: parse = CMD_CALC_CRC32;
        8'hF5: parse = CMD_SET_PIN;
        default: ;
    endcase
endfunction

command_t next_command;
assign next_command = (in.command == CMD_IDLE && in.rx_valid) ? parse(in.rx_data) : in.command;

always @(*) begin
    out = in;
    out.command = next_command;
end

endmodule
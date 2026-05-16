module lk_cmd_query_fw_info_t(
    input wire clk,
    input wire en,
    output wire complete,
    input wire rx_valid,
    input wire [7:0] rx_data,
    output wire tx_valid,
    output reg [7:0] tx_data
);
    localparam ROM_LEN = 26;
    reg [7:0] rom [0:ROM_LEN-1];
    reg [4:0] index = 0;

    wire valid_index = (index < ROM_LEN);
    assign tx_valid = valid_index;
    assign tx_data = valid_index ? rom[index] : 8'd0;
    assign complete = !valid_index;

    initial begin
        // FW info buffer
        // size=8
        rom[0]  = 8'd8;
        // cfw_id = 'L'  (uses LK protocol, but pcb_ver 0x42 ∉ Joey-Jr PCB_VERSIONS)
        rom[1]  = "L";
        // fw_ver = 12  (big-endian 16-bit)
        rom[2]  = 8'd0;
        rom[3]  = 8'd12;
        // pcb_ver = 0x42  (not in Joey-Jr's PCB_VERSIONS → rejected by hw_JoeyJr.py)
        rom[4]  = 8'h42;
        // fw_ts = 0x6A07ACAC (2026-05-15)
        rom[5]  = 8'h6A;
        rom[6]  = 8'h07;
        rom[7]  = 8'hAC;
        rom[8]  = 8'hAC;
        // name_len = 14  ("Chromatic Cart")
        rom[9]  = 8'd14;
        rom[10] = "C";
        rom[11] = "h";
        rom[12] = "r";
        rom[13] = "o";
        rom[14] = "m";
        rom[15] = "a";
        rom[16] = "t";
        rom[17] = "i";
        rom[18] = "c";
        rom[19] = " ";
        rom[20] = "C";
        rom[21] = "a";
        rom[22] = "r";
        rom[23] = "t";
        // cart_power_ctrl = 0
        rom[24] = 8'd0;
        // bootloader_reset = 0
        rom[25] = 8'd0;
    end

    always @(posedge clk) begin
        if (!en) begin
            index <= 0;
        end else if (valid_index) begin
            index <= index + 1;
        end
    end
endmodule
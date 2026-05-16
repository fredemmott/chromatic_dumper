module lk_cmd_query_fw_info_t(
    input wire clk,
    input wire en,
    output wire complete,
    input wire rx_valid,
    input wire [7:0] rx_data,
    output wire tx_valid,
    output wire [7:0] tx_data
);
    localparam ROM_LEN = 14;
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
        // fw_ver = 12  (big-endian 16-bit) - "LK" protocol version
        rom[2]  = 8'd0;
        rom[3]  = 8'd12;
        // pcb_ver
        rom[4]  = 8'd1;
        // fw_ts - unix timestamp, 1778887852 == 0x6A07ACAC (2026-05-15)
        rom[5]  = 8'h6A;
        rom[6]  = 8'h07;
        rom[7]  = 8'hAC;
        rom[8]  = 8'hAC;
        // BCD readable version - 0xYYYY_MM_DD_NN
        rom[9]  = 8'h20;
        rom[10] = 8'h26;
        rom[11] = 8'h05;
        rom[12] = 8'h15;
        rom[13] = 8'h01;
    end

    always @(posedge clk) begin
        if (!en) begin
            index <= 0;
        end else if (valid_index) begin
            index <= index + 1;
        end
    end
endmodule
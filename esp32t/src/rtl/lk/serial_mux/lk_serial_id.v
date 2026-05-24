module lk_serial_id_t(
    input clk,
    input enable,
    output reg complete,
    output reg tx_valid,
    output reg [7:0] tx_data
);

localparam ROM_LEN = 22;
localparam ROM_ADDR_WIDTH = $clog2(ROM_LEN);
reg [7:0] rom[0:ROM_LEN - 1];

initial begin
    rom[0] = 8'h00;
    rom[1] = "C";
    rom[2] = "h";
    rom[3] = "r";
    rom[4] = "o";
    rom[5] = "m";
    rom[6] = "a";
    rom[7] = "t";
    rom[8] = "i";
    rom[9] = "c";
    rom[10] = " ";
    rom[11] = "C";
    rom[12] = "a";
    rom[13] = "r";
    rom[14] = "t";
    rom[15] = " ";
    rom[16] = "F";
    rom[17] = "W";
    rom[18] = " ";
    rom[19] = "L";
    rom[20] = "\r";
    rom[21] = 8'h00;
end

reg [ROM_ADDR_WIDTH-1:0] idx;

always @(posedge clk) begin
    if (!enable) begin
        idx <= 0;
    end else if (idx < ROM_LEN) begin
        idx <= idx + 1;
    end
end

always @(posedge clk) complete <= (enable && (idx == (ROM_LEN - 1)));
always @(posedge clk) tx_valid <= idx < ROM_LEN;

always @(posedge clk) begin
    tx_data <= 8'd0;
    if (idx < ROM_LEN) tx_data <= rom[idx];
end

endmodule
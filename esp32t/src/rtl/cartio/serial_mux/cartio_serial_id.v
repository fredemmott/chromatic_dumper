module cartio_serial_id_t(
    input clk,
    input enable,
    output reg complete,
    output reg tx_valid,
    output reg [7:0] tx_data
);

localparam ROM_BLOB = {
    // Size of this response block, overwritten in initial block
    // This is intended to allow chaining with other modifications or IDs
    16'd0,
    // Identify this block
    "fredemmott/CartIO", 8'h00,

    // Our version timestamp - BCD
    /*  YYYY_MM_DD */
    32'h2026_09_19,

    // If we do multiple builds on the same day... __NOT__ BCD!
    8'd01, // Revision

    // Upstream (ModRetro) version number - __NOT__ BCD
    8'd18, 8'd08,

    // USB interface number for cartridge IO
    8'd06
};
localparam ROM_LEN = $bits(ROM_BLOB) / 8;
localparam ROM_ADDR_WIDTH = $clog2(ROM_LEN);
reg [7:0] rom[0:ROM_LEN - 1];

integer i;
initial begin
    for (i = 0; i < ROM_LEN; i = i + 1) begin
        rom[i] = ROM_BLOB[(ROM_LEN - 1 - i)*8 +: 8];
    end
    rom[0] = ROM_LEN >> 8;
    rom[1] = ROM_LEN & 8'hFF;
end

reg [ROM_ADDR_WIDTH-1:0] idx;

always @(posedge clk) begin
    tx_valid <= 1'b0;
    if (!enable) begin
        idx <= 0;
        complete <= 1'b0;
    end else if (idx < ROM_LEN) begin
        idx <= idx + 1;
        tx_valid <= 1'b1;
        tx_data <= rom[idx];
    end else begin
        complete <= 1'b1;
    end
end

endmodule
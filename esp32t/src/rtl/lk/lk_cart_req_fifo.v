import lk_types::*;

module lk_cart_req_fifo_t(
    input wire clk,
    input wire reset,

    output reg empty,

    input wire enqueue,
    input wire dequeue,

    input cart_req_t in,
    output cart_req_t out
);

// Up to 256 elements is genuinelly handy in that
// almost any non-blob operation just doesn't need
// to worry about having its' own buffer, but
// we have a BRAM slot, we're going to use the whole
// BRAM slot, so 512 :)
cart_req_t storage[0:511];
reg [8:0] read_idx;
reg [8:0] read_idx_next;
reg [8:0] write_idx;
reg [8:0] write_idx_next;

assign empty = (read_idx == write_idx);

always @(*) begin
    read_idx_next = read_idx;
    write_idx_next = write_idx;
    if (enqueue) write_idx_next = write_idx + 1'd1;
    if (dequeue) read_idx_next = read_idx + 1'd1;
 end


always @(posedge clk) begin
    if (reset) begin
        read_idx <= 1'd0;
        write_idx <= 1'd0;
    end else begin
        read_idx <= read_idx_next;
        write_idx <= write_idx_next;
    end
end

always @(posedge clk) if (enqueue) storage[write_idx] <= in;

cart_req_t mem_read;
always @(posedge clk) mem_read <= storage[read_idx_next];

cart_req_t previous_in;
always @(posedge clk) previous_in <= in;

reg read_after_write;
always @(posedge clk) read_after_write <= enqueue && empty;

always @(*) out = read_after_write ? previous_in : mem_read;

endmodule
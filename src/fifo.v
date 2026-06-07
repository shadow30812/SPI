/*
 * Generic synchronous Tx circular buffer FIFO template
 */

module fifo #(
    parameter DATA_WIDTH = 8,  // Length of individual data entries 
    parameter ADDR_WIDTH = 3   // FIFO Depth (Number of data entries)
) (
    // Control Signals
    input wire clk,
    input wire rst_n,

    // Write interface (Host -> FIFO)
    input wire wr_en,
    input wire [DATA_WIDTH-1:0] wr_data,
    output wire full,

    // Read interface (FIFO -> Master)
    input wire rd_en,
    output reg [DATA_WIDTH-1:0] rd_data,
    output wire empty
);
  localparam DEPTH = 1 << ADDR_WIDTH;
  reg [DATA_WIDTH-1:0] mem[0:DEPTH-1];

  // Extra bit for wrap-around and empty/full easy detection
  reg [ADDR_WIDTH:0] wr_ptr;
  reg [ADDR_WIDTH:0] rd_ptr;

  // Empty if both pointers are completely equal
  // Full if both addresses are equal but MSB isn't
  assign empty = (rd_ptr == wr_ptr);
  assign full  = (rd_ptr[ADDR_WIDTH] != wr_ptr[ADDR_WIDTH]) && 
                 (rd_ptr[ADDR_WIDTH-1:0] == wr_ptr[ADDR_WIDTH-1:0]);

  // Sequential Write Logic
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) wr_ptr <= 0;
    else if (wr_en && !full) begin
      mem[wr_ptr[ADDR_WIDTH-1:0]] <= wr_data;
      wr_ptr <= wr_ptr + 1;
    end
  end

  // Sequential Read Logic
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rd_ptr  <= 0;
      rd_data <= {DATA_WIDTH{1'b0}};
    end else if (rd_en && !empty) begin
      rd_data <= mem[rd_ptr[ADDR_WIDTH-1:0]];
      rd_ptr  <= rd_ptr + 1;
    end
  end

endmodule

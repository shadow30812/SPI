module top #(
    // Parameters from other modules
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 3,
    parameter CLK_DIV    = 4
) (
    // Control Signals
    input wire clk,
    input wire rst_n,

    // Host Write Interface (to FIFO)
    input wire wr_en,
    input wire [DATA_WIDTH-1:0] wr_data,
    output wire full,
    output wire empty,

    // SPI Mode Control
    input wire cpol,
    input wire cpha,

    // SPI Physical Bus
    input  wire miso,
    output wire sclk,
    output wire mosi,
    output wire cs_n,

    // Host Read Interface (from SPI)
    output wire [DATA_WIDTH-1:0] rx_data,
    output wire rx_valid,
    output wire busy
);
  // Internal interconnect signals
  wire fifo_empty;
  wire [DATA_WIDTH-1:0] tx_data_int;
  wire core_busy;
  reg fifo_rd_en;
  reg core_start;

  assign empty = fifo_empty;  // Empty if fifo is empty
  assign busy  = core_busy | fifo_rd_en | core_start;  // Busy if either stage is active

  // Pop from FIFO iff
  wire pop_ok = !fifo_empty && !core_busy && !core_start && !fifo_rd_en;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      fifo_rd_en <= 1'b0;
      core_start <= 1'b0;
    end else begin
      fifo_rd_en <= pop_ok;
      core_start <= fifo_rd_en;  // Delay start by 1 cycle to account for 
                                 // inherent FIFO read latency
    end
  end

  // Module Instantiations
  fifo #(
      .DATA_WIDTH(DATA_WIDTH),
      .ADDR_WIDTH(ADDR_WIDTH)
  ) tx_buffer (
      .clk    (clk),
      .rst_n  (rst_n),
      .wr_en  (wr_en),
      .wr_data(wr_data),
      .rd_en  (fifo_rd_en),
      .full   (full),
      .empty  (fifo_empty),
      .rd_data(tx_data_int)
  );

  core #(
      .DATA_WIDTH(DATA_WIDTH),
      .CLK_DIV   (CLK_DIV)
  ) spi_engine (
      .clk     (clk),
      .rst_n   (rst_n),
      .tx_data (tx_data_int),
      .start   (core_start),
      .cpol    (cpol),
      .cpha    (cpha),
      .miso    (miso),
      .sclk    (sclk),
      .mosi    (mosi),
      .cs_n    (cs_n),
      .rx_data (rx_data),
      .rx_valid(rx_valid),
      .busy    (core_busy)
  );

endmodule

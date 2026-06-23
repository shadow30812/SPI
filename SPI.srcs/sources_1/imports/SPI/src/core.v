/*
 * Dynamic SPI Master controller
 * (*_n) represents active low signals
 */

module core #(
    parameter DATA_WIDTH = 8,
    parameter CLK_DIV    = 4   // Must be an even number
) (
    // Control Signals
    input wire clk,
    input wire rst_n,

    // Control Interface
    input wire [DATA_WIDTH-1:0] tx_data,
    input wire start,
    input wire cpol,
    input wire cpha,

    // Status Interface
    output reg [DATA_WIDTH-1:0] rx_data,
    output reg busy,
    output reg rx_valid,

    // SPI Interface
    input  wire miso,
    output reg  mosi,
    output reg  sclk,
    output reg  cs_n
);
  // State encodings
  localparam IDLE = 2'b00;
  localparam ACTIVE = 2'b01;
  localparam DONE = 2'b10;

  reg [1:0] state;

  reg [DATA_WIDTH-1:0] tx_shift;
  reg [DATA_WIDTH-1:0] rx_shift;

  reg [log2(CLK_DIV)-1:0] clk_cnt;  // Counts every clock cycle once
  reg [log2(DATA_WIDTH):0] edge_cnt;  // Counts every clock edge once (2x per clock cycle)
                                      // edge_cnt is 0-indexed, 1st edge = 0th index 

  function integer log2;
    input integer value;
    integer i;
    begin
      value = value - 1;
      for (i = 0; value > 0; i = i + 1) value = value >> 1;
      log2 = i;
    end
  endfunction

  // Clock divider pulse (trigger for SCLK, every half SPI Clock cycle)
  wire clk_tick = (clk_cnt == CLK_DIV / 2 - 1);

  wire sample_edge = (cpha == 1'b0) ? (~edge_cnt[0]) : edge_cnt[0];
  wire shift_edge = (cpha == 1'b0) ? edge_cnt[0] : (~edge_cnt[0]);
  // edge_cnt changes at the end of cycle when detected, so checked using old value

  wire last_edge = (edge_cnt == (2 * DATA_WIDTH - 1));

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state    <= IDLE;
      clk_cnt  <= 0;
      edge_cnt <= 0;
      sclk     <= 1'b0;
      mosi     <= 1'b0;
      cs_n     <= 1'b1;
      busy     <= 1'b0;
      rx_valid <= 1'b0;
      rx_data  <= {DATA_WIDTH{1'b0}};
      rx_shift <= {DATA_WIDTH{1'b0}};
      tx_shift <= {DATA_WIDTH{1'b0}};

    end else begin
      rx_valid <= 1'b0;
      case (state)

        IDLE: begin
          sclk     <= cpol;  // CPOL defines the idle state directly
          cs_n     <= 1'b1;
          busy     <= 1'b0;
          clk_cnt  <= 0;
          edge_cnt <= 0;

          // Transition to ACTIVE if start is activated
          if (start) begin
            state    <= ACTIVE;
            busy     <= 1'b1;
            cs_n     <= 1'b0;
            tx_shift <= tx_data;

            // CPHA = 0 requires valid data on MOSI before the first clock edge
            // The first bit is pre-shifted onto the bus during the transition
            if (cpha == 1'b0) begin
              mosi     <= tx_data[DATA_WIDTH-1];
              tx_shift <= {tx_data[DATA_WIDTH-2:0], 1'b0};
            end
          end
        end

        ACTIVE: begin
          // Only act on clock ticks
          if (clk_tick) begin
            sclk     <= ~sclk;
            clk_cnt  <= 0;
            edge_cnt <= edge_cnt + 1;

            // Sample from MISO
            if (sample_edge) rx_shift <= {rx_shift[DATA_WIDTH-2:0], miso};

            // Shift to MOSI
            if (shift_edge) begin
              mosi     <= tx_shift[DATA_WIDTH-1];
              tx_shift <= {tx_shift[DATA_WIDTH-2:0], 1'b0};
            end

            // Transition to DONE when last edge is reached
            if (last_edge) state <= DONE;

          end else clk_cnt <= clk_cnt + 1;
        end

        DONE: begin
          // Wait one half-clock cycle before resetting cs_n
          // to ensure hold time is sufficient for the final bit
          if (clk_tick) begin
            state    <= IDLE;
            sclk     <= cpol;
            cs_n     <= 1'b1;
            rx_valid <= 1'b1;
            rx_data  <= rx_shift;
            busy     <= 1'b0;
          end else clk_cnt <= clk_cnt + 1;
        end

        default: state <= IDLE;  // Safe Signal
      endcase
    end
  end

endmodule

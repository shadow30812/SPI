`timescale 1ns / 1ps


module test_spi ();
  localparam DATA_WIDTH = 8;
  localparam ADDR_WIDTH = 3;
  localparam CLK_DIV = 4;

  reg clk = 0;
  reg rst_n;

  reg wr_en;
  reg [DATA_WIDTH-1:0] wr_data;
  wire full;
  wire empty;

  reg cpol;
  reg cpha;
  wire miso;
  wire sclk;
  wire mosi;
  wire cs_n;

  wire [DATA_WIDTH-1:0] rx_data;
  wire rx_valid;
  wire busy;

  // Top-level module instantiation
  top #(
      .DATA_WIDTH(DATA_WIDTH),
      .ADDR_WIDTH(ADDR_WIDTH),
      .CLK_DIV   (CLK_DIV)
  ) uut (
      .clk     (clk),
      .rst_n   (rst_n),
      .wr_en   (wr_en),
      .wr_data (wr_data),
      .full    (full),
      .empty   (empty),
      .cpol    (cpol),
      .cpha    (cpha),
      .miso    (miso),
      .sclk    (sclk),
      .mosi    (mosi),
      .cs_n    (cs_n),
      .rx_data (rx_data),
      .rx_valid(rx_valid),
      .busy    (busy)
  );

  assign #1 miso = mosi;  // Force the master to be its own slave

  initial forever #5 clk = ~clk;  // Clock generation (100 MHz)

  // Dumping for waveform generation and failsafe for timeout
  initial begin
    $dumpfile("dump_spi.vcd");
    $dumpvars(0, test_spi);

    #50000;
    $display("[FATAL] Simulation Timeout. Check for infinite loops or stalled FSMs.");
    $finish;
  end

  reg [DATA_WIDTH-1:0] expected_queue[0:255];  // FIFO queue to store expected results
  integer push_idx = 0;
  integer pop_idx = 0;
  integer errors = 0;

  // Push data into hardware FIFO and software queue
  task write_byte;
    input [DATA_WIDTH-1:0] data;
    begin
      @(posedge clk)
      if (!full) begin
        wr_en <= 1'b1;
        wr_data <= data;
        expected_queue[push_idx] <= data;
        push_idx <= push_idx + 1;

        @(posedge clk) wr_en <= 1'b0;
      end else $display("[WARNING] Dropped byte %h - FIFO Full", data);
    end
  endtask

  // Monitor Block
  always @(posedge clk) begin
    if (rx_valid) begin
      if (rx_data !== expected_queue[pop_idx]) begin

        $display("[FAIL] Mode %b%b | Expected: %h, Got: %h", cpol, cpha, expected_queue[pop_idx],
                 rx_data);
        errors = errors + 1;

      end else $display("[PASS] Mode %b%b | Successfully looped back: %h", cpol, cpha, rx_data);
      pop_idx = pop_idx + 1;
    end
  end

  initial begin
    // Initialize system before testing
    rst_n   = 0;
    wr_en   = 0;
    wr_data = 0;
    cpol    = 0;
    cpha    = 0;

    $display("   Starting SPI Master Verification Suite...");

    #20 rst_n = 1;
    #20;

    // Test Mode 0 (CPOL=0, CPHA=0)
    cpol = 0;
    cpha = 0;
    write_byte(8'hA5);
    write_byte(8'h3C);
    while (!(empty && !busy)) @(posedge clk);  // Wait for transaction pipeline to flush
    #100;

    // Test Mode 1 (CPOL=0, CPHA=1)
    cpol = 0;
    cpha = 1;
    write_byte(8'hFF);
    write_byte(8'h00);
    while (!(empty && !busy)) @(posedge clk);
    #100;

    // Test Mode 2 (CPOL=1, CPHA=0)
    cpol = 1;
    cpha = 0;
    write_byte(8'h5A);
    write_byte(8'hC3);
    while (!(empty && !busy)) @(posedge clk);
    #100;

    // Test Mode 3 (CPOL=1, CPHA=1)
    cpol = 1;
    cpha = 1;
    write_byte(8'hAA);
    write_byte(8'h55);
    while (!(empty && !busy)) @(posedge clk);
    #100;

    // Test Burst / FIFO Full capability
    $display("--- Testing Burst/FIFO Integrity ---");
    cpol = 0;
    cpha = 0;
    write_byte(8'h11);
    write_byte(8'h22);
    write_byte(8'h33);
    write_byte(8'h44);
    write_byte(8'h55);
    write_byte(8'h66);
    write_byte(8'h77);
    write_byte(8'h88);
    while (!(empty && !busy)) @(posedge clk);
    #100;

    if (errors == 0) $display("   [SUCCESS] ALL TESTS PASSED! 0 ERRORS.");
    else $display("   [FAILURE] TEST SUITE FAILED WITH %0d ERRORS.", errors);

    $finish;
  end

endmodule

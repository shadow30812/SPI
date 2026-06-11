# **High-Performance Parametric SPI Master with Pipelined Circular FIFO Buffer**

This repository contains a silicon-ready, parameterizable Serial Peripheral Interface (SPI) Master Controller implemented in synthesizable Verilog (IEEE 1364-2001). The architecture comprises a dynamic SPI protocol core, an optimized synchronous circular FIFO buffer, and an integrated top-level wrapper managing deterministic interlock handshakes.

Designed with stringent digital design practices, this controller addresses critical physical-layer realities such as dynamic protocol switching, hold-time margins, registered storage path-delay isolation, and hazard-free clock division.

## **Technical Highlights and Interview-Grade Features**

While standard SPI controllers often utilize hardcoded timing parameters and combinational datapath designs, this implementation introduces several structural improvements targeted at high-frequency robustness, low logic footprint, and run-time flexibility:

1. **Dynamic On-The-Fly Protocol Reconfiguration**  
   Unlike standard SPI controllers that parameterize CPOL (Clock Polarity) and CPHA (Clock Phase) at compile time, this design exposes them as dynamic control ports. A host processor can change SPI modes on a transaction-by-transaction basis without requiring FPGA reconfiguration or register map rewrites.  
2. **One-Cycle Latency-Compensated Pipeline Interlock**  
   Synchronous FIFOs with registered read outputs introduce a single-cycle read latency. This design implements a deterministic state handshake that inserts a calculated 1-cycle pre-fetch delay between the FIFO read enable and the SPI engine start trigger, eliminating structural hazards and race conditions during high-speed burst transactions.  
3. **Optimized Pointer-Based Wrap-Around Circular FIFO**  
   Instead of using area-intensive counter registers to track FIFO occupancy, this design implements optimized pointer arithmetic using an extra MSB. This approach resolves the Empty and Full boundaries purely via combinational bit-wise comparisons, optimizing the design's critical path and maximizing the maximum operating frequency ($F_{max}$).  
4. **Dedicated Hold-Time Protection Margin**  
   A frequent cause of data corruption in standard SPI controllers is the premature de-assertion of the Chip Select (cs\_n) line before the slave device has fully latched the final transmitted bit (LSB). This design incorporates a dedicated DONE state that enforces a strict half-SPI-clock-cycle hold delay, ensuring safe interface timings before idle restoration.  
5. **Resource-Efficient Compile-Time Parameter Optimization**  
   The architecture utilizes a local recursive constant function to compute the exact bit-width required for counters (log2 function). This prevents synthesis tools from over-allocating flip-flops for state and divisor counters, maintaining a minimal hardware-resource footprint.

## **Architecture and Data Flow**

The system partition cleanly isolates host interface protocols from the serial physical-line interface.

```txt
                    +---------------------------------------+                  
                    |                  top                  |                  
                    |                                       |                  
+----------+        |  +----------+           +----------+  |       +---------+
|          | Write  |  |          | Read Data |          |  | SCLK  |         |
|          |------->|  |          |---------->|          |  |------>|         |
| Host/CPU |        |  |   fifo   |           |   core   |  | MOSI  |         |
|          | Status |  |          | Pop/Start |          |  |------>|   SPI   |
|          |<-------|  |          |<----------|          |  | MISO  |  Slave  |
+----------+        |  +----------+           +----------+  |<------|         |
                    |                                       | CS_N  |         |
                    |                                       |------>|         |
                    +---------------------------------------+       +---------+
```

### **Signal Map**

| Port Name | Direction | Width | Domain | Description |
| :---- | :---- | :---- | :---- | :---- |
| clk | Input | 1 | System | Primary system high-speed clock |
| rst\_n | Input | 1 | System | Asynchronous active-low reset |
| wr\_en | Input | 1 | System | FIFO write enable (pushes wr\_data into transmission buffer) |
| wr\_data | Input | DATA\_WIDTH | System | Data payload write bus |
| full | Output | 1 | System | FIFO full status flag (asserts backpressure to host) |
| empty | Output | 1 | System | FIFO empty status flag |
| cpol | Input | 1 | System | Run-time SPI clock polarity control (0: idle low, 1: idle high) |
| cpha | Input | 1 | System | Run-time SPI clock phase control (0: sample on 1st edge, 1: sample on 2nd) |
| miso | Input | 1 | SPI | Master-In Slave-Out physical line |
| sclk | Output | 1 | SPI | Generated SPI clock physical line |
| mosi | Output | 1 | SPI | Master-Out Slave-In physical line |
| cs\_n | Output | 1 | SPI | Active-low Chip Select line |
| rx\_data | Output | DATA\_WIDTH | System | Received byte data bus from SPI transaction |
| rx\_valid | Output | 1 | System | Pulse asserting that rx\_data is valid and ready for consumption |
| busy | Output | 1 | System | Asserted when FIFO contains data or core transaction is in progress |

## **Deep Dive into Implementation Details**

### **1\. Pipelined Top-Level Controller (top.v)**

The top-level module functions as a zero-overhead structural scheduler. The primary design challenge is synchronizing the synchronous FIFO read latency with the SPI master core execution.

Standard circular FIFOs require one clock cycle from the assertion of rd\_en until the memory array outputs valid data on the read data bus (rd\_data). To solve this without adding combinational delays to critical timing paths, a sequence controller is implemented using a 2-stage shift enable pipeline:

```verilog
wire pop_ok = !fifo_empty && !core_busy && !core_start && !fifo_rd_en;

always @(posedge clk or negedge rst_n) begin  
  if (!rst_n) begin  
    fifo_rd_en <= 1'b0;  
    core_start <= 1'b0;  
  end else begin  
    fifo_rd_en <= pop_ok;  
    core_start <= fifo_rd_en; // Exact 1-cycle pipeline delay matching the RAM latency  
  end  
end
```

* **Safety Guarantees:** The condition pop\_ok evaluates the status of both modules. It restricts a FIFO read unless the SPI core is ready, the pipeline is clear, and no active reads are in flight.  
* **Aggressive Busy Reporting:** The top-level busy flag is a combination of active signals:  
  $$
  busy = core\_busy \lor fifo\_rd\_en \lor core\_start
  $$  
  This ensures that the host interface recognizes the system as busy the precise cycle a read starts, preventing double-reads or write collisions.

### **2\. High-Fidelity SPI Engine (core.v)**

The core driver implements a robust finite state machine (FSM) spanning three states: IDLE, ACTIVE, and DONE.

#### **Cycle-Accurate Phase Coordination**

To support dynamic phase configurations, the engine does not use distinct state machines for each mode. Instead, it computes logical sampling and shifting triggers based on the phase configuration input (cpha) and a running clock edge index tracker (edge\_cnt):

```verilog
wire sample_edge = (cpha == 1'b0) ? (~edge_cnt[0]) : edge_cnt[0];  
wire shift_edge  = (cpha == 1'b0) ? edge_cnt[0]  : (~edge_cnt[0]);
```

* **Edge Count Mechanics:** edge\_cnt increments at every SPI SCLK half-period transition (triggered by clk\_tick).  
* Because edge\_cnt is 0-indexed, edge\_cnt\[0\] directly corresponds to whether the master is on an even or odd clock transition.  
* This mapping aligns sample and shift triggers across all four modes (Mode 0, 1, 2, and 3), preserving duty cycle symmetry on the physical line.

#### **Setup-Time Preservation on First-Bit (Mode 0 & Mode 2\)**

For CPHA \= 0 configurations, the receiver samples data on the very first physical SCLK transition. This requires the transmitter to place the MSB onto the mosi line *before* the first clock edge. The design achieves this by pre-shifting during the transition from the IDLE state:

```verilog
if (cpha == 1'b0) begin  
  mosi     <= tx_data[DATA_WIDTH-1];  
  tx_shift <= {tx_data[DATA_WIDTH-2:0], 1'b0};  
end
```

This guarantees a setup time equal to a full half-period of the SCLK before the first sampling edge.

#### **Hold-Time Protection Margin on Chip-Select**

De-asserting the Chip Select line (cs\_n) immediately after the final SCLK transition can cause transmission errors in slaves that require data hold time relative to the final clock edge. The engine implements a deterministic guard-band inside the DONE state:

```verilog
DONE: begin  
  if (clk_tick) begin  
    state    <= IDLE;  
    sclk     <= cpol;  
    cs_n     <= 1'b1;  
    rx_valid <= 1'b1;  
    rx_data  <= rx_shift;  
    busy     <= 1'b0;  
  end else clk_cnt <= clk_cnt + 1;  
end
```

By waiting for clk\_tick (a full half-period of SCLK) before transitioning to IDLE and raising cs\_n, the physical interface guarantees a timing safety margin that prevents data corruption on the receiving end.

### **3\. Registered Circular FIFO Buffer (fifo.v)**

Standard FIFOs often use an occupancy counter register that increments on writes and decrements on reads. This structure introduces unnecessary adder logic and increases critical path delays.

This design implements a highly optimized double-pointer wrap-around architecture. By allocating pointers with a bit-width of ADDR\_WIDTH \+ 1 (where the depth is $2^{ADDR\_WIDTH}$), the MSB acts as a virtual quadrant wrap-around bit:

```verilog
assign empty = (rd_ptr == wr_ptr);  
assign full  = (rd_ptr[ADDR_WIDTH] != wr_ptr[ADDR_WIDTH]) &&  
               (rd_ptr[ADDR_WIDTH-1:0] == wr_ptr[ADDR_WIDTH-1:0]);
```

* **No-Counter Area Saving:** The status indicators are evaluated combinationally from the pointers, removing the need for a dedicated counter register.  
* **Registered Read Output:** Storage access registers the output data port (rd\_data) on the rising edge of clk, decoupling the memory array from subsequent logical operators and preventing combinational routing delays from degrading timing margins.

## **Simulation and Verification**

The system includes a self-checking verification environment in test\_spi.v. It configures the SPI Master in an internal loopback configuration (assign miso \= mosi) to thoroughly verify the datapath across all operation modes.

### **Verification Features:**

* **All-Mode Coverage:** Evaluates Mode 0 (00), Mode 1 (01), Mode 2 (10), and Mode 3 (11) sequentially.  
* **Self-Checking Architecture:** Uses a software tracking queue (expected\_queue) to store pushed values. A monitor process verifies incoming transactions on rx\_valid pulses and flags any byte mismatches, tracking system error metrics.  
* **Stress Burst/FIFO Full Testing:** Drives continuous writes to push the FIFO to its upper limit, validating both the backpressure mechanism (full) and transaction sequencing under high load.  
* **Safety Failsafe:** Implements a simulation timeout watchdog to prevent hang-ups from infinite loops or stalled FSMs in the development environment.

### **Simulation Commands**

To compile and simulate the codebase using the standard Icarus Verilog (iverilog) toolchain, execute the following commands in your terminal:

\# Compile the top module, core driver, FIFO, and testbench  

```bash
iverilog \-o sim.out src/top.v src/core.v src/fifo.v test\_spi.v
```

\# Execute the compiled simulation binary

```bash  
vvp sim.out
```

\# Launch GTKWave to analyze the physical line transitions

```bash
gtkwave dump\_spi.vcd
```

## **Dynamic Verification Analysis**

Below is an analysis of a standard verification cycle extracted from simulation logs:

Starting SPI Master Verification Suite...  
\[PASS\] Mode 00 | Successfully looped back: a5  
\[PASS\] Mode 00 | Successfully looped back: 3c  
\[PASS\] Mode 01 | Successfully looped back: ff  
\[PASS\] Mode 01 | Successfully looped back: 00  
\[PASS\] Mode 10 | Successfully looped back: 5a  
\[PASS\] Mode 10 | Successfully looped back: c3  
\[PASS\] Mode 11 | Successfully looped back: aa  
\[PASS\] Mode 11 | Successfully looped back: 55  
\--- Testing Burst/FIFO Integrity \---  
\[PASS\] Mode 00 | Successfully looped back: 11  
\[PASS\] Mode 00 | Successfully looped back: 22  
\[PASS\] Mode 00 | Successfully looped back: 33  
\[PASS\] Mode 00 | Successfully looped back: 44  
\[PASS\] Mode 00 | Successfully looped back: 55  
\[PASS\] Mode 00 | Successfully looped back: 66  
\[PASS\] Mode 00 | Successfully looped back: 77  
\[PASS\] Mode 00 | Successfully looped back: 88  
   \[SUCCESS\] ALL TESTS PASSED\! 0 ERRORS.

### **Waveform Integrity Insights (Reference: graph\_spi.pdf)**

* During high-performance burst operations, the system clock (clk) runs at a higher frequency than the SPI Clock (sclk).  
* With CLK\_DIV set to 4, the internal clock tick generator pulses every 2 clk cycles.  
* The FIFO push\_idx increments sequentially up to 16, verifying correct boundary wrap-around.  
* The error metric remains at 0 across all mode switches, confirming correct physical-layer timing synchronization across all configurations.

## **Design Parameters and Portability**

The modules are written in standard-compliant Verilog-2001, making them highly portable across major synthesis and implementation tools (such as AMD Vivado, Intel Quartus, and Yosys).

### **Module Instantiation Template**

To integrate this controller into a system design, use the following instantiation template:

```verilog
top #(  
    .DATA_WIDTH(8), // Configurable data width (e.g., 8-bit, 16-bit, 32-bit)  
    .ADDR_WIDTH(3), // FIFO Address depth parameter (2^3 = 8-deep buffer)  
    .CLK_DIV   (4)  // Master Clock Divisor parameter (must be an even integer)  
) u_spi_master (  
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
```

// Example for using the NESTang SDRAM controller on Tang Nano 20K

`timescale 1ps /1ps

module sdram_top
  (
    input sys_clk,
    input s1,

    // "Magic" port names that the gowin compiler connects to the on-chip SDRAM
    output O_sdram_clk,
    output O_sdram_cke,
    output O_sdram_cs_n,            // chip select
    output O_sdram_cas_n,           // columns address select
    output O_sdram_ras_n,           // row address select
    output O_sdram_wen_n,           // write enable
    inout [31:0] IO_sdram_dq,       // 32 bit bidirectional data bus
    output [10:0] O_sdram_addr,     // 11 bit multiplexed address bus
    output [1:0] O_sdram_ba,        // two banks
    output [3:0] O_sdram_dqm,       // 32/4

    output [5:0] led,

    output uart_txp
  );

reg sys_resetn = 0;
reg start;      // press s1 to start the system
always @(posedge clk) begin
    if (s1) begin
        start <= 1;
        sys_resetn <= 1;
    end
end

reg rd, wr, refresh;
reg [22:0] addr;
reg [7:0] din;
wire [7:0] dout;

localparam FREQ=27_000_000;

Gowin_rPLL pll(
    .clkout(clk),           // Main clock
    .clkoutp(clk_sdram),    // Phase shifted clock for SDRAM
    .clkin(sys_clk)         // 27Mhz system clock
);

sdram #(.FREQ(FREQ)) u_sdram (
    .clk(clk), .clk_sdram(clk_sdram), .resetn(sys_resetn && start),
	.addr(addr), .rd(rd), .wr(wr), .refresh(refresh),
	.din(din), .dout(dout), .data_ready(data_ready), .busy(busy),

    .SDRAM_DQ(IO_sdram_dq),     // 32 bit bidirectional data bus
    .SDRAM_A(O_sdram_addr),     // 11 bit multiplexed address bus
    .SDRAM_BA(O_sdram_ba),      // 4 banks
    .SDRAM_nCS(O_sdram_cs_n),   // a single chip select
    .SDRAM_nWE(O_sdram_wen_n),  // write enable
    .SDRAM_nRAS(O_sdram_ras_n), // row address select
    .SDRAM_nCAS(O_sdram_cas_n), // columns address select
    .SDRAM_CLK(O_sdram_clk),
    .SDRAM_CKE(O_sdram_cke),
    .SDRAM_DQM(O_sdram_dqm)
);

localparam [3:0] INIT = 3'd0;
localparam [3:0] WRITE1 = 3'd1;
localparam [3:0] WRITE2 = 3'd2;
localparam [3:0] READ = 3'd3;
localparam [3:0] READ_RESULT = 3'd4;
localparam [3:0] WRITE_BLOCK = 3'd5;
localparam [3:0] VERIFY_BLOCK = 3'd6;
localparam [3:0] FINISH = 3'd7;

reg error_bit = 1'b0;
localparam V0=8'h3e;
localparam V1=8'b1110_1101;
localparam TOTAL_SIZE = 1 * 1024 * 1024;        // write 1MB of memory

assign led = ~{error_bit, 1'b0, state};

// SDRAM refresh logic:
// - This is one way of generating the refresh pulses. A refresh pulse is generated every 15us.
//   It is generated here (refresh_needed) and executed by the main state machine.
// - NESTang uses another way to generate refresh pulses. It requests a refresh whenver it is
//   supposed to have memory accesses and does not have one to issue.
//   See nes_tang20k.v for details.
localparam REFRESH_COUNT=FREQ/1000/1000*15;     // 15us refresh
reg refresh_needed;
reg refresh_executed;                           // pulse from main FSM
reg [11:0] refresh_time;
always @(posedge clk && state) begin
    refresh_time <= refresh_time == (REFRESH_COUNT*2-2) ? (REFRESH_COUNT*2-2) : refresh_time + 1;
    if (refresh_time == REFRESH_COUNT) 
        refresh_needed <= 1;
    if (refresh_executed) begin
        refresh_time <= refresh_time - REFRESH_COUNT;
        refresh_needed <= 0;
    end
    if (~sys_resetn) begin
        refresh_time <= 0;
        refresh_needed <= 0;
    end
end

// Main state machine
reg [3:0] state, end_state;
reg [7:0] work_counter;                         // 9.5ms per state to give UART time to print a message
reg [7:0] expected, actual;
reg refresh_cycle;
always @(posedge clk) begin
    // all commands default to 0, so they are always pulses
    wr <= 0; rd <= 0; refresh <= 0; refresh_executed <= 0;
    // loops from 0 - 255
    work_counter <= work_counter + 1;

    case (state)
        INIT: if (start && work_counter == 0) begin
            state <= WRITE1;
        end
        WRITE1: if (!busy) begin 
            wr <= 1'b1;
            addr <= 23'b0;
            din <= V0;
            state <= WRITE2;
        end
        WRITE2: begin
            if (!wr && !busy && work_counter == 0) begin
                wr <= 1'b1;
                addr <= 23'h1;
                din <= V1;
                state <= READ;
            end
        end
        READ: begin
            if (!wr && !busy && work_counter == 0) begin
                rd <= 1'b1;
                addr <= 23'h1;
                state <= READ_RESULT;
            end
        end
        READ_RESULT: begin
            if (!rd && !busy && work_counter == 0) begin
                actual <= dout;
                expected <= V1;
                if (dout == V1) begin
                    state <= WRITE_BLOCK;
                    addr <= 0;
                    work_counter <= 0;
                end else begin
                    error_bit <= 1;
                    end_state <= state;
                    state <= FINISH;
                end
            end
        end
        WRITE_BLOCK: begin
            // write some data
            if (addr == TOTAL_SIZE) begin
                state <= VERIFY_BLOCK;
                work_counter <= 0;
                addr <= 0;
            end else begin
                if (work_counter == 0) begin
                    if (!refresh_needed) begin
                        wr <= 1'b1;
                        din <= addr[7:0];
                        refresh_cycle <= 0;
                    end else begin
                        refresh <= 1'b1;
                        refresh_executed <= 1'b1;
                        refresh_cycle <= 1'b1;
                    end
                end else if (!wr && !refresh && !busy) begin
                    work_counter <= 0;
                    if (!refresh_cycle)
                        addr <= addr + 1;
                end
            end
        end
        VERIFY_BLOCK: begin
            if (addr == TOTAL_SIZE) begin
                end_state <= state;
                state <= FINISH;
            end else begin
                if (work_counter == 0) begin
                    // send next read request or refresh
                    if (!refresh_needed) begin
                        rd <= 1'b1;
                        refresh_cycle <= 1'b0;
                    end else begin
                        refresh <= 1'b1;
                        refresh_executed <= 1'b1;
                        refresh_cycle <= 1'b1;
                    end
                end else if (data_ready) begin
                    // verify result
                    expected <= addr[7:0];
                    actual <= dout;
                    if (dout != addr[7:0]) begin
                        error_bit <= 1'b1;
                        end_state <= state;
                        state <= FINISH;
                    end
                end else if (!rd && !refresh && !busy) begin
                    work_counter <= 0;      // start next read
                    if (!refresh_cycle) begin
                        addr <= addr + 1;
                    end
                end
            end
        end
    endcase

    if (~sys_resetn) begin
        error_bit <= 1'b0;
        state <= INIT;
    end
end


// Print controll
`include "print.v"
defparam tx.uart_freq=115200;
defparam tx.clk_freq=FREQ;
assign print_clk = clk;
assign txp = uart_txp;

reg[3:0] state_0;
reg[3:0] state_1;
reg[3:0] state_old;
wire[3:0] state_new = state_1;

always@(posedge clk)begin
    state_1<=state_0;
    state_0<=state;

    if(state_0==state_1) begin //stable value
        state_old<=state_new;

        if(state_old!=state_new)begin//state changes
            if(state_new==INIT)`print("Initializing SDRAM\n",STR);
          
            if(state_new==WRITE1) `print("Writing to address 1\n",STR);
          
            if(state_new==WRITE2)`print("Writing to address 2\n",STR);

            if(state_new==READ)`print("Reading from address 1\n",STR);

            if(state_new==WRITE_BLOCK)`print("Writing a block of data\n",STR);

            if(state_new==VERIFY_BLOCK)`print("Verifying the block\n",STR);

            if(state_new==FINISH)begin
                if(error_bit) begin
                    `print("ERROR Occured.\n\n",STR);
                end else begin
                    `print("SUCCESS: Test Finished\n\n",STR);
                end
            end      
        end
    end
end

endmodule

//============================================================================
//  ZMX Bridge for MiSTer MSX Core
//  Connects MSX Slot 2 / IO Bus to HPS via MiSTer EXT_BUS (SPI)
//============================================================================

`timescale 1ns / 1ps

module zmx_bridge
(
	input             clk_sys,
	input             reset,

	// MiSTer EXT_BUS interface
	inout      [35:0] EXT_BUS,

	// Configuration / Control
	input             enable,          // 1 = ZMX Bridge enabled
	input             slot_sel,        // 0 = Slot 1, 1 = Slot 2

	// MSX Slot bus interface
	input             slt_cs,          // Slot selected (Active High)
	input             slt_merq_n,      // Memory Request (Active Low)
	input             slt_iorq_n,      // IO Request (Active Low)
	input             slt_rd_n,        // Read (Active Low)
	input             slt_wr_n,        // Write (Active Low)
	input      [15:0] slt_addr,        // 16-bit address
	input       [7:0] slt_wdata,       // 8-bit write data
	output reg  [7:0] slt_rdata,       // 8-bit read data
	output reg        slt_rdata_en,    // 1 = read data valid
	output reg        cpu_wait_n       // CPU Wait signal (Active Low)
);

// SPI commands (matching Main_MiSTer support/msx)
localparam CMD_ZMX_POLL = 8'h68;
localparam CMD_ZMX_ADDR = 8'h69;
localparam CMD_ZMX_RESP = 8'h6A;

// Commands matching zmxbus.h
localparam RD_SLTSL1 = 8'h00;
localparam WR_SLTSL1 = 8'h01;
localparam RD_SLTSL2 = 8'h10;
localparam WR_SLTSL2 = 8'h11;
localparam RD_IO     = 8'h02;
localparam WR_IO     = 8'h03;

// EXT_BUS signal breakdown
wire        io_strobe = EXT_BUS[33];
wire        io_enable = EXT_BUS[34];
wire [15:0] io_din    = EXT_BUS[31:16];

reg         ext_active = 0;
reg  [15:0] ext_dout = 16'h0000;

assign EXT_BUS[32]   = ext_active;
assign EXT_BUS[15:0] = ext_active ? ext_dout : 16'hZZZZ;

// Transaction latches
reg        req_pending = 0;
reg  [7:0] req_cmd     = 8'h00;
reg [15:0] req_addr    = 16'h0000;
reg  [7:0] req_wdata   = 8'h00;

// State machine for MSX Slot detection
typedef enum logic [1:0] {
	S_IDLE,
	S_WAIT_HPS,
	S_DONE
} state_t;

state_t state = S_IDLE;

// SPI communication handler registers
reg [7:0] spi_cmd  = 8'h00;
reg [3:0] byte_cnt = 0;
reg [23:0] timeout_cnt = 0;

always @(posedge clk_sys or posedge reset) begin
	if (reset) begin
		state        <= S_IDLE;
		req_pending  <= 0;
		req_cmd      <= 8'h00;
		req_addr     <= 16'h0000;
		req_wdata    <= 8'h00;
		slt_rdata    <= 8'hFF;
		slt_rdata_en <= 0;
		cpu_wait_n   <= 1;
		timeout_cnt  <= 0;

		ext_active   <= 0;
		ext_dout     <= 16'h0000;
		spi_cmd      <= 8'h00;
		byte_cnt     <= 0;
	end else if (!enable) begin
		state        <= S_IDLE;
		req_pending  <= 0;
		slt_rdata_en <= 0;
		cpu_wait_n   <= 1;
		timeout_cnt  <= 0;

		ext_active   <= 0;
		ext_dout     <= 16'h0000;
		spi_cmd      <= 8'h00;
		byte_cnt     <= 0;
	end else begin
		// --- SPI EXT_BUS handling ---
		if (!io_enable) begin
			ext_active <= 0;
			byte_cnt   <= 0;
		end else if (io_strobe) begin
			byte_cnt <= byte_cnt + 1'd1;
			if (byte_cnt == 0) begin
				spi_cmd <= io_din[7:0];
				case (io_din[7:0])
					CMD_ZMX_POLL: begin
						ext_active <= 1;
						// First read word returns [15:8] = req_cmd, [7:0] = {7'b0, req_pending}
						ext_dout   <= {req_cmd, 7'b0, req_pending};
					end
					CMD_ZMX_ADDR: begin
						ext_active <= 1;
						// First read word returns 16-bit address
						ext_dout   <= req_addr;
					end
					CMD_ZMX_RESP: begin
						ext_active <= 0;
					end
					default: begin
						ext_active <= 0;
					end
				endcase
			end else if (byte_cnt == 1) begin
				if (spi_cmd == CMD_ZMX_POLL) begin
					ext_active <= 0;
				end else if (spi_cmd == CMD_ZMX_ADDR) begin
					// Second word returns write data: [7:0] = req_wdata
					ext_dout <= {8'h00, req_wdata};
				end else if (spi_cmd == CMD_ZMX_RESP) begin
					// HPS sends back read data: io_din[7:0]
					slt_rdata   <= io_din[7:0];
					req_pending <= 0; // Handshake: clear pending -> releases CPU WAIT
				end
			end else if (byte_cnt == 2) begin
				ext_active <= 0;
			end
		end

		// --- MSX Slot Bus Cycle State Machine ---
		case (state)
			S_IDLE: begin
				slt_rdata_en <= 0;
				// Slot Memory Read or Write cycle detection
				if (slt_cs && !slt_merq_n) begin
					timeout_cnt <= 0;
					if (!slt_rd_n) begin
						// Memory Read
						req_pending <= 1;
						req_cmd     <= slot_sel ? RD_SLTSL2 : RD_SLTSL1;
						req_addr    <= slt_addr;
						cpu_wait_n  <= 0; // Hold CPU
						state       <= S_WAIT_HPS;
					end else if (!slt_wr_n) begin
						// Memory Write
						req_pending <= 1;
						req_cmd     <= slot_sel ? WR_SLTSL2 : WR_SLTSL1;
						req_addr    <= slt_addr;
						req_wdata   <= slt_wdata;
						cpu_wait_n  <= 0; // Hold CPU
						state       <= S_WAIT_HPS;
					end
				end
			end

			S_WAIT_HPS: begin
				// Handshake complete when HPS responds with CMD_ZMX_RESP (which clears req_pending)
				if (!req_pending) begin
					slt_rdata_en <= (req_cmd == RD_SLTSL1 || req_cmd == RD_SLTSL2 || req_cmd == RD_IO);
					cpu_wait_n   <= 1; // Release CPU WAIT
					state        <= S_DONE;
				end else begin
					timeout_cnt <= timeout_cnt + 1'd1;
					// 21.48MHz clock: 2000 cycles = ~93us timeout (ultra-fast safety timeout)
					if (timeout_cnt >= 24'd2000) begin
						// Safety Timeout: Release CPU and return 0xFF to prevent hang/black screen
						slt_rdata    <= 8'hFF;
						slt_rdata_en <= 1;
						cpu_wait_n   <= 1; // Release CPU WAIT
						req_pending  <= 0;
						state        <= S_DONE;
					end
				end
			end

			S_DONE: begin
				// Wait for bus cycle to complete before returning to IDLE
				if (slt_merq_n && slt_iorq_n) begin
					slt_rdata_en <= 0;
					state        <= S_IDLE;
				end
			end
		endcase
	end
end

endmodule

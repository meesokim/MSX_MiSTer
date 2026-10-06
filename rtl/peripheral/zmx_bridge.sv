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
	input             slt_rfsh_n,      // Refresh (Active Low). High during a real memory cycle
	input             slt_iorq_n,      // IO Request (Active Low)
	input             slt_rd_n,        // Read (Active Low)
	input             slt_wr_n,        // Write (Active Low)
	input             slt_m1_n,        // M1 (Active Low). Low during interrupt acknowledge
	input             slt_busdir,      // 1 = internal MSX I/O device owns this port
	input      [15:0] slt_addr,        // 16-bit address
	input       [7:0] slt_wdata,       // 8-bit write data
	output reg  [7:0] slt_rdata,       // 8-bit read data
	output reg        slt_rdata_en,    // 1 = read data valid
	output reg        slt_rdata_io,    // 1 = read data is an I/O response (not slot memory)
	output            cpu_wait_n       // CPU Wait signal (Active Low)
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
reg [2:0]  hold_cnt = 0;

// T80pa samples WAIT_n on the T2 falling phase, which is the same edge that
// drops WR_n. Waiting only after RD/WR is already low misses that sample
// (writes always, and reads too when the CPU is at 10.74 MHz). A real slot
// memory cycle is MREQ with RFSH high; assert WAIT for that whole cycle.
// External I/O (not an internal device, not interrupt acknowledge) is the
// same kind of cycle and is reported as RD_IO / WR_IO.
wire slot_mem = enable && !reset && slt_cs && !slt_merq_n && (!slt_rd_n || !slt_wr_n);
wire slot_io  = enable && !reset && slt_m1_n && !slt_iorq_n && slt_merq_n && !slt_busdir && (!slt_rd_n || !slt_wr_n);
wire is_read  = (req_cmd == RD_SLTSL1) || (req_cmd == RD_SLTSL2) || (req_cmd == RD_IO);

assign cpu_wait_n = ~(
	(state == S_WAIT_HPS) ||
	(state == S_IDLE && (slot_mem || slot_io))
);

always @(posedge clk_sys or posedge reset) begin
	if (reset) begin
		state        <= S_IDLE;
		req_pending  <= 0;
		req_cmd      <= 8'h00;
		req_addr     <= 16'h0000;
		req_wdata    <= 8'h00;
		slt_rdata    <= 8'hFF;
		slt_rdata_en <= 0;
		slt_rdata_io <= 0;
		timeout_cnt  <= 0;
		hold_cnt     <= 0;

		ext_active   <= 0;
		ext_dout     <= 16'h0000;
		spi_cmd      <= 8'h00;
		byte_cnt     <= 0;
	end else if (!enable) begin
		state        <= S_IDLE;
		req_pending  <= 0;
		slt_rdata_en <= 0;
		slt_rdata_io <= 0;
		timeout_cnt  <= 0;
		hold_cnt     <= 0;

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
						// First read word returns [15:8] = 0xA5 (magic signature), [7:0] = {req_cmd[6:0], req_pending}
						ext_dout   <= {8'hA5, req_cmd[6:0], req_pending};
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
					// Word 1: HPS is reading status word. Keep status data valid on bus!
					ext_active <= 1;
					ext_dout   <= {8'hA5, req_cmd[6:0], req_pending};
				end else if (spi_cmd == CMD_ZMX_ADDR) begin
					// Legacy mode - Word 1: HPS reads req_addr
					ext_active <= 1;
					ext_dout   <= req_addr;
				end else if (spi_cmd == CMD_ZMX_RESP) begin
					// Legacy mode: HPS sends back read data: io_din[7:0]
					slt_rdata   <= io_din[7:0];
					req_pending <= 0; // Handshake: clear pending -> releases CPU WAIT
				end
			end else if (byte_cnt == 2) begin
				if (spi_cmd == CMD_ZMX_POLL) begin
					// Word 2: HPS is reading req_addr! Keep req_addr valid on bus!
					ext_active <= 1;
					ext_dout   <= req_addr;
				end else if (spi_cmd == CMD_ZMX_ADDR) begin
					// Legacy mode - Word 2: HPS reads req_wdata
					ext_active <= 1;
					ext_dout   <= {8'h00, req_wdata};
				end
			end else if (byte_cnt == 3) begin
				if (spi_cmd == CMD_ZMX_POLL) begin
					if (is_read) begin
						// Word 3: Single-session read complete! HPS sent rdata in io_din[7:0]
						slt_rdata   <= io_din[7:0];
						req_pending <= 0; // Releases CPU WAIT immediately!
						ext_active  <= 0;
					end else begin
						// Word 3: HPS is reading req_wdata! Keep wdata valid on bus!
						ext_active <= 1;
						ext_dout   <= {8'h00, req_wdata};
					end
				end
			end else if (byte_cnt == 4) begin
				if (spi_cmd == CMD_ZMX_POLL && !is_read) begin
					// Word 4: Single-session write complete! HPS ack received
					req_pending <= 0; // Releases CPU WAIT!
					ext_active  <= 0;
				end
			end
		end

		// --- MSX Slot Bus Cycle State Machine ---
		case (state)
			S_IDLE: begin
				slt_rdata_en <= 0;
				slt_rdata_io <= 0;
				hold_cnt     <= 0;
				// WAIT is already held combinationally on slot_mem / slot_io.
				// Classify the cycle only once RD or WR is actually low:
				// memory writes drop WR_n on the T2 wait-sample edge, so they
				// show up one clock after MREQ while the CPU is stalled.
				if (slot_mem && !slt_rd_n) begin
					timeout_cnt <= 0;
					req_pending <= 1;
					req_cmd     <= slot_sel ? RD_SLTSL2 : RD_SLTSL1;
					req_addr    <= slt_addr;
					state       <= S_WAIT_HPS;
				end else if (slot_mem && !slt_wr_n) begin
					timeout_cnt <= 0;
					req_pending <= 1;
					req_cmd     <= slot_sel ? WR_SLTSL2 : WR_SLTSL1;
					req_addr    <= slt_addr;
					req_wdata   <= slt_wdata;
					state       <= S_WAIT_HPS;
				end else if (slot_io && !slt_rd_n) begin
					timeout_cnt <= 0;
					req_pending <= 1;
					req_cmd     <= RD_IO;
					req_addr    <= slt_addr;
					state       <= S_WAIT_HPS;
				end else if (slot_io && !slt_wr_n) begin
					timeout_cnt <= 0;
					req_pending <= 1;
					req_cmd     <= WR_IO;
					req_addr    <= slt_addr;
					req_wdata   <= slt_wdata;
					state       <= S_WAIT_HPS;
				end
			end

			S_WAIT_HPS: begin
				// Handshake complete when HPS responds with CMD_ZMX_RESP (which clears req_pending)
				if (!req_pending) begin
					slt_rdata_en <= is_read;
					slt_rdata_io <= (req_cmd == RD_IO);
					hold_cnt     <= 3'd3;
					state        <= S_DONE;
				end else begin
					timeout_cnt <= timeout_cnt + 1'd1;
					// 21.48MHz clock: 100000 cycles = ~4.65ms timeout (gives HPS main loop sufficient time)
					if (timeout_cnt >= 24'd100000) begin
						// Safety Timeout: Release CPU and return 0xFF to prevent hang/black screen
						slt_rdata    <= 8'hFF;
						slt_rdata_en <= is_read;
						slt_rdata_io <= (req_cmd == RD_IO);
						hold_cnt     <= 3'd3;
						req_pending  <= 0;
						state        <= S_DONE;
					end
				end
			end

			S_DONE: begin
				// T80pa drops M1 RD_n one phase before it samples DI.
				// Keep the byte driven for a few 21.48 MHz clocks after RD/WR rise
				// so that sample still sees it, then return to IDLE.
				if (!slt_rd_n || !slt_wr_n) begin
					hold_cnt <= 3'd3;
				end else if (hold_cnt != 0) begin
					hold_cnt <= hold_cnt - 3'd1;
				end else begin
					slt_rdata_en <= 0;
					slt_rdata_io <= 0;
					state        <= S_IDLE;
				end
			end
		endcase
	end
end

endmodule

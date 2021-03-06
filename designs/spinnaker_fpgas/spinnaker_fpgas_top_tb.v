/**
 * A testbench for the SpiNNaker FPGA design.
 */

`timescale 1ns / 1ps

`include "../../modules/spinnaker_link/spio_spinnaker_link.h"

`include "../../modules/hss_multiplexer/spio_hss_multiplexer_common.h"
`include "../../modules/hss_multiplexer/spio_hss_multiplexer_reg_bank.h"

module spinnaker_fpgas_top_tb;

genvar i;

`include "spinnaker_fpgas_top.h"

// Select the FPGA ID and spinnaker module connectivity
localparam FPGA_ID = 2;

// Which HSS links are used
localparam NORTH_SOUTH_ON_FRONT = 1;

// External differential clock
reg  refclk_pad_p_i;
wire refclk_pad_n_i = ~refclk_pad_p_i;

// Clock for testbench devices
reg tb_clk_i;

// Reset signals
reg  reset_i;
wire n_reset_i;
reg  lreset_i;

// LEDs
wire red_led_i;
wire grn_led_i;

// High-speed serial links
wire [3:0] hss_rxn_i;
wire [3:0] hss_rxp_i;
wire [3:0] hss_txn_i;
wire [3:0] hss_txp_i;

// SpiNNaker link pins
wire [15:0] sl_pins_i     [15:0];
wire [6:0]  sl_out_data_i [15:0];
wire        sl_out_ack_i  [15:0];
wire [6:0]  sl_in_data_i  [15:0];
wire        sl_in_ack_i   [15:0];



////////////////////////////////////////////////////////////////////////////////
// External 150 MHz Clock Generator
////////////////////////////////////////////////////////////////////////////////

initial
	begin
		refclk_pad_p_i = 1'b0;
		forever #3.33333333 refclk_pad_p_i = ~refclk_pad_p_i;
	end


////////////////////////////////////////////////////////////////////////////////
// Test-bench clock
////////////////////////////////////////////////////////////////////////////////

initial
	begin
		tb_clk_i = 1'b0;
		forever #4.166666667 tb_clk_i = ~tb_clk_i;
	end


////////////////////////////////////////////////////////////////////////////////
// Reset generation
////////////////////////////////////////////////////////////////////////////////

initial
	begin
		reset_i = 1'b1;
		lreset_i = 1'b1;
		#100
		reset_i = 1'b0;
		#50
		lreset_i = 1'b0;
	end

assign n_reset_i = !reset_i;


////////////////////////////////////////////////////////////////////////////////
// High-speed-serial Wires
////////////////////////////////////////////////////////////////////////////////

generate case ({NORTH_SOUTH_ON_FRONT, FPGA_ID})
	{0, 0},
	{0, 1},
	{0, 2},
	{1, 1}:
		begin
			// Cross-connect the two board-to-board links
			assign hss_rxn_i[0] = hss_txn_i[1];
			assign hss_rxp_i[0] = hss_txp_i[1];
			assign hss_rxn_i[1] = hss_txn_i[0];
			assign hss_rxp_i[1] = hss_txp_i[0];
			
			// Loop-back the peripheral link and ring link
			assign hss_rxn_i[2] = hss_txn_i[2];
			assign hss_rxp_i[2] = hss_txp_i[2];
			assign hss_rxn_i[3] = hss_txn_i[3];
			assign hss_rxp_i[3] = hss_txp_i[3];
		end
	
	{1, 0}:
		begin
			// Cross-connect the two board-to-board links
			assign hss_rxn_i[0] = hss_txn_i[2];
			assign hss_rxp_i[0] = hss_txp_i[2];
			assign hss_rxn_i[2] = hss_txn_i[0];
			assign hss_rxp_i[2] = hss_txp_i[0];
			
			// Loop-back the peripheral link and ring link
			assign hss_rxn_i[1] = hss_txn_i[1];
			assign hss_rxp_i[1] = hss_txp_i[1];
			assign hss_rxn_i[3] = hss_txn_i[3];
			assign hss_rxp_i[3] = hss_txp_i[3];
		end
	
	{1, 2}:
		begin
			// Cross-connect the two board-to-board links
			assign hss_rxn_i[2] = hss_txn_i[1];
			assign hss_rxp_i[2] = hss_txp_i[1];
			assign hss_rxn_i[1] = hss_txn_i[2];
			assign hss_rxp_i[1] = hss_txp_i[2];
			
			// Loop-back the peripheral link and ring link
			assign hss_rxn_i[0] = hss_txn_i[0];
			assign hss_rxp_i[0] = hss_txp_i[0];
			assign hss_rxn_i[3] = hss_txn_i[3];
			assign hss_rxp_i[3] = hss_txp_i[3];
		end
endcase endgenerate

////////////////////////////////////////////////////////////////////////////////
// Device under test
////////////////////////////////////////////////////////////////////////////////

spinnaker_fpgas_top #( // Enable simulation mode for GTP tile
                       .SIMULATION(1)
                     , .SIMULATION_GTPRESET_SPEEDUP(1)
                       // Disable (redundant) chipscope regbank interface during simulation
                     , .DEBUG_CHIPSCOPE_VIO(0)
                       // Simulate the FPGA design for an arbitrary chip
                     , .FPGA_ID(FPGA_ID)
                       // Connect I/O or B2B links arbitrarily
                     , .NORTH_SOUTH_ON_FRONT(NORTH_SOUTH_ON_FRONT)
                       // The interval at which clock correction sequences should
                       // be inserted (in cycles).
                     ,    .B2B_CLOCK_CORRECTION_INTERVAL(1000)
                     , .PERIPH_CLOCK_CORRECTION_INTERVAL(1000)
                     ,   .RING_CLOCK_CORRECTION_INTERVAL(1000)
                       // The number of bits for the above counters.
                     ,    .B2B_CLOCK_CORRECTION_INTERVAL_BITS(10)
                     , .PERIPH_CLOCK_CORRECTION_INTERVAL_BITS(10)
                     ,   .RING_CLOCK_CORRECTION_INTERVAL_BITS(10)
                       // Number of consecutive handshakes which must arrive
                       // before advancing the handshake phase.
                     ,    .B2B_NUM_HANDSHAKES(100)
                     , .PERIPH_NUM_HANDSHAKES(100)
                     ,   .RING_NUM_HANDSHAKES(100)
                       // Number of bits required for the above counters.
                     ,    .B2B_NUM_HANDSHAKES_BITS(7)
                     , .PERIPH_NUM_HANDSHAKES_BITS(7)
                     ,   .RING_NUM_HANDSHAKES_BITS(7)
                     )
spinnaker_fpgas_top_i( // Reset signal
                       .N_RESET_IN(n_reset_i)
                       
                       // Status LEDs
                     , .RED_LED_OUT(red_led_i)
                     , .GRN_LED_OUT(grn_led_i)
                       
                       // Differential 150 MHz clock source for each of
                       // the tiles
                     , .REFCLK_PAD_P_IN(refclk_pad_p_i)
                     , .REFCLK_PAD_N_IN(refclk_pad_n_i)
                     
                       // Wires for all four high speed differential
                       // links from the two GTP tiles
                     , .HSS_RXN_IN(hss_rxn_i)
                     , .HSS_RXP_IN(hss_rxp_i)
                     , .HSS_TXN_OUT(hss_txn_i)
                     , .HSS_TXP_OUT(hss_txp_i)
                       
                       // Wires for the SpiNNaker 2-of-7 links. Since the
                       // three different FPGAs are connected to
                       // different configurations, these pins have their
                       // directions set by the FPGA ID signal.
                     , .SL_INOUT( { sl_pins_i[15], sl_pins_i[14], sl_pins_i[13], sl_pins_i[12]
                                  , sl_pins_i[11], sl_pins_i[10], sl_pins_i[9],  sl_pins_i[8]
                                  , sl_pins_i[7],  sl_pins_i[6],  sl_pins_i[5],  sl_pins_i[4]
                                  , sl_pins_i[3],  sl_pins_i[2],  sl_pins_i[1],  sl_pins_i[0]
                                  }
                                )
                       
                       // Wires for the (not-tested-here) SPI interface
                     , .SPI_NSS_IN(1'b1) // Tie off as not selected
                     , .SPI_SCLK_IN(1'b0)
                     , .SPI_MOSI_IN(1'b0)
                     , .SPI_MISO_OUT()
                     );


////////////////////////////////////////////////////////////////////////////////
// 2-of-7 Link Packet Generators
////////////////////////////////////////////////////////////////////////////////

generate
	for (i = 0; i < 16; i = i + 1)
	begin : spinnaker_link_stimulus_generation
		// Wire up the links depending on the FPGA in use
		if (FPGA_SL_TYPES[(32*FPGA_ID) + (2*i) +: 2] == HIGH_SL)
		begin
			assign sl_out_ack_i[i]    = sl_pins_i[i][0];
			assign sl_pins_i[i][7:1]  = sl_out_data_i[i];
			assign sl_pins_i[i][8]    = sl_in_ack_i[i];
			assign sl_in_data_i[i]    = sl_pins_i[i][15:9];
		end
		else     
		begin
			assign sl_pins_i[i][0]    = sl_in_ack_i[i];
			assign sl_in_data_i[i]    = sl_pins_i[i][7:1];
			assign sl_out_ack_i[i]    = sl_pins_i[i][8];
			assign sl_pins_i[i][15:9] = sl_out_data_i[i];
		end
		
		// Packet generator
		spio_spinnaker_link_packet_gen
		spio_spinnaker_link_packet_gen_i
		(
			.clk        (tb_clk_i),
			.reset      (lreset_i),
			.data_2of7  (sl_out_data_i[i]),
			.ack        (sl_out_ack_i[i])
		);
		
		// Packet consumer
		spio_spinnaker_link_packet_con
		spio_spinnaker_link_packet_con_i
		(
			.clk        (tb_clk_i),
			.reset      (reset_i),
			.fifo_full  (1'b0),
			.fifo_write (),
			.code2of7   (sl_in_data_i[i]),
			.ack        (sl_in_ack_i[i]),
			.packet     ()
		);
	end // block: lktb
endgenerate


endmodule


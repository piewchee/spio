// -------------------------------------------------------------------------
//  SpiNNaker link transmitter module
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
//  Based on work by J Pepper (Date 08/08/2012)
//
// -------------------------------------------------------------------------
// Taken from:
// https://solem.cs.man.ac.uk/svn/spiNNlink/testing/src/packet_sender.v
// Revision 2517 (Last-modified date: 2013-08-19 10:33:30 +0100)
//
// -------------------------------------------------------------------------
// COPYRIGHT
//  Copyright (c) The University of Manchester, 2012. All rights reserved.
//  SpiNNaker Project
//  Advanced Processor Technologies Group
//  School of Computer Science
// -------------------------------------------------------------------------
// TODO
// -------------------------------------------------------------------------


// ----------------------------------------------------------------
// include spiNNlink global constants and parameters
//
`include "spio_spinnaker_link.h"
// ----------------------------------------------------------------


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module spio_spinnaker_link_sender
(
  input                        CLK_IN,
  input                        RESET_IN,

  // synchronous packet interface
  input      [`PKT_BITS - 1:0] PKT_DATA_IN,
  input                        PKT_VLD_IN,
  output                       PKT_RDY_OUT,

  // SpiNNaker link asynchronous interface
  output                 [6:0] SL_DATA_2OF7_OUT,
  input                        SL_ACK_IN
);

  //-------------------------------------------------------------
  // internal signals
  //-------------------------------------------------------------
  wire [6:0] flt_data_2of7;  //2-of-7 encoded data
  wire       flt_vld;
  wire       flt_rdy;

		
  pkt_serializer ps
  (
    .CLK_IN           (CLK_IN),
    .RESET_IN         (RESET_IN),
    .PKT_DATA_IN      (PKT_DATA_IN),
    .PKT_VLD_IN       (PKT_VLD_IN),
    .PKT_RDY_OUT      (PKT_RDY_OUT),
    .flt_data_2of7    (flt_data_2of7),
    .flt_vld          (flt_vld),
    .flt_rdy          (flt_rdy)
  );

  spio_spinnaker_link_sync_to_async_fifo fo
  (
    .CLK_IN           (CLK_IN),
    .RESET_IN         (RESET_IN),
    .flt_data_2of7    (flt_data_2of7),
    .flt_vld          (flt_vld),
    .flt_rdy          (flt_rdy),
    .SL_DATA_2OF7_OUT (SL_DATA_2OF7_OUT),
    .SL_ACK_IN        (SL_ACK_IN)
  );
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module pkt_serializer
(
  input                         CLK_IN,
  input                         RESET_IN,

  // packet interface
  input       [`PKT_BITS - 1:0] PKT_DATA_IN,
  input                         PKT_VLD_IN,
  output reg                    PKT_RDY_OUT,

  // flit interface
  output reg              [6:0] flt_data_2of7,
  output reg                    flt_vld,
  input                         flt_rdy
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  //# Xilinx recommends one-hot state encoding
  localparam STATE_BITS = 2;
  localparam IDLE_ST    = 0;
  localparam PARK_ST    = IDLE_ST + 1;
  localparam TRAN_ST    = PARK_ST + 1;


  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  reg   [`PKT_BITS - 1:0] pkt_buf;   // buffer incoming pkt
  reg                     long_pkt;  // remember pkt length

  reg                     flt_busy;  // flit interface busy
  reg               [4:0] flt_cnt;   // count sent flits

  reg                     eop;       // time to send end-of-packet

  reg  [STATE_BITS - 1:0] state;     // current state


  //-------------------------------------------------------------
  // NRZ 2-of-7 encoder
  //-------------------------------------------------------------
  function [6:0] encode_nrz_2of7 ;
    input [4:0] din;
    input [6:0] old_din;

    casex (din)
      5'b00000 : encode_nrz_2of7 = old_din ^ 7'b0010001; // 0
      5'b00001 : encode_nrz_2of7 = old_din ^ 7'b0010010; // 1
      5'b00010 : encode_nrz_2of7 = old_din ^ 7'b0010100; // 2
      5'b00011 : encode_nrz_2of7 = old_din ^ 7'b0011000; // 3
      5'b00100 : encode_nrz_2of7 = old_din ^ 7'b0100001; // 4
      5'b00101 : encode_nrz_2of7 = old_din ^ 7'b0100010; // 5
      5'b00110 : encode_nrz_2of7 = old_din ^ 7'b0100100; // 6
      5'b00111 : encode_nrz_2of7 = old_din ^ 7'b0101000; // 7
      5'b01000 : encode_nrz_2of7 = old_din ^ 7'b1000001; // 8
      5'b01001 : encode_nrz_2of7 = old_din ^ 7'b1000010; // 9
      5'b01010 : encode_nrz_2of7 = old_din ^ 7'b1000100; // 10
      5'b01011 : encode_nrz_2of7 = old_din ^ 7'b1001000; // 11
      5'b01100 : encode_nrz_2of7 = old_din ^ 7'b0000011; // 12
      5'b01101 : encode_nrz_2of7 = old_din ^ 7'b0000110; // 13
      5'b01110 : encode_nrz_2of7 = old_din ^ 7'b0001100; // 14
      5'b01111 : encode_nrz_2of7 = old_din ^ 7'b0001001; // 15
      5'b1xxxx : encode_nrz_2of7 = old_din ^ 7'b1100000; // EOP
      default  : encode_nrz_2of7 = 7'bxxxxxxx;
    endcase
  endfunction


  //-------------------------------------------------------------
  // packet interface: generate PKT_RDY_OUT
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      PKT_RDY_OUT <= 1'b1;
    else
      case (state)
        IDLE_ST: if (PKT_VLD_IN)
                   PKT_RDY_OUT <= 1'b0;  // start new pkt, not ready for next
                 else
                   PKT_RDY_OUT <= 1'b1;  // waiting for next pkt

        PARK_ST:   PKT_RDY_OUT <= 1'b0;  // not ready yet for next pkt

        default: if (eop && !flt_busy)
                   PKT_RDY_OUT <= 1'b1;  // finished, ready for next pkt
                 else
                   PKT_RDY_OUT <= 1'b0;  // not ready yet for next pkt
      endcase 


  //-------------------------------------------------------------
  // buffer packet (or part of it) to serialize it
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    case (state)
      IDLE_ST:
        case ({PKT_VLD_IN, flt_busy})
          2'b10:   pkt_buf <= PKT_DATA_IN >> 4;  // first nibble gone
          2'b11:   pkt_buf <= PKT_DATA_IN;       // park new packet
        endcase

      default: if (!flt_busy)
                   pkt_buf <= pkt_buf >> 4;      // prepare for next nibble
    endcase 


  //-------------------------------------------------------------
  // remember length of packet
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    if ((state == IDLE_ST) && PKT_VLD_IN)
      long_pkt <= PKT_DATA_IN[1];


  //-------------------------------------------------------------
  // flit interface: generate flt_data_2of7
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      flt_data_2of7 <= 7'd0;
    else
      case (state)
        IDLE_ST: if (PKT_VLD_IN && !flt_busy)
                   flt_data_2of7 <= encode_nrz_2of7 ({1'b0, PKT_DATA_IN[3:0]},
                                                 flt_data_2of7
                                               );  // first nibble

        default: if (!flt_busy)
                   flt_data_2of7 <= encode_nrz_2of7 ({eop, pkt_buf[3:0]},
                                                 flt_data_2of7
                                               );  // next nibble or eop
	                                           // first if parked
      endcase 


  //-------------------------------------------------------------
  // flit interface: generate flt_vld
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      flt_vld <= 1'b0;
    else
      case (state)
        IDLE_ST: if (!flt_busy)
                   if (PKT_VLD_IN)
                     flt_vld <= 1'b1;  // first flit in pkt
                   else
                     flt_vld <= 1'b0;  // no new flit to send

        default: flt_vld <= 1'b1;  // next flit always available
      endcase 


  //-------------------------------------------------------------
  // flit interface busy
  //-------------------------------------------------------------
  always @ (*)
    flt_busy = flt_vld && !flt_rdy;


  //-------------------------------------------------------------
  // keep track of how many flits have been sent
  //-------------------------------------------------------------
  always @(posedge CLK_IN)
    case (state)
      IDLE_ST,
      PARK_ST:   flt_cnt <= 1;

      default: if (!flt_busy)
                 flt_cnt <= flt_cnt + 1;  // one more flit gone
    endcase 


  //-------------------------------------------------------------
  // time to send end-of-packet
  //-------------------------------------------------------------
  always @ (*)
    eop = (!long_pkt && (flt_cnt == 10)) || (flt_cnt == 18);


  //-------------------------------------------------------------
  // state machine
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      state <= IDLE_ST;
    else
      case (state)
        IDLE_ST:
          case ({PKT_VLD_IN, flt_busy})
            2'b10:   state <= TRAN_ST;  // start new packet
	    2'b11:   state <= PARK_ST;  // park new packet
            default: state <= IDLE_ST;  // wait for new packet
          endcase

        PARK_ST: if (!flt_busy)
                     state <= TRAN_ST;   

        default: if (eop && !flt_busy)
                     state <= IDLE_ST;  // done with packet
      endcase 
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module spio_spinnaker_link_sync_to_async_fifo
(
  input            CLK_IN,
  input            RESET_IN,

  // packet serializer interface
  input      [6:0] flt_data_2of7,
  input            flt_vld,
  output reg       flt_rdy,

  // SpiNNaker link interface
  output reg [6:0] SL_DATA_2OF7_OUT,
  input            SL_ACK_IN
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  localparam ADDR_WIDTH = 2;
  localparam BUFF_DEPTH = 4;


  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  // data buffer
  reg               [6:0] buffer [BUFF_DEPTH - 1:0];

  reg  [ADDR_WIDTH - 1:0] rdpg;    // read pointer
  wire [ADDR_WIDTH - 1:0] s_rdpg;  // synchronized read pointer
  reg  [ADDR_WIDTH - 1:0] wrpg;    // write pointer

  // buffer status
  reg  buf_empty;
  reg  buf_full;
  reg  buf_nf;

  // buffer operations
  reg  vld_rd;
  reg  vld_wr;


  // async interface
  reg old_ack;


  //-------------------------------------------------------------
  // gray code counter
  //-------------------------------------------------------------
  function [ADDR_WIDTH - 1:0] gray_count ;
    input [ADDR_WIDTH - 1:0] ctr;

    case (ctr)
      2'b00: gray_count = 2'b01;
      2'b01: gray_count = 2'b11;
      2'b11: gray_count = 2'b10;
      2'b10: gray_count = 2'b00;
    endcase
  endfunction


  //-------------------------------------------------------------
  // packet serializer interface: generate flt_rdy
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      flt_rdy <= 1'b1;
    else
      if (buf_full || (buf_nf && vld_wr))  // stall if full or going full
        flt_rdy <= 1'b0;
      else
        flt_rdy <= 1'b1;


  //---------------------------------------------------------------
  // valid write operation
  //---------------------------------------------------------------
  always @ (*)
    vld_wr = flt_vld && flt_rdy;


  //-------------------------------------------------------------
  // write to buffer
  //-------------------------------------------------------------
  always @ (posedge CLK_IN)
    if (vld_wr)
      buffer[wrpg] <= flt_data_2of7;


  //---------------------------------------------------------------
  // write pointer in gray code
  //---------------------------------------------------------------
  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      wrpg <= 0;
    else
      if (vld_wr)
        wrpg <= gray_count (wrpg);


  //---------------------------------------------------------------
  // synchronize the read pointer -- cross async/sync boundary
  //---------------------------------------------------------------
  spinnaker_fpgas_sync #(.SIZE (ADDR_WIDTH)) rdp_sync
  (
    .CLK_IN (CLK_IN),
    .IN     (rdpg),
    .OUT    (s_rdpg)
  );


  //---------------------------------------------------------------
  // buffer full indication
  //---------------------------------------------------------------
  always @(*)
    buf_full = #1 (s_rdpg == gray_count (wrpg));


  //---------------------------------------------------------------
  // buffer near full indication (will go full if write happens)
  //---------------------------------------------------------------
  always @(*)
    buf_nf = #1 (s_rdpg == gray_count (gray_count (wrpg)));


  //-------------------------------------------------------------
  // wait for not empty to start async output process
  //-------------------------------------------------------------
  wire init;

  cel_n initc
  (
    .rst (RESET_IN),
    .a   (~buf_empty),
    .bn  (~RESET_IN),
    .o   (init)
  );


  //-------------------------------------------------------------
  // iterate to send flits to SpiNNaker
  //-------------------------------------------------------------
  reg  loop_r;
  wire loop_a;

  always @(*)
    loop_r = ~(loop_a || ~init);


  //-------------------------------------------------------------
  // sequence operations:
  //  wait for not empty, read buffer, update rdpg and wait for ack
  //-------------------------------------------------------------
  wire s0_s1;

  wire upd_r;
  wire upd_a;
  wire akd_r;
  wire akd_a;
  wire empt_r;
  wire empt_a;

  tel upds
  (
    .rst (RESET_IN),
    .ri  (loop_r),
    .ai  (s0_s1),
    .ro  (upd_r),
    .ao  (upd_a)
  );

  tel akds
  (
    .rst (RESET_IN),
    .ri  (s0_s1),
    .ai  (loop_a),
    .ro  (akd_r),
    .ao  (akd_a)
  );


  //-------------------------------------------------------------
  // SpiNNaker async link interface: generate SL_DATA_2OF7_OUT
  //-------------------------------------------------------------
  always @ (posedge loop_r or posedge RESET_IN)
    if (RESET_IN)
      SL_DATA_2OF7_OUT <= 7'd0;
    else
      SL_DATA_2OF7_OUT <= buffer[rdpg];


  //---------------------------------------------------------------
  // update read pointer (gray code -- to cross async/sync boundary)
  //---------------------------------------------------------------
  always @ (posedge upd_r or posedge RESET_IN)
    if (RESET_IN)
      rdpg <= 0;
    else
      rdpg <= gray_count (rdpg);


  //-------------------------------------------------------------
  // ack read pointer update
  //-------------------------------------------------------------
  cel updc
  (
    .rst (RESET_IN),
    .a   (upd_r),
    .b   (~upd_a),
    .o   (upd_a)
  );


  //---------------------------------------------------------------
  // wait for acked and not empty
  //---------------------------------------------------------------
  reg  acked;

  cel3_n2 akdc
  (
    .rst (RESET_IN),
    .a   (akd_r),
    .bn  (~acked),
    .cn  (buf_empty),
    .o   (akd_a)
  );


  //---------------------------------------------------------------
  // buffer full indication
  //---------------------------------------------------------------
  always @(*)
    buf_empty = (rdpg == wrpg);


  //---------------------------------------------------------------
  // output data has been acked (detect transition on SL_ACK_IN)
  //---------------------------------------------------------------
  always @ (*)
    acked = (old_ack != SL_ACK_IN);


  //-------------------------------------------------------------
  // remember previous value of ack to detect transition
  //-------------------------------------------------------------
  always @ (posedge loop_r)
    old_ack <= SL_ACK_IN;
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// -------------------------------------------------------------------------
//  spiNNaker link 2-of-7 data sync to async fifo
//
// -------------------------------------------------------------------------
// AUTHOR
//  lap - luis.plana@manchester.ac.uk
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
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
endmodule

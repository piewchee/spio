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
        case (wrpg)
          2'b00: wrpg <= 2'b01;
          2'b01: wrpg <= 2'b11;
          2'b11: wrpg <= 2'b10;
          2'b10: wrpg <= 2'b00;
	endcase


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
    case ({wrpg, s_rdpg})
      4'b00_01,
      4'b01_11,
      4'b11_10,
      4'b10_00: buf_full = #1 1'b1;
      default:  buf_full = #1 1'b0;
    endcase


  //---------------------------------------------------------------
  // buffer near full iindication
  //---------------------------------------------------------------
  always @(*)
    case ({wrpg, s_rdpg})
      4'b00_11,
      4'b01_10,
      4'b11_00,
      4'b10_01: buf_nf = #1 1'b1;
      default:  buf_nf = #1 1'b0;
    endcase


  //-------------------------------------------------------------
  // iterate to send flits to SpiNNaker
  //-------------------------------------------------------------
  reg  loop_r;
  wire loop_a;

  always @(*)
    loop_r = ~(loop_a || RESET_IN);


  //-------------------------------------------------------------
  // sequence operations:
  //  wait for not empty, read buffer, update rdpg and wait for ack
  //-------------------------------------------------------------
  wire empty_to_read;
  wire read_to_update;
  wire update_to_acked;

  wire empt_r;
  wire empt_a;
  wire rd_r;
  wire rd_a;
  wire upd_r;
  wire upd_a;
  wire akd_r;
  wire akd_a;

  tel empts
  (
    .rst (RESET_IN),
    .ri  (loop_r),
    .ai  (empty_to_read),
    .ro  (empt_r),
    .ao  (empt_a)
  );

  tel rds
  (
    .rst (RESET_IN),
    .ri  (empty_to_read),
    .ai  (read_to_update),
    .ro  (rd_r),
    .ao  (rd_a)
  );

  tel upds
  (
    .rst (RESET_IN),
    .ri  (read_to_update),
    .ai  (update_to_acked),
    .ro  (upd_r),
    .ao  (upd_a)
  );

  tel akds
  (
    .rst (RESET_IN),
    .ri  (update_to_acked),
    .ai  (loop_a),
    .ro  (akd_r),
    .ao  (akd_a)
  );


  //---------------------------------------------------------------
  // wait for not empty
  //---------------------------------------------------------------
  cel_p emptc
  (
    .rst (RESET_IN),
    .a   (empt_r),
    .bp  (~buf_empty),
    .o   (empt_a)
  );


  //---------------------------------------------------------------
  // buffer full indication
  //---------------------------------------------------------------
  always @(*)
    buf_empty = (rdpg == wrpg);


  //-------------------------------------------------------------
  // SpiNNaker async link interface: generate SL_DATA_2OF7_OUT
  //-------------------------------------------------------------
  always @ (posedge rd_r or posedge RESET_IN)
    if (RESET_IN)
      SL_DATA_2OF7_OUT <= 7'd0;
    else
      SL_DATA_2OF7_OUT <= buffer[rdpg];


  //-------------------------------------------------------------
  // ack read from buffer
  //-------------------------------------------------------------
  cel rdc
  (
    .rst (RESET_IN),
    .a   (rd_r),
    .b   (~rd_a),
    .o   (rd_a)
  );


  //---------------------------------------------------------------
  // update read pointer (gray code -- to cross async/sync boundary)
  //---------------------------------------------------------------
  always @ (posedge upd_r or posedge RESET_IN)
    if (RESET_IN)
      rdpg <= 0;
    else
      case (rdpg)
        2'b00: rdpg <= 2'b01;
        2'b01: rdpg <= 2'b11;
        2'b11: rdpg <= 2'b10;
        2'b10: rdpg <= 2'b00;
      endcase


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
  // wait for acked
  //---------------------------------------------------------------
  reg  acked;

  cel_n akdc
  (
    .rst (RESET_IN),
    .a   (akd_r),
    .bn  (~acked),
    .o   (akd_a)
  );


  //---------------------------------------------------------------
  // output data has been acked (detect transition on SL_ACK_IN)
  //---------------------------------------------------------------
  always @ (*)
    acked = (old_ack != SL_ACK_IN);


  //-------------------------------------------------------------
  // remember previous value of ack to detect transition
  //-------------------------------------------------------------
  always @ (posedge rd_r)
    old_ack <= SL_ACK_IN;
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
endmodule

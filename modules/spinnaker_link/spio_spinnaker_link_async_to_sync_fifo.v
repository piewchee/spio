// -------------------------------------------------------------------------
//  spiNNaker link 2-of-7 data async to sync fifo
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
module cel
(
  input  wire rst,

  input  wire a,
  input  wire b,
  output reg  o
);
  wire c;

  assign #1 c = (~a & ~b & o) | (a & b & ~o);

  always @(posedge c or posedge rst)
    if (rst)
      o <= #1 1'b0;
    else
      o <= #1 ~o;
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module cel_p
(
  input  wire rst,

  input  wire a,
  input  wire bp,
  output reg  o
);
  wire c;

  assign #1 c = (~a & o) | (a & bp & ~o);

  always @(posedge c or posedge rst)
    if (rst)
      o <= #1 1'b0;
    else
      o <= #1 ~o;
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module cel_n
(
  input  wire rst,

  input  wire a,
  input  wire bn,
  output reg  o
);
  wire c;

  assign #1 c = (~a & ~bn & o) | (a & ~o);

  always @(posedge c or posedge rst)
    if (rst)
      o <= #1 1'b0;
    else
      o <= #1 ~o;
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module sel
(
  input  wire rst,
  input  wire ri,
  output wire ai,
  output wire ro,
  input  wire ao
);
  wire i;

  cel c0
  (
    .rst (rst),
    .a   (ri), 
    .b   (ao),
    .o   (i)
  );

  assign #1 ro = ~i & ri;

  assign #1 ai = ~(~i | ao);
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module tel
(
  input  wire rst,
  input  wire ri,
  output wire ai,
  output wire ro,
  input  wire ao
);
  wire i;

  cel c0
  (
    .rst (rst),
    .a   (ri), 
    .b   (ao),
    .o   (ai)
  );

  assign #1 ro = ~ai & ri;
endmodule
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
`timescale 1ns / 1ps
module spio_spinnaker_link_async_to_sync_fifo
(
  input                         CLK_IN,
  input                         RESET_IN,

  // asynchronous SpiNNaker link interface
  input                   [6:0] SL_DATA_2OF7_IN,
  output reg                    SL_ACK_OUT,

  // synchronous packet deserializer interface
  output reg              [6:0] flt_data_2of7,
  output reg                    flt_vld,
  input                         flt_rdy
);

  //---------------------------------------------------------------
  // constants
  //---------------------------------------------------------------
  localparam ADDR_WIDTH = 2;
  localparam BUFF_DEPTH = 4;


  //---------------------------------------------------------------
  // internal signals
  //---------------------------------------------------------------
  // async interface
  reg  ack_out;   // ack to SpiNNaker

  reg  new_flit;  // new flit arrived
  wire ack_flit;  // send ack to SpiNNaker (toggle SL_ACK_OUT)

  reg  [6:0] old_data;  // remember previous nrz 2of7 data for rtz translation
  reg  [6:0] rtz_data;  // data translated from nrz to rtz

  // data buffer
  reg               [6:0] buffer [BUFF_DEPTH - 1:0];

  // buffer pointers
  reg  [ADDR_WIDTH - 1:0] rdpg;    // read pointer
  reg  [ADDR_WIDTH - 1:0] wrpg;    // write pointer
  wire [ADDR_WIDTH - 1:0] s_wrpg;  // synchronized write pointer

  // buffer status
  reg  buf_empty;
  reg  buf_full;

  // buffer operations
  reg  vld_rd;

  // sync interface
  reg  flt_busy;


  //-------------------------------------------------------------
  // 2-of-7 symbol detector (correct data, eop or error)
  //-------------------------------------------------------------
  function detect_2of7 ;
    input [6:0] data;

    case (data)
      0, 1, 2, 4,
      8, 16, 32,
      64:         detect_2of7 = 0;  // incomplete (no/single-bit change)
      default:    detect_2of7 = 1;  // correct data, eop or error
    endcase
  endfunction


  //-------------------------------------------------------------
  // SpiNNaker async link interface: generate SL_ACK_OUT
  //-------------------------------------------------------------
  always @(*)
    SL_ACK_OUT = ~(ack_out || RESET_IN);


  //-------------------------------------------------------------
  // ack signal to SpiNNaker (merged with RESET_IN to ack on reset exit)
  //-------------------------------------------------------------
  always @(posedge ack_flit or posedge RESET_IN)
    if (RESET_IN)
      ack_out <= 1'b0;
    else
      ack_out <= ~ack_out;


  //-------------------------------------------------------------
  // remember previous nrz 2of7 data for translation to rtz
  //-------------------------------------------------------------
  always @(posedge ack_flit or posedge RESET_IN)
    if (RESET_IN)
      old_data <= 1'b0;
    else
      old_data <= SL_DATA_2OF7_IN;


  //---------------------------------------------------------------
  // translate data from nrz to rtz
  //---------------------------------------------------------------
  always @(*)
    rtz_data = SL_DATA_2OF7_IN ^ old_data;


  //-------------------------------------------------------------
  // detect the arrival of a new flit (2 or more transitions)
  //-------------------------------------------------------------
  always @(*)
    new_flit = detect_2of7 (rtz_data);


  //-------------------------------------------------------------
  // sequence operations:
  // wait for not full, write buffer and update wrpg
  //-------------------------------------------------------------
  wire full_to_write;
  wire write_to_update;

  wire full_r;
  wire full_a;
  wire wrt_r;
  wire wrt_a;
  wire upd_r;
  wire upd_a;

  tel fulls
  (
    .rst (RESET_IN),
    .ri  (new_flit),
    .ai  (full_to_write),
    .ro  (full_r),
    .ao  (full_a)
  );

  tel wrts
  (
    .rst (RESET_IN),
    .ri  (full_to_write),
    .ai  (write_to_update),
    .ro  (wrt_r),
    .ao  (wrt_a)
  );

  tel upds
  (
    .rst (RESET_IN),
    .ri  (write_to_update),
    .ai  (ack_flit),
    .ro  (upd_r),
    .ao  (upd_a)
  );


  //---------------------------------------------------------------
  // wait for not full
  //---------------------------------------------------------------
  cel_p fullc
  (
    .rst (RESET_IN),
    .a   (full_r),
    .bp  (~buf_full),
    .o   (full_a)
  );


  //---------------------------------------------------------------
  // buffer full indication
  //---------------------------------------------------------------
  always @(*)
    case ({wrpg, rdpg})
      4'b00_01,
      4'b01_11,
      4'b11_10,
      4'b10_00: buf_full = #1 1'b1;
      default:  buf_full = #1 1'b0;
    endcase


  //-------------------------------------------------------------
  // write to buffer
  //-------------------------------------------------------------
  always @ (posedge wrt_r)
    buffer[wrpg] <= rtz_data;


  //-------------------------------------------------------------
  // ack write to buffer
  //-------------------------------------------------------------
  cel wrtc
  (
    .rst (RESET_IN),
    .a   (wrt_r),
    .b   (~wrt_a),
    .o   (wrt_a)
  );


  //---------------------------------------------------------------
  // update write pointer (gray code -- to cross async/sync boundary)
  //---------------------------------------------------------------
  always @ (posedge upd_r or posedge RESET_IN)
    if (RESET_IN)
      wrpg <= 0;
    else
      case (wrpg)
        2'b00: wrpg <= 2'b01;
        2'b01: wrpg <= 2'b11;
        2'b11: wrpg <= 2'b10;
        2'b10: wrpg <= 2'b00;
      endcase


  //-------------------------------------------------------------
  // ack write pointer update
  //-------------------------------------------------------------
  cel updc
  (
    .rst (RESET_IN),
    .a   (upd_r),
    .b   (~upd_a),
    .o   (upd_a)
  );


  //-------------------------------------------------------------
  // packet deserializer interface: generate flt_data_2of7
  //-------------------------------------------------------------
  always @ (posedge CLK_IN)
    if (vld_rd)
      flt_data_2of7 <= buffer[rdpg];


  //-------------------------------------------------------------
  // packet deserializer interface: generate flt_vld
  //-------------------------------------------------------------
  always @(posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      flt_vld <= 1'b0;
    else
      if (!flt_busy)
        if (!buf_empty)
          flt_vld <= 1'b1;
        else
          flt_vld <= 1'b0;


  //---------------------------------------------------------------
  // deserializer not ready for new flit
  //---------------------------------------------------------------
  always @(*)
    flt_busy = flt_vld && !flt_rdy;


  //---------------------------------------------------------------
  // valid read operation
  //---------------------------------------------------------------
  always @ (*)
    vld_rd = !buf_empty && !flt_busy;


  //---------------------------------------------------------------
  // read pointer in gray code
  //---------------------------------------------------------------
  always @ (posedge CLK_IN or posedge RESET_IN)
    if (RESET_IN)
      rdpg <= 0;
    else
      if (vld_rd)      // update pointer on valid read
        case (rdpg)
          2'b00: rdpg <= 2'b01;
          2'b01: rdpg <= 2'b11;
          2'b11: rdpg <= 2'b10;
          2'b10: rdpg <= 2'b00;
	endcase


  //---------------------------------------------------------------
  // synchronize the write pointer -- cross async/sync boundary
  //---------------------------------------------------------------
  spinnaker_fpgas_sync #(.SIZE (ADDR_WIDTH)) wrp_sync
  (
    .CLK_IN (CLK_IN),
    .IN     (wrpg),
    .OUT    (s_wrpg)
  );


  //---------------------------------------------------------------
  // buffer empty indication
  //---------------------------------------------------------------
  always @(*)
    buf_empty = (rdpg == s_wrpg);
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
endmodule

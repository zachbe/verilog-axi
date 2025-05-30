/*

Copyright (c) 2019 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

// Language: Verilog 2001

`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * AXI4 to AXI4-Lite adapter (read)
 */
module axi_axil_adapter_rd #
(
    // Width of address bus in bits
    parameter ADDR_WIDTH = 32,
    // Width of input (slave) AXI interface data bus in bits
    parameter AXI_DATA_WIDTH = 32,
    // Width of input (slave) AXI interface wstrb (width of data bus in words)
    parameter AXI_STRB_WIDTH = (AXI_DATA_WIDTH/8),
    // Width of AXI ID signal
    parameter AXI_ID_WIDTH = 8,
    // Width of output (master) AXI lite interface data bus in bits
    parameter AXIL_DATA_WIDTH = 32,
    // Width of output (master) AXI lite interface wstrb (width of data bus in words)
    parameter AXIL_STRB_WIDTH = (AXIL_DATA_WIDTH/8),
    // When adapting to a wider bus, re-pack full-width burst instead of passing through narrow burst if possible
    parameter CONVERT_BURST = 1,
    // When adapting to a wider bus, re-pack all bursts instead of passing through narrow burst if possible
    parameter CONVERT_NARROW_BURST = 0
)
(
    input  wire                        clk,
    input  wire                        rst,

    /*
     * AXI slave interface
     */
    input  wire [AXI_ID_WIDTH-1:0]     s_axi_arid,
    input  wire [ADDR_WIDTH-1:0]       s_axi_araddr,
    input  wire [7:0]                  s_axi_arlen,
    input  wire [2:0]                  s_axi_arsize,
    input  wire [1:0]                  s_axi_arburst,
    input  wire                        s_axi_arlock,
    input  wire [3:0]                  s_axi_arcache,
    input  wire [2:0]                  s_axi_arprot,
    input  wire                        s_axi_arvalid,
    output wire                        s_axi_arready,
    output wire [AXI_ID_WIDTH-1:0]     s_axi_rid,
    output wire [AXI_DATA_WIDTH-1:0]   s_axi_rdata,
    output wire [1:0]                  s_axi_rresp,
    output wire                        s_axi_rlast,
    output wire                        s_axi_rvalid,
    input  wire                        s_axi_rready,

    /*
     * AXI lite master interface
     */
    output wire [ADDR_WIDTH-1:0]       m_axil_araddr,
    output wire [2:0]                  m_axil_arprot,
    output wire                        m_axil_arvalid,
    input  wire                        m_axil_arready,
    input  wire [AXIL_DATA_WIDTH-1:0]  m_axil_rdata,
    input  wire [1:0]                  m_axil_rresp,
    input  wire                        m_axil_rvalid,
    output wire                        m_axil_rready
);

parameter AXI_ADDR_BIT_OFFSET = $clog2(AXI_STRB_WIDTH);
parameter AXIL_ADDR_BIT_OFFSET = $clog2(AXIL_STRB_WIDTH);
parameter AXI_WORD_WIDTH = AXI_STRB_WIDTH;
parameter AXIL_WORD_WIDTH = AXIL_STRB_WIDTH;
parameter AXI_WORD_SIZE = AXI_DATA_WIDTH/AXI_WORD_WIDTH;
parameter AXIL_WORD_SIZE = AXIL_DATA_WIDTH/AXIL_WORD_WIDTH;
parameter AXI_BURST_SIZE = $clog2(AXI_STRB_WIDTH);
parameter AXIL_BURST_SIZE = $clog2(AXIL_STRB_WIDTH);

// output bus is wider
parameter EXPAND = AXIL_STRB_WIDTH > AXI_STRB_WIDTH;
parameter DATA_WIDTH = EXPAND ? AXIL_DATA_WIDTH : AXI_DATA_WIDTH;
parameter STRB_WIDTH = EXPAND ? AXIL_STRB_WIDTH : AXI_STRB_WIDTH;
// required number of segments in wider bus
parameter SEGMENT_COUNT = EXPAND ? (AXIL_STRB_WIDTH / AXI_STRB_WIDTH) : (AXI_STRB_WIDTH / AXIL_STRB_WIDTH);
// data width and keep width per segment
parameter SEGMENT_DATA_WIDTH = DATA_WIDTH / SEGMENT_COUNT;
parameter SEGMENT_STRB_WIDTH = STRB_WIDTH / SEGMENT_COUNT;

// bus width assertions
initial begin
    if (AXI_WORD_SIZE * AXI_STRB_WIDTH != AXI_DATA_WIDTH) begin
        $error("Error: AXI slave interface data width not evenly divisble (instance %m)");
        $finish;
    end

    if (AXIL_WORD_SIZE * AXIL_STRB_WIDTH != AXIL_DATA_WIDTH) begin
        $error("Error: AXI lite master interface data width not evenly divisble (instance %m)");
        $finish;
    end

    if (AXI_WORD_SIZE != AXIL_WORD_SIZE) begin
        $error("Error: word size mismatch (instance %m)");
        $finish;
    end

    if (2**$clog2(AXI_WORD_WIDTH) != AXI_WORD_WIDTH) begin
        $error("Error: AXI slave interface word width must be even power of two (instance %m)");
        $finish;
    end

    if (2**$clog2(AXIL_WORD_WIDTH) != AXIL_WORD_WIDTH) begin
        $error("Error: AXI lite master interface word width must be even power of two (instance %m)");
        $finish;
    end
end

localparam [1:0]
    STATE_IDLE = 2'd0,
    STATE_DATA = 2'd1,
    STATE_DATA_READ = 2'd2,
    STATE_DATA_SPLIT = 2'd3;

reg [1:0] state_reg = STATE_IDLE, state_next;

reg [AXI_ID_WIDTH-1:0] id_reg = {AXI_ID_WIDTH{1'b0}}, id_next;
reg [ADDR_WIDTH-1:0] addr_reg = {ADDR_WIDTH{1'b0}}, addr_next;
reg [DATA_WIDTH-1:0] data_reg = {DATA_WIDTH{1'b0}}, data_next;
reg [1:0] resp_reg = 2'd0, resp_next;
reg [7:0] burst_reg = 8'd0, burst_next;
reg [2:0] burst_size_reg = 3'd0, burst_size_next;
reg [7:0] master_burst_reg = 8'd0, master_burst_next;
reg [2:0] master_burst_size_reg = 3'd0, master_burst_size_next;

reg s_axi_arready_reg = 1'b0, s_axi_arready_next;
reg [AXI_ID_WIDTH-1:0] s_axi_rid_reg = {AXI_ID_WIDTH{1'b0}}, s_axi_rid_next;
reg [AXI_DATA_WIDTH-1:0] s_axi_rdata_reg = {AXI_DATA_WIDTH{1'b0}}, s_axi_rdata_next;
reg [1:0] s_axi_rresp_reg = 2'd0, s_axi_rresp_next;
reg s_axi_rlast_reg = 1'b0, s_axi_rlast_next;
reg s_axi_rvalid_reg = 1'b0, s_axi_rvalid_next;

reg [ADDR_WIDTH-1:0] m_axil_araddr_reg = {ADDR_WIDTH{1'b0}}, m_axil_araddr_next;
reg [2:0] m_axil_arprot_reg = 3'd0, m_axil_arprot_next;
reg m_axil_arvalid_reg = 1'b0, m_axil_arvalid_next;
reg m_axil_rready_reg = 1'b0, m_axil_rready_next;

assign s_axi_arready = s_axi_arready_reg;
assign s_axi_rid = s_axi_rid_reg;
assign s_axi_rdata = s_axi_rdata_reg;
assign s_axi_rresp = s_axi_rresp_reg;
assign s_axi_rlast = s_axi_rlast_reg;
assign s_axi_rvalid = s_axi_rvalid_reg;

assign m_axil_araddr = m_axil_araddr_reg;
assign m_axil_arprot = m_axil_arprot_reg;
assign m_axil_arvalid = m_axil_arvalid_reg;
assign m_axil_rready = m_axil_rready_reg;

always @* begin
    if (SEGMENT_COUNT == 1) begin
        // master output is same width; direct transfer with no splitting/merging
        case (state_reg)
            STATE_IDLE: begin
                // idle state; wait for new burst
                if (s_axi_arready && s_axi_arvalid) begin
                    state_next = STATE_DATA;
                    id_next = s_axi_arid;
                    addr_next = s_axi_araddr;
                    data_next = data_reg;
                    resp_next = resp_reg;
                    burst_next = s_axi_arlen;
                    burst_size_next = s_axi_arsize;
                    master_burst_next = master_burst_reg;
                    master_burst_size_next = master_burst_size_reg;
                    s_axi_arready_next = 1'b0;
                    s_axi_rid_next = s_axi_rid_reg;
                    s_axi_rdata_next = s_axi_rdata_reg;
                    s_axi_rresp_next = s_axi_rresp_reg;
                    s_axi_rlast_next = s_axi_rlast_reg;
                    s_axi_rvalid_next = s_axi_rvalid_reg && !s_axi_rready;
                    m_axil_araddr_next = s_axi_araddr;
                    m_axil_arprot_next = s_axi_arprot;
                    m_axil_arvalid_next = 1'b1;
                    m_axil_rready_next = 1'b0;
                end else begin
                    state_next = STATE_IDLE;
                    id_next = id_reg;
                    addr_next = addr_reg;
                    data_next = data_reg;
                    resp_next = resp_reg;
                    burst_next = burst_reg;
                    burst_size_next = burst_size_reg;
                    master_burst_next = master_burst_reg;
                    master_burst_size_next = master_burst_size_reg;
                    s_axi_arready_next = !m_axil_arvalid;
                    s_axi_rid_next = s_axi_rid_reg;
                    s_axi_rdata_next = s_axi_rdata_reg;
                    s_axi_rresp_next = s_axi_rresp_reg;
                    s_axi_rlast_next = s_axi_rlast_reg;
                    s_axi_rvalid_next = s_axi_rvalid_reg && !s_axi_rready;
                    m_axil_araddr_next = m_axil_araddr_reg;
                    m_axil_arprot_next = m_axil_arprot_reg;
                    m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
                    m_axil_rready_next = 1'b0;
                end
            end
            STATE_DATA: begin
                // data state; transfer read data
                if (m_axil_rready && m_axil_rvalid) begin
                    if (burst_reg == 0) begin
                        // last data word, return to idle
                        state_next = STATE_IDLE;
                        id_next = id_reg;
                        addr_next = addr_reg + (1 << burst_size_reg);
                        data_next = data_reg;
                        resp_next = resp_reg;
                        burst_next = burst_reg - 1;
                        burst_size_next = burst_size_reg;
                        master_burst_next = master_burst_reg;
                        master_burst_size_next = master_burst_size_reg;
                        s_axi_arready_next = !m_axil_arvalid;
                        s_axi_rid_next = id_reg;
                        s_axi_rdata_next = m_axil_rdata;
                        s_axi_rresp_next = m_axil_rresp;
                        s_axi_rlast_next = 1'b1;
                        s_axi_rvalid_next = 1'b1;
                        m_axil_araddr_next = m_axil_araddr_reg;
                        m_axil_arprot_next = m_axil_arprot_reg;
                        m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
                        m_axil_rready_next = 1'b0;
                    end else begin
                        // start new AXI lite read
                        state_next = STATE_DATA;
                        id_next = id_reg;
                        addr_next = addr_reg + (1 << burst_size_reg);
                        data_next = data_reg;
                        resp_next = resp_reg;
                        burst_next = burst_reg - 1;
                        burst_size_next = burst_size_reg;
                        master_burst_next = master_burst_reg;
                        master_burst_size_next = master_burst_size_reg;
                        s_axi_arready_next = 1'b0;
                        s_axi_rid_next = id_reg;
                        s_axi_rdata_next = m_axil_rdata;
                        s_axi_rresp_next = m_axil_rresp;
                        s_axi_rlast_next = 1'b0;
                        s_axi_rvalid_next = 1'b1;
                        m_axil_araddr_next = addr_next;
                        m_axil_arprot_next = m_axil_arprot_reg;
                        m_axil_arvalid_next = 1'b1;
                        m_axil_rready_next = 1'b0;
                    end
                end else begin
                    state_next = STATE_DATA;
                    id_next = id_reg;
                    addr_next = addr_reg;
                    data_next = data_reg;
                    resp_next = resp_reg;
                    burst_next = burst_reg;
                    burst_size_next = burst_size_reg;
                    master_burst_next = master_burst_reg;
                    master_burst_size_next = master_burst_size_reg;
                    s_axi_arready_next = 1'b0;
                    s_axi_rid_next = s_axi_rid_reg;
                    s_axi_rdata_next = s_axi_rdata_reg;
                    s_axi_rresp_next = s_axi_rresp_reg;
                    s_axi_rlast_next = s_axi_rlast_reg;
                    s_axi_rvalid_next = s_axi_rvalid_reg && !s_axi_rready;
                    m_axil_araddr_next = m_axil_araddr_reg;
                    m_axil_arprot_next = m_axil_arprot_reg;
                    m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
                    m_axil_rready_next = !s_axi_rvalid && !m_axil_arvalid;
                end
            end
        endcase
    end else if (EXPAND) begin
        // master output is wider; split reads
        case (state_reg)
            STATE_IDLE: begin
                // idle state; wait for new burst
                if (s_axi_arready && s_axi_arvalid) begin
                    id_next = s_axi_arid;
                    addr_next = s_axi_araddr;
                    data_next = data_reg;
                    resp_next = resp_reg;
                    burst_next = s_axi_arlen;
                    burst_size_next = s_axi_arsize;
                    master_burst_next = master_burst_reg;
                    s_axi_arready_next = 1'b0;
                    s_axi_rid_next = s_axi_rid_reg;
                    s_axi_rdata_next = s_axi_rdata_reg;
                    s_axi_rresp_next = s_axi_rresp_reg;
                    s_axi_rlast_next = s_axi_rlast_reg;
                    s_axi_rvalid_next = s_axi_rvalid_reg && !s_axi_rready;
                    m_axil_araddr_next = s_axi_araddr;
                    m_axil_arprot_next = s_axi_arprot;
                    m_axil_arvalid_next = 1'b1;
                    m_axil_rready_next = 1'b0;
                    if (CONVERT_BURST && s_axi_arcache[1] && (CONVERT_NARROW_BURST || s_axi_arsize == AXI_BURST_SIZE)) begin
                        // split reads
                        // require CONVERT_BURST and arcache[1] set
                        master_burst_size_next = AXIL_BURST_SIZE;
                        state_next = STATE_DATA_READ;
                    end else begin
                        // output narrow burst
                        master_burst_size_next = s_axi_arsize;
                        state_next = STATE_DATA;
                    end
                end else begin
                    state_next = STATE_IDLE;
                    id_next = id_reg;
                    addr_next = addr_reg;
                    data_next = data_reg;
                    resp_next = resp_reg;
                    burst_next = burst_reg;
                    burst_size_next = burst_size_reg;
                    master_burst_next = master_burst_reg;
                    master_burst_size_next = master_burst_size_reg;
                    s_axi_arready_next = !m_axil_arvalid;
                    s_axi_rid_next = s_axi_rid_reg;
                    s_axi_rdata_next = s_axi_rdata_reg;
                    s_axi_rresp_next = s_axi_rresp_reg;
                    s_axi_rlast_next = s_axi_rlast_reg;
                    s_axi_rvalid_next = s_axi_rvalid_reg && !s_axi_rready;
                    m_axil_araddr_next = m_axil_araddr_reg;
                    m_axil_arprot_next = m_axil_arprot_reg;
                    m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
                    m_axil_rready_next = 1'b0;
                end
            end
            STATE_DATA: begin
                if (m_axil_rready && m_axil_rvalid) begin
                    if (burst_reg == 0) begin
                        // last data word, return to idle
                        state_next = STATE_IDLE;
                        id_next = id_reg;
                        addr_next = addr_reg + (1 << burst_size_reg);
                        data_next = data_reg;
                        resp_next = resp_reg;
                        burst_next = burst_reg - 1;
                        burst_size_next = burst_size_reg;
                        master_burst_next = master_burst_reg;
                        master_burst_size_next = master_burst_size_reg;
                        s_axi_arready_next = !m_axil_arvalid;
                        s_axi_rid_next = id_reg;
                        s_axi_rdata_next = m_axil_rdata >> (addr_reg[AXI_ADDR_BIT_OFFSET:AXIL_ADDR_BIT_OFFSET-1] * AXI_DATA_WIDTH);
                        s_axi_rresp_next = m_axil_rresp;
                        s_axi_rlast_next = 1'b1;
                        s_axi_rvalid_next = 1'b1;
                        m_axil_araddr_next = m_axil_araddr_reg;
                        m_axil_arprot_next = m_axil_arprot_reg;
                        m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
                        m_axil_rready_next = 1'b0;
                    end else begin
                        // start new AXI lite read
                        state_next = STATE_DATA;
                        id_next = id_reg;
                        addr_next = addr_reg + (1 << burst_size_reg);
                        data_next = data_reg;
                        resp_next = resp_reg;
                        burst_next = burst_reg - 1;
                        burst_size_next = burst_size_reg;
                        master_burst_next = master_burst_reg;
                        master_burst_size_next = master_burst_size_reg;
                        s_axi_arready_next = 1'b0;
                        s_axi_rid_next = id_reg;
                        s_axi_rdata_next = m_axil_rdata >> (addr_reg[AXI_ADDR_BIT_OFFSET:AXIL_ADDR_BIT_OFFSET-1] * AXI_DATA_WIDTH);
                        s_axi_rresp_next = m_axil_rresp;
                        s_axi_rlast_next = 1'b0;
                        s_axi_rvalid_next = 1'b1;
                        m_axil_araddr_next = addr_next;
                        m_axil_arprot_next = m_axil_arprot_reg;
                        m_axil_arvalid_next = 1'b1;
                        m_axil_rready_next = 1'b0;
                    end
                end else begin
                    state_next = STATE_DATA;
                    id_next = id_reg;
                    addr_next = addr_reg;
                    data_next = data_reg;
                    resp_next = resp_reg;
                    burst_next = burst_reg;
                    burst_size_next = burst_size_reg;
                    master_burst_next = master_burst_reg;
                    master_burst_size_next = master_burst_size_reg;
                    s_axi_arready_next = 1'b0;
                    s_axi_rid_next = s_axi_rid_reg;
                    s_axi_rdata_next = s_axi_rdata_reg;
                    s_axi_rresp_next = s_axi_rresp_reg;
                    s_axi_rlast_next = s_axi_rlast_reg;
                    s_axi_rvalid_next = s_axi_rvalid_reg && !s_axi_rready;
                    m_axil_araddr_next = m_axil_araddr_reg;
                    m_axil_arprot_next = m_axil_arprot_reg;
                    m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
                    m_axil_rready_next = !s_axi_rvalid && !m_axil_arvalid;
                end
            end
            STATE_DATA_READ: begin
                if (m_axil_rready && m_axil_rvalid) begin
                    if (burst_reg == 0) begin
                        state_next = STATE_IDLE;
                        id_next = id_reg;
                        addr_next = addr_reg + (1 << burst_size_reg);
                        data_next = m_axil_rdata;
                        resp_next = m_axil_rresp;
                        burst_next = burst_reg - 1;
                        burst_size_next = burst_size_reg;
                        master_burst_next = master_burst_reg;
                        master_burst_size_next = master_burst_size_reg;
                        s_axi_arready_next = !m_axil_arvalid;
                        s_axi_rid_next = id_reg;
                        s_axi_rdata_next = m_axil_rdata >> (addr_reg[AXI_ADDR_BIT_OFFSET:AXIL_ADDR_BIT_OFFSET-1] * AXI_DATA_WIDTH);
                        s_axi_rresp_next = m_axil_rresp;
                        s_axi_rlast_next = 1'b1;
                        s_axi_rvalid_next = 1'b1;
                        m_axil_araddr_next = m_axil_araddr_reg;
                        m_axil_arprot_next = m_axil_arprot_reg;
                        m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
                        m_axil_rready_next = 1'b0;
                    end else if (addr_next[master_burst_size_reg] != addr_reg[master_burst_size_reg]) begin
                        // start new AXI lite read
                        state_next = STATE_DATA_READ;
                        id_next = id_reg;
                        addr_next = addr_reg + (1 << burst_size_reg);
                        data_next = m_axil_rdata;
                        resp_next = m_axil_rresp;
                        burst_next = burst_reg - 1;
                        burst_size_next = burst_size_reg;
                        master_burst_next = master_burst_reg;
                        master_burst_size_next = master_burst_size_reg;
                        s_axi_arready_next = 1'b0;
                        s_axi_rid_next = id_reg;
                        s_axi_rdata_next = m_axil_rdata >> (addr_reg[AXI_ADDR_BIT_OFFSET:AXIL_ADDR_BIT_OFFSET-1] * AXI_DATA_WIDTH);
                        s_axi_rresp_next = m_axil_rresp;
                        s_axi_rlast_next = 1'b0;
                        s_axi_rvalid_next = 1'b1;
                        m_axil_araddr_next = addr_next;
                        m_axil_arprot_next = m_axil_arprot_reg;
                        m_axil_arvalid_next = 1'b1;
                        m_axil_rready_next = 1'b0;
                    end else begin
                        state_next = STATE_DATA_SPLIT;
                        id_next = id_reg;
                        addr_next = addr_reg + (1 << burst_size_reg);
                        data_next = m_axil_rdata;
                        resp_next = m_axil_rresp;
                        burst_next = burst_reg - 1;
                        burst_size_next = burst_size_reg;
                        master_burst_next = master_burst_reg;
                        master_burst_size_next = master_burst_size_reg;
                        s_axi_arready_next = 1'b0;
                        s_axi_rid_next = id_reg;
                        s_axi_rdata_next = m_axil_rdata >> (addr_reg[AXI_ADDR_BIT_OFFSET:AXIL_ADDR_BIT_OFFSET-1] * AXI_DATA_WIDTH);
                        s_axi_rresp_next = m_axil_rresp;
                        s_axi_rlast_next = 1'b0;
                        s_axi_rvalid_next = 1'b1;
                        m_axil_araddr_next = m_axil_araddr_reg;
                        m_axil_arprot_next = m_axil_arprot_reg;
                        m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
                        m_axil_rready_next = 1'b0;
                    end
                end else begin
                    state_next = STATE_DATA_READ;
                    id_next = id_reg;
                    addr_next = addr_reg;
                    data_next = data_reg;
                    resp_next = resp_reg;
                    burst_next = burst_reg;
                    burst_size_next = burst_size_reg;
                    master_burst_next = master_burst_reg;
                    master_burst_size_next = master_burst_size_reg;
                    s_axi_arready_next = 1'b0;
                    s_axi_rid_next = s_axi_rid_reg;
                    s_axi_rdata_next = s_axi_rdata_reg;
                    s_axi_rresp_next = s_axi_rresp_reg;
                    s_axi_rlast_next = s_axi_rlast_reg;
                    s_axi_rvalid_next = s_axi_rvalid_reg && !s_axi_rready;
                    m_axil_araddr_next = m_axil_araddr_reg;
                    m_axil_arprot_next = m_axil_arprot_reg;
                    m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
                    m_axil_rready_next = !s_axi_rvalid && !m_axil_arvalid;
                end
            end
            STATE_DATA_SPLIT: begin
                if (s_axi_rready || !s_axi_rvalid) begin
                    if (burst_reg == 0) begin
                        state_next = STATE_IDLE;
                        id_next = id_reg;
                        addr_next = addr_reg + (1 << burst_size_reg);
                        data_next = data_reg;
                        resp_next = resp_reg;
                        burst_next = burst_reg - 1;
                        burst_size_next = burst_size_reg;
                        master_burst_next = master_burst_reg;
                        master_burst_size_next = master_burst_size_reg;
                        s_axi_arready_next = !m_axil_arvalid;
                        s_axi_rid_next = id_reg;
                        s_axi_rdata_next = data_reg >> (addr_reg[AXI_ADDR_BIT_OFFSET:AXIL_ADDR_BIT_OFFSET-1] * AXI_DATA_WIDTH);
                        s_axi_rresp_next = resp_reg;
                        s_axi_rlast_next = 1'b1;
                        s_axi_rvalid_next = 1'b1;
                        m_axil_araddr_next = m_axil_araddr_reg;
                        m_axil_arprot_next = m_axil_arprot_reg;
                        m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
                        m_axil_rready_next = 1'b0;
                    end else if (addr_next[master_burst_size_reg] != addr_reg[master_burst_size_reg]) begin
                        // start new AXI lite read
                        state_next = STATE_DATA_READ;
                        id_next = id_reg;
                        addr_next = addr_reg + (1 << burst_size_reg);
                        data_next = data_reg;
                        resp_next = resp_reg;
                        burst_next = burst_reg - 1;
                        burst_size_next = burst_size_reg;
                        master_burst_next = master_burst_reg;
                        master_burst_size_next = master_burst_size_reg;
                        s_axi_arready_next = 1'b0;
                        s_axi_rid_next = id_reg;
                        s_axi_rdata_next = data_reg >> (addr_reg[AXI_ADDR_BIT_OFFSET:AXIL_ADDR_BIT_OFFSET-1] * AXI_DATA_WIDTH);
                        s_axi_rresp_next = resp_reg;
                        s_axi_rlast_next = 1'b0;
                        s_axi_rvalid_next = 1'b1;
                        m_axil_araddr_next = addr_next;
                        m_axil_arprot_next = m_axil_arprot_reg;
                        m_axil_arvalid_next = 1'b1;
                        m_axil_rready_next = 1'b0;
                    end else begin
                        state_next = STATE_DATA_SPLIT;
                        id_next = id_reg;
                        addr_next = addr_reg + (1 << burst_size_reg);
                        data_next = data_reg;
                        resp_next = resp_reg;
                        burst_next = burst_reg - 1;
                        burst_size_next = burst_size_reg;
                        master_burst_next = master_burst_reg;
                        master_burst_size_next = master_burst_size_reg;
                        s_axi_arready_next = 1'b0;
                        s_axi_rid_next = id_reg;
                        s_axi_rdata_next = data_reg >> (addr_reg[AXI_ADDR_BIT_OFFSET:AXIL_ADDR_BIT_OFFSET-1] * AXI_DATA_WIDTH);
                        s_axi_rresp_next = resp_reg;
                        s_axi_rlast_next = 1'b0;
                        s_axi_rvalid_next = 1'b1;
                        m_axil_araddr_next = m_axil_araddr_reg;
                        m_axil_arprot_next = m_axil_arprot_reg;
                        m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
                        m_axil_rready_next = 1'b0;
                    end
                end else begin
                    state_next = STATE_DATA_SPLIT;
                    id_next = id_reg;
                    addr_next = addr_reg;
                    data_next = data_reg;
                    resp_next = resp_reg;
                    burst_next = burst_reg;
                    burst_size_next = burst_size_reg;
                    master_burst_next = master_burst_reg;
                    master_burst_size_next = master_burst_size_reg;
                    s_axi_arready_next = 1'b0;
                    s_axi_rid_next = s_axi_rid_reg;
                    s_axi_rdata_next = s_axi_rdata_reg;
                    s_axi_rresp_next = s_axi_rresp_reg;
                    s_axi_rlast_next = s_axi_rlast_reg;
                    s_axi_rvalid_next = s_axi_rvalid_reg && !s_axi_rready;
                    m_axil_araddr_next = m_axil_araddr_reg;
                    m_axil_arprot_next = m_axil_arprot_reg;
                    m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
                    m_axil_rready_next = 1'b0;
                end
            end
        endcase
    end else begin
        // master output is narrower; merge reads and possibly split burst
        case (state_reg)
            STATE_IDLE: begin
                // idle state; wait for new burst
                if (s_axi_arready && s_axi_arvalid) begin
                    state_next = STATE_DATA;
                    id_next = s_axi_arid;
                    addr_next = s_axi_araddr;
                    data_next = data_reg;
                    resp_next = 2'd0;
                    burst_next = s_axi_arlen;
                    burst_size_next = s_axi_arsize;
                    s_axi_arready_next = 1'b0;
                    s_axi_rid_next = s_axi_rid_reg;
                    s_axi_rdata_next = s_axi_rdata_reg;
                    s_axi_rresp_next = s_axi_rresp_reg;
                    s_axi_rlast_next = s_axi_rlast_reg;
                    s_axi_rvalid_next = s_axi_rvalid_reg && !s_axi_rready;
                    m_axil_araddr_next = s_axi_araddr;
                    m_axil_arprot_next = s_axi_arprot;
                    m_axil_arvalid_next = 1'b1;
                    m_axil_rready_next = 1'b0;
                    if (s_axi_arsize > AXIL_BURST_SIZE) begin
                        // need to adjust burst size
                        if (s_axi_arlen >> (8+AXIL_BURST_SIZE-s_axi_arsize) != 0) begin
                            // limit burst length to max
                            master_burst_next = (8'd255 << (s_axi_arsize-AXIL_BURST_SIZE)) | ((~s_axi_araddr & (8'hff >> (8-s_axi_arsize))) >> AXIL_BURST_SIZE);
                        end else begin
                            master_burst_next = (s_axi_arlen << (s_axi_arsize-AXIL_BURST_SIZE)) | ((~s_axi_araddr & (8'hff >> (8-s_axi_arsize))) >> AXIL_BURST_SIZE);
                        end
                        master_burst_size_next = AXIL_BURST_SIZE;
                    end else begin
                        // pass through narrow (enough) burst
                        master_burst_next = s_axi_arlen;
                        master_burst_size_next = s_axi_arsize;
                    end
                end else begin
                    state_next = STATE_IDLE;
                    id_next = id_reg;
                    addr_next = addr_reg;
                    data_next = data_reg;
                    resp_next = resp_reg;
                    burst_next = burst_reg;
                    burst_size_next = burst_size_reg;
                    master_burst_next = master_burst_reg;
                    master_burst_size_next = master_burst_size_reg;
                    s_axi_arready_next = !m_axil_arvalid;
                    s_axi_rid_next = s_axi_rid_reg;
                    s_axi_rdata_next = s_axi_rdata_reg;
                    s_axi_rresp_next = s_axi_rresp_reg;
                    s_axi_rlast_next = s_axi_rlast_reg;
                    s_axi_rvalid_next = s_axi_rvalid_reg && !s_axi_rready;
                    m_axil_araddr_next = m_axil_araddr_reg;
                    m_axil_arprot_next = m_axil_arprot_reg;
                    m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
                    m_axil_rready_next = 1'b0;
                end
            end
            STATE_DATA: begin
                if (m_axil_rready && m_axil_rvalid) begin
                    if (m_axil_rresp) begin
                        resp_next = m_axil_rresp;
                    end else begin
                        resp_next = resp_reg;
                    end
                    id_next = id_reg;
                    addr_next = (addr_reg + (1 << master_burst_size_reg)) & ({ADDR_WIDTH{1'b1}} << master_burst_size_reg);
                    burst_size_next = burst_size_reg;
                    master_burst_size_next = master_burst_size_reg;
                    s_axi_rid_next = id_reg;
                    s_axi_rresp_next = resp_next;
                    m_axil_araddr_next = addr_next;
                    m_axil_arprot_next = m_axil_arprot_reg;
                    m_axil_rready_next = 1'b0;
                    
                    if (master_burst_reg == 0) begin
                        if (burst_reg == 0) begin
                            state_next = STATE_IDLE;
                            s_axi_arready_next = !m_axil_arvalid;
                            s_axi_rlast_next = 1'b1;
                            s_axi_rvalid_next = 1'b1;
                            m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
                            if (addr_next[burst_size_reg] != addr_reg[burst_size_reg]) begin
                                data_next = {DATA_WIDTH{1'b0}};
                                s_axi_rdata_next = (data_reg & ~({{DATA_WIDTH-SEGMENT_DATA_WIDTH{1'b0}}, {SEGMENT_DATA_WIDTH{1'b1}}} << (addr_reg[AXIL_ADDR_BIT_OFFSET:AXI_ADDR_BIT_OFFSET-1]*SEGMENT_DATA_WIDTH))) |
                                                  ({{DATA_WIDTH-SEGMENT_DATA_WIDTH{1'b0}}, m_axil_rdata} << (addr_reg[AXIL_ADDR_BIT_OFFSET:AXI_ADDR_BIT_OFFSET-1]*SEGMENT_DATA_WIDTH));
                                burst_next = burst_reg - 1;
                            end else begin
                                data_next = (data_reg & ~({{DATA_WIDTH-SEGMENT_DATA_WIDTH{1'b0}}, {SEGMENT_DATA_WIDTH{1'b1}}} << (addr_reg[AXIL_ADDR_BIT_OFFSET:AXI_ADDR_BIT_OFFSET-1]*SEGMENT_DATA_WIDTH))) |
                                           ({{DATA_WIDTH-SEGMENT_DATA_WIDTH{1'b0}}, m_axil_rdata} << (addr_reg[AXIL_ADDR_BIT_OFFSET:AXI_ADDR_BIT_OFFSET-1]*SEGMENT_DATA_WIDTH));
                                s_axi_rdata_next = data_next;
                                burst_next = burst_reg;
                            end
                            if (burst_next >> (8+AXIL_BURST_SIZE-burst_size_reg) != 0) begin
                                master_burst_next = 8'd255;
                            end else begin
                                master_burst_next = (burst_next << (burst_size_reg-AXIL_BURST_SIZE)) | (8'hff >> (8-burst_size_reg) >> AXIL_BURST_SIZE);
                            end
                        end else begin
                            state_next = STATE_DATA;
                            s_axi_arready_next = 1'b0;
                            m_axil_arvalid_next = 1'b1;
                            if (addr_next[burst_size_reg] != addr_reg[burst_size_reg]) begin
                                data_next = {DATA_WIDTH{1'b0}};
                                s_axi_rdata_next = (data_reg & ~({{DATA_WIDTH-SEGMENT_DATA_WIDTH{1'b0}}, {SEGMENT_DATA_WIDTH{1'b1}}} << (addr_reg[AXIL_ADDR_BIT_OFFSET:AXI_ADDR_BIT_OFFSET-1]*SEGMENT_DATA_WIDTH))) |
                                                  ({{DATA_WIDTH-SEGMENT_DATA_WIDTH{1'b0}}, m_axil_rdata} << (addr_reg[AXIL_ADDR_BIT_OFFSET:AXI_ADDR_BIT_OFFSET-1]*SEGMENT_DATA_WIDTH));
                                burst_next = burst_reg - 1;
                                s_axi_rlast_next = 1'b0;
                                s_axi_rvalid_next = 1'b1;
                            end else begin
                                data_next = (data_reg & ~({{DATA_WIDTH-SEGMENT_DATA_WIDTH{1'b0}}, {SEGMENT_DATA_WIDTH{1'b1}}} << (addr_reg[AXIL_ADDR_BIT_OFFSET:AXI_ADDR_BIT_OFFSET-1]*SEGMENT_DATA_WIDTH))) |
                                           ({{DATA_WIDTH-SEGMENT_DATA_WIDTH{1'b0}}, m_axil_rdata} << (addr_reg[AXIL_ADDR_BIT_OFFSET:AXI_ADDR_BIT_OFFSET-1]*SEGMENT_DATA_WIDTH));
                                s_axi_rdata_next = data_next;
                                burst_next = burst_reg;
                                s_axi_rlast_next = 1'b0;
                                s_axi_rvalid_next = 1'b0;
                            end
                            if (burst_next >> (8+AXIL_BURST_SIZE-burst_size_reg) != 0) begin
                                master_burst_next = 8'd255;
                            end else begin
                                master_burst_next = (burst_next << (burst_size_reg-AXIL_BURST_SIZE)) | (8'hff >> (8-burst_size_reg) >> AXIL_BURST_SIZE);
                            end
                        end
                    end else begin
                        state_next = STATE_DATA;
                        s_axi_arready_next = 1'b0;
                        m_axil_arvalid_next = 1'b1;
                        master_burst_next = master_burst_reg - 1;
                        if (addr_next[burst_size_reg] != addr_reg[burst_size_reg]) begin
                            data_next = {DATA_WIDTH{1'b0}};
                            s_axi_rdata_next = (data_reg & ~({{DATA_WIDTH-SEGMENT_DATA_WIDTH{1'b0}}, {SEGMENT_DATA_WIDTH{1'b1}}} << (addr_reg[AXIL_ADDR_BIT_OFFSET:AXI_ADDR_BIT_OFFSET-1]*SEGMENT_DATA_WIDTH))) |
                                              ({{DATA_WIDTH-SEGMENT_DATA_WIDTH{1'b0}}, m_axil_rdata} << (addr_reg[AXIL_ADDR_BIT_OFFSET:AXI_ADDR_BIT_OFFSET-1]*SEGMENT_DATA_WIDTH));
                            burst_next = burst_reg - 1;
                            s_axi_rlast_next = 1'b0;
                            s_axi_rvalid_next = 1'b1;
                        end else begin
                            data_next = (data_reg & ~({{DATA_WIDTH-SEGMENT_DATA_WIDTH{1'b0}}, {SEGMENT_DATA_WIDTH{1'b1}}} << (addr_reg[AXIL_ADDR_BIT_OFFSET:AXI_ADDR_BIT_OFFSET-1]*SEGMENT_DATA_WIDTH))) |
                                       ({{DATA_WIDTH-SEGMENT_DATA_WIDTH{1'b0}}, m_axil_rdata} << (addr_reg[AXIL_ADDR_BIT_OFFSET:AXI_ADDR_BIT_OFFSET-1]*SEGMENT_DATA_WIDTH));
                            s_axi_rdata_next = data_next;
                            burst_next = burst_reg;
                            s_axi_rlast_next = 1'b0;
                            s_axi_rvalid_next = 1'b0;
                        end
                    end
                    if (master_burst_reg == 0) begin
                        if (burst_next >> (8+AXIL_BURST_SIZE-burst_size_reg) != 0) begin
                            // limit burst length to max
                            master_burst_next = 8'd255;
                        end else begin
                            master_burst_next = (burst_next << (burst_size_reg-AXIL_BURST_SIZE)) | (8'hff >> (8-burst_size_reg) >> AXIL_BURST_SIZE);
                        end
                        if (burst_reg == 0) begin
                            state_next = STATE_IDLE;
                            s_axi_rlast_next = 1'b1;
                            s_axi_rvalid_next = 1'b1;
                            s_axi_arready_next = !m_axil_arvalid;
                            m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
                        end else begin
                            state_next = STATE_DATA;
                            m_axil_arvalid_next = 1'b1;
                        end
                    end else begin
                        state_next = STATE_DATA;
                        m_axil_arvalid_next = 1'b1;
                    end
                end else begin
                    state_next = STATE_DATA;
                    id_next = id_reg;
                    addr_next = addr_reg;
                    data_next = data_reg;
                    resp_next = resp_reg;
                    burst_next = burst_reg;
                    burst_size_next = burst_size_reg;
                    master_burst_next = master_burst_reg;
                    master_burst_size_next = master_burst_size_reg;
                    s_axi_arready_next = 1'b0;
                    s_axi_rid_next = s_axi_rid_reg;
                    s_axi_rdata_next = s_axi_rdata_reg;
                    s_axi_rresp_next = s_axi_rresp_reg;
                    s_axi_rlast_next = s_axi_rlast_reg;
                    s_axi_rvalid_next = s_axi_rvalid_reg && !s_axi_rready;
                    m_axil_araddr_next = m_axil_araddr_reg;
                    m_axil_arprot_next = m_axil_arprot_reg;
                    m_axil_arvalid_next = m_axil_arvalid_reg && !m_axil_arready;
                    m_axil_rready_next = !s_axi_rvalid && !m_axil_arvalid;
                end
            end
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state_reg <= STATE_IDLE;

        id_reg <= 0;
        addr_reg <= 0;
        data_reg <= 0;
        resp_reg <= 0;
        burst_reg <= 0;
        burst_size_reg <= 0;
        master_burst_reg <= 0;
        master_burst_size_reg <= 0;

        s_axi_arready_reg <= 0;
        s_axi_rid_reg <= 0;
        s_axi_rdata_reg <= 0;
        s_axi_rresp_reg <= 0;
        s_axi_rlast_reg <= 0;
        s_axi_rvalid_reg <= 0;

        m_axil_araddr_reg <= 0;
        m_axil_arprot_reg <= 0;
        m_axil_arvalid_reg <= 0;
        m_axil_rready_reg <= 0;
    end else begin
        state_reg <= state_next;

        id_reg <= id_next;
        addr_reg <= addr_next;
        data_reg <= data_next;
        resp_reg <= resp_next;
        burst_reg <= burst_next;
        burst_size_reg <= burst_size_next;
        master_burst_reg <= master_burst_next;
        master_burst_size_reg <= master_burst_size_next;

        s_axi_arready_reg <= s_axi_arready_next;
        s_axi_rid_reg <= s_axi_rid_next;
        s_axi_rdata_reg <= s_axi_rdata_next;
        s_axi_rresp_reg <= s_axi_rresp_next;
        s_axi_rlast_reg <= s_axi_rlast_next;
        s_axi_rvalid_reg <= s_axi_rvalid_next;

        m_axil_araddr_reg <= m_axil_araddr_next;
        m_axil_arprot_reg <= m_axil_arprot_next;
        m_axil_arvalid_reg <= m_axil_arvalid_next;
        m_axil_rready_reg <= m_axil_rready_next;
    end
end

endmodule

`resetall

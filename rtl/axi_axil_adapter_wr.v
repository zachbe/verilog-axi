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
 * AXI4 to AXI4-Lite adapter (write)
 */
module axi_axil_adapter_wr #
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
    input  wire [AXI_ID_WIDTH-1:0]     s_axi_awid,
    input  wire [ADDR_WIDTH-1:0]       s_axi_awaddr,
    input  wire [7:0]                  s_axi_awlen,
    input  wire [2:0]                  s_axi_awsize,
    input  wire [1:0]                  s_axi_awburst,
    input  wire                        s_axi_awlock,
    input  wire [3:0]                  s_axi_awcache,
    input  wire [2:0]                  s_axi_awprot,
    input  wire                        s_axi_awvalid,
    output wire                        s_axi_awready,
    input  wire [AXI_DATA_WIDTH-1:0]   s_axi_wdata,
    input  wire [AXI_STRB_WIDTH-1:0]   s_axi_wstrb,
    input  wire                        s_axi_wlast,
    input  wire                        s_axi_wvalid,
    output wire                        s_axi_wready,
    output wire [AXI_ID_WIDTH-1:0]     s_axi_bid,
    output wire [1:0]                  s_axi_bresp,
    output wire                        s_axi_bvalid,
    input  wire                        s_axi_bready,

    /*
     * AXI lite master interface
     */
    output wire [ADDR_WIDTH-1:0]       m_axil_awaddr,
    output wire [2:0]                  m_axil_awprot,
    output wire                        m_axil_awvalid,
    input  wire                        m_axil_awready,
    output wire [AXIL_DATA_WIDTH-1:0]  m_axil_wdata,
    output wire [AXIL_STRB_WIDTH-1:0]  m_axil_wstrb,
    output wire                        m_axil_wvalid,
    input  wire                        m_axil_wready,
    input  wire [1:0]                  m_axil_bresp,
    input  wire                        m_axil_bvalid,
    output wire                        m_axil_bready
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
    STATE_DATA_2 = 2'd2,
    STATE_RESP = 2'd3;

reg [1:0] state_reg = STATE_IDLE, state_next;

reg [AXI_ID_WIDTH-1:0] id_reg = {AXI_ID_WIDTH{1'b0}}, id_next;
reg [ADDR_WIDTH-1:0] addr_reg = {ADDR_WIDTH{1'b0}}, addr_next;
reg [DATA_WIDTH-1:0] data_reg = {DATA_WIDTH{1'b0}}, data_next;
reg [STRB_WIDTH-1:0] strb_reg = {STRB_WIDTH{1'b0}}, strb_next;
reg [7:0] burst_reg = 8'd0, burst_next;
reg [2:0] burst_size_reg = 3'd0, burst_size_next;
reg [2:0] master_burst_size_reg = 3'd0, master_burst_size_next;
reg burst_active_reg = 1'b0, burst_active_next;
reg convert_burst_reg = 1'b0, convert_burst_next;
reg first_transfer_reg = 1'b0, first_transfer_next;
reg last_segment_reg = 1'b0, last_segment_next;

reg s_axi_awready_reg = 1'b0, s_axi_awready_next;
reg s_axi_wready_reg = 1'b0, s_axi_wready_next;
reg [AXI_ID_WIDTH-1:0] s_axi_bid_reg = {AXI_ID_WIDTH{1'b0}}, s_axi_bid_next;
reg [1:0] s_axi_bresp_reg = 2'd0, s_axi_bresp_next;
reg s_axi_bvalid_reg = 1'b0, s_axi_bvalid_next;

reg [ADDR_WIDTH-1:0] m_axil_awaddr_reg = {ADDR_WIDTH{1'b0}}, m_axil_awaddr_next;
reg [2:0] m_axil_awprot_reg = 3'd0, m_axil_awprot_next;
reg m_axil_awvalid_reg = 1'b0, m_axil_awvalid_next;
reg [AXIL_DATA_WIDTH-1:0] m_axil_wdata_reg = {AXIL_DATA_WIDTH{1'b0}}, m_axil_wdata_next;
reg [AXIL_STRB_WIDTH-1:0] m_axil_wstrb_reg = {AXIL_STRB_WIDTH{1'b0}}, m_axil_wstrb_next;
reg m_axil_wvalid_reg = 1'b0, m_axil_wvalid_next;
reg m_axil_bready_reg = 1'b0, m_axil_bready_next;

assign s_axi_awready = s_axi_awready_reg;
assign s_axi_wready = s_axi_wready_reg;
assign s_axi_bid = s_axi_bid_reg;
assign s_axi_bresp = s_axi_bresp_reg;
assign s_axi_bvalid = s_axi_bvalid_reg;

assign m_axil_awaddr = m_axil_awaddr_reg;
//assign m_axil_awlen = m_axil_awlen_reg;
//assign m_axil_awsize = m_axil_awsize_reg;
//assign m_axil_awburst = m_axil_awburst_reg;
assign m_axil_awprot = m_axil_awprot_reg;
assign m_axil_awvalid = m_axil_awvalid_reg;
assign m_axil_wdata = m_axil_wdata_reg;
assign m_axil_wstrb = m_axil_wstrb_reg;
assign m_axil_wvalid = m_axil_wvalid_reg;
assign m_axil_bready = m_axil_bready_reg;

integer i;

// NOTE: This refactor assumes that we only do direct transfers.
// It was SUCH a pain to do the RD path so I'm just simplifying this.
always @* begin
    // master output is same width; direct transfer with no splitting/merging
    case (state_reg)
        STATE_IDLE: begin
            // idle state; wait for new burst
            first_transfer_next = 1'b1;
            if (s_axi_awready && s_axi_awvalid) begin
                state_next = STATE_DATA;
                id_next = s_axi_awid;
                addr_next = s_axi_awaddr;
                data_next = data_reg;
                strb_next = strb_reg;
                burst_next = s_axi_awlen;
                burst_size_next = s_axi_awsize;
                master_burst_size_next = master_burst_size_reg;
                burst_active_next = 1'b1;
                convert_burst_next = convert_burst_reg;
                last_segment_next = last_segment_reg;
                s_axi_awready_next = 1'b0;
                s_axi_wready_next = !m_axil_wvalid;
                s_axi_bid_next = s_axi_bid_reg;
                s_axi_bresp_next = s_axi_bresp_reg;
                s_axi_bvalid_next = s_axi_bvalid_reg && !s_axi_bready;
                m_axil_awaddr_next = s_axi_awaddr;
                m_axil_awprot_next = s_axi_awprot;
                m_axil_awvalid_next = 1'b1;
                m_axil_wdata_next = m_axil_wdata_reg;
                m_axil_wstrb_next = m_axil_wstrb_reg;
                m_axil_wvalid_next = m_axil_wvalid_reg && !m_axil_wready;
                m_axil_bready_next = 1'b0;
            end else begin
                state_next = STATE_IDLE;
                id_next = id_reg;
                addr_next = addr_reg;
                data_next = data_reg;
                strb_next = strb_reg;
                burst_next = burst_reg;
                burst_size_next = burst_size_reg;
                master_burst_size_next = master_burst_size_reg;
                burst_active_next = burst_active_reg;
                convert_burst_next = convert_burst_reg;
                last_segment_next = last_segment_reg;
                s_axi_awready_next = !m_axil_awvalid;
                s_axi_wready_next = 1'b0;
                s_axi_bid_next = s_axi_bid_reg;
                s_axi_bresp_next = s_axi_bresp_reg;
                s_axi_bvalid_next = s_axi_bvalid_reg && !s_axi_bready;
                m_axil_awaddr_next = m_axil_awaddr_reg;
                m_axil_awprot_next = m_axil_awprot_reg;
                m_axil_awvalid_next = m_axil_awvalid_reg && !m_axil_awready;
                m_axil_wdata_next = m_axil_wdata_reg;
                m_axil_wstrb_next = m_axil_wstrb_reg;
                m_axil_wvalid_next = m_axil_wvalid_reg && !m_axil_wready;
                m_axil_bready_next = 1'b0;
            end
        end
        STATE_DATA: begin
            // data state; transfer write data
            if (s_axi_wready && s_axi_wvalid) begin
                state_next = STATE_RESP;
                id_next = id_reg;
                addr_next = addr_reg + (1 << burst_size_reg);
                data_next = data_reg;
                strb_next = strb_reg;
                burst_next = burst_reg - 1;
                burst_size_next = burst_size_reg;
                master_burst_size_next = master_burst_size_reg;
                burst_active_next = burst_reg != 0;
                convert_burst_next = convert_burst_reg;
                first_transfer_next = first_transfer_reg;
                last_segment_next = last_segment_reg;
                s_axi_awready_next = 1'b0;
                s_axi_wready_next = 1'b0;
                s_axi_bid_next = s_axi_bid_reg;
                s_axi_bresp_next = s_axi_bresp_reg;
                s_axi_bvalid_next = s_axi_bvalid_reg && !s_axi_bready;
                m_axil_awaddr_next = m_axil_awaddr_reg;
                m_axil_awprot_next = m_axil_awprot_reg;
                m_axil_awvalid_next = m_axil_awvalid_reg && !m_axil_awready;
                m_axil_wdata_next = s_axi_wdata;
                m_axil_wstrb_next = s_axi_wstrb;
                m_axil_wvalid_next = 1'b1;
                m_axil_bready_next = !s_axi_bvalid && !m_axil_awvalid;
            end else begin
                state_next = STATE_DATA;
                id_next = id_reg;
                addr_next = addr_reg;
                data_next = data_reg;
                strb_next = strb_reg;
                burst_next = burst_reg;
                burst_size_next = burst_size_reg;
                master_burst_size_next = master_burst_size_reg;
                burst_active_next = burst_active_reg;
                convert_burst_next = convert_burst_reg;
                first_transfer_next = first_transfer_reg;
                last_segment_next = last_segment_reg;
                s_axi_awready_next = 1'b0;
                s_axi_wready_next = !m_axil_wvalid;
                s_axi_bid_next = s_axi_bid_reg;
                s_axi_bresp_next = s_axi_bresp_reg;
                s_axi_bvalid_next = s_axi_bvalid_reg && !s_axi_bready;
                m_axil_awaddr_next = m_axil_awaddr_reg;
                m_axil_awprot_next = m_axil_awprot_reg;
                m_axil_awvalid_next = m_axil_awvalid_reg && !m_axil_awready;
                m_axil_wdata_next = m_axil_wdata_reg;
                m_axil_wstrb_next = m_axil_wstrb_reg;
                m_axil_wvalid_next = m_axil_wvalid_reg && !m_axil_wready;
                m_axil_bready_next = 1'b0;
            end
        end
        STATE_RESP: begin
            // resp state; transfer write response
            if (m_axil_bready && m_axil_bvalid) begin
                id_next = id_reg;
                addr_next = addr_reg;
                data_next = data_reg;
                strb_next = strb_reg;
                burst_next = burst_reg;
                burst_size_next = burst_size_reg;
                master_burst_size_next = master_burst_size_reg;
                burst_active_next = burst_active_reg;
                convert_burst_next = convert_burst_reg;
                first_transfer_next = 1'b0;
                last_segment_next = last_segment_reg;
                s_axi_bid_next = id_reg;
                if (first_transfer_reg || m_axil_bresp != 0) begin
                    s_axi_bresp_next = m_axil_bresp;
                end else begin
                    s_axi_bresp_next = s_axi_bresp_reg;
                end
                m_axil_bready_next = 1'b0;
                if (burst_active_reg) begin
                    // burst on slave interface still active; start new AXI lite write
                    state_next = STATE_DATA;
                    s_axi_awready_next = 1'b0;
                    s_axi_wready_next = !m_axil_wvalid;
                    s_axi_bvalid_next = s_axi_bvalid_reg && !s_axi_bready;
                    m_axil_awaddr_next = addr_reg;
                    m_axil_awprot_next = m_axil_awprot_reg;
                    m_axil_awvalid_next = 1'b1;
                    m_axil_wdata_next = m_axil_wdata_reg;
                    m_axil_wstrb_next = m_axil_wstrb_reg;
                    m_axil_wvalid_next = m_axil_wvalid_reg && !m_axil_wready;
                end else begin
                    // burst on slave interface finished; return to idle
                    state_next = STATE_IDLE;
                    s_axi_awready_next = !m_axil_awvalid;
                    s_axi_wready_next = 1'b0;
                    s_axi_bvalid_next = 1'b1;
                    m_axil_awaddr_next = m_axil_awaddr_reg;
                    m_axil_awprot_next = m_axil_awprot_reg;
                    m_axil_awvalid_next = m_axil_awvalid_reg && !m_axil_awready;
                    m_axil_wdata_next = m_axil_wdata_reg;
                    m_axil_wstrb_next = m_axil_wstrb_reg;
                    m_axil_wvalid_next = m_axil_wvalid_reg && !m_axil_wready;
                end
            end else begin
                state_next = STATE_RESP;
                id_next = id_reg;
                addr_next = addr_reg;
                data_next = data_reg;
                strb_next = strb_reg;
                burst_next = burst_reg;
                burst_size_next = burst_size_reg;
                master_burst_size_next = master_burst_size_reg;
                burst_active_next = burst_active_reg;
                convert_burst_next = convert_burst_reg;
                first_transfer_next = first_transfer_reg;
                last_segment_next = last_segment_reg;
                s_axi_awready_next = 1'b0;
                s_axi_wready_next = 1'b0;
                s_axi_bid_next = s_axi_bid_reg;
                s_axi_bresp_next = s_axi_bresp_reg;
                s_axi_bvalid_next = s_axi_bvalid_reg && !s_axi_bready;
                m_axil_awaddr_next = m_axil_awaddr_reg;
                m_axil_awprot_next = m_axil_awprot_reg;
                m_axil_awvalid_next = m_axil_awvalid_reg && !m_axil_awready;
                m_axil_wdata_next = m_axil_wdata_reg;
                m_axil_wstrb_next = m_axil_wstrb_reg;
                m_axil_wvalid_next = m_axil_wvalid_reg && !m_axil_wready;
                m_axil_bready_next = !s_axi_bvalid && !m_axil_awvalid;
            end
        end
    endcase
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state_reg <= STATE_IDLE;

        id_reg <= 0;
        addr_reg <= 0;
        data_reg <= 0;
        strb_reg <= 0;
        burst_reg <= 0;
        burst_size_reg <= 0;
        master_burst_size_reg <= 0;
        burst_active_reg <= 0;
        convert_burst_reg <= 0;
        first_transfer_reg <= 0;
        last_segment_reg <= 0;

        s_axi_awready_reg <= 0;
        s_axi_wready_reg <= 0;
        s_axi_bid_reg <= 0;
        s_axi_bresp_reg <= 0;
        s_axi_bvalid_reg <= 0;

        m_axil_awaddr_reg <= 0;
        m_axil_awprot_reg <= 0;
        m_axil_awvalid_reg <= 0;
        m_axil_wdata_reg <= 0;
        m_axil_wstrb_reg <= 0;
        m_axil_wvalid_reg <= 0;
        m_axil_bready_reg <= 0;
    end else begin
        state_reg <= state_next;

        id_reg <= id_next;
        addr_reg <= addr_next;
        data_reg <= data_next;
        strb_reg <= strb_next;
        burst_reg <= burst_next;
        burst_size_reg <= burst_size_next;
        master_burst_size_reg <= master_burst_size_next;
        burst_active_reg <= burst_active_next;
        convert_burst_reg <= convert_burst_next;
        first_transfer_reg <= first_transfer_next;
        last_segment_reg <= last_segment_next;

        s_axi_awready_reg <= s_axi_awready_next;
        s_axi_wready_reg <= s_axi_wready_next;
        s_axi_bid_reg <= s_axi_bid_next;
        s_axi_bresp_reg <= s_axi_bresp_next;
        s_axi_bvalid_reg <= s_axi_bvalid_next;

        m_axil_awaddr_reg <= m_axil_awaddr_next;
        m_axil_awprot_reg <= m_axil_awprot_next;
        m_axil_awvalid_reg <= m_axil_awvalid_next;
        m_axil_wdata_reg <= m_axil_wdata_next;
        m_axil_wstrb_reg <= m_axil_wstrb_next;
        m_axil_wvalid_reg <= m_axil_wvalid_next;
        m_axil_bready_reg <= m_axil_bready_next;
    end
end

endmodule

`resetall

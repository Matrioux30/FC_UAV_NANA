// MPC accelerator top: AXI4-Lite slave + mpc_core. Plain Verilog-2001 so it can be a Vivado BD module reference.
//
// Register map (byte offsets, 32-bit words; partial-word writes are treated as full-word writes):
//   0x000 CTRL    W   bit0 START (write 1, ignored while busy)       R  bit1 IRQ_EN
//                 RW  bit1 IRQ_EN  (irq = DONE & IRQ_EN)
//   0x004 STATUS  R   bit0 BUSY, bit1 DONE (sticky, cleared by START), bit2 SAT (saturation in last solve)
//   0x008 N_ITER  RW  ADMM iterations per solve (default N_ITER_DEFAULT)
//   0x00C CYCLES  R   clock cycles of the last solve (start .. done)
//   0x010 ID      R   0x4D504331 ("MPC1")
//   0x014 CONFIG  R   [7:0] FRAC_W, [15:8] FRAC_MX, [23:16] FRAC_MR, [31:24] FRAC_P  (identifies the ROM format)
//   0x400 + 4k    RW  input buffer, k = 0..91: x0..x11 (k = 0..11), r0..r79 (k = 12..91); writes ignored while busy
//   0x800 + 4i    R   z[i], i = 0..19  (u*(k) uses z[0..3])
//   0x900 + 4i    R   g[i], i = 0..19  (debug)
module mpc_accel_axi #(
    parameter C_S_AXI_ADDR_WIDTH = 12,
    parameter FRAC_MX        = 27,
    parameter FRAC_MR        = 31,
    parameter FRAC_W         = 29,
    parameter FRAC_P         = 30,
    parameter N_ITER_DEFAULT = 80,
    parameter COEF_FILE      = "coef_col.mem",
    parameter ZMIN_FILE      = "zmin.mem",
    parameter ZMAX_FILE      = "zmax.mem",
    parameter PARAM_FILE     = "params.mem"
) (
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET s_axi_aresetn" *)
    input  wire                          s_axi_aclk,
    input  wire                          s_axi_aresetn,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
    input  wire [2:0]                    s_axi_awprot,
    input  wire                          s_axi_awvalid,
    output wire                          s_axi_awready,
    input  wire [31:0]                   s_axi_wdata,
    input  wire [3:0]                    s_axi_wstrb,
    input  wire                          s_axi_wvalid,
    output wire                          s_axi_wready,
    output wire [1:0]                    s_axi_bresp,
    output reg                           s_axi_bvalid,
    input  wire                          s_axi_bready,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_araddr,
    input  wire [2:0]                    s_axi_arprot,
    input  wire                          s_axi_arvalid,
    output wire                          s_axi_arready,
    output reg  [31:0]                   s_axi_rdata,
    output wire [1:0]                    s_axi_rresp,
    output reg                           s_axi_rvalid,
    input  wire                          s_axi_rready,
    (* X_INTERFACE_INFO = "xilinx.com:signal:interrupt:1.0 irq INTERRUPT" *)
    (* X_INTERFACE_PARAMETER = "SENSITIVITY LEVEL_HIGH" *)
    output wire                          irq
);
    localparam NL = 20;
    localparam [31:0] ID_VALUE = 32'h4D504331;

    wire clk = s_axi_aclk;
    wire rst = !s_axi_aresetn;

    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;

    // ---------------------------------------------------------------- write channel
    reg                          aw_ok, w_ok;
    reg [C_S_AXI_ADDR_WIDTH-1:0] aw_addr;
    reg [31:0]                   w_data;

    assign s_axi_awready = !aw_ok && !s_axi_bvalid;
    assign s_axi_wready  = !w_ok && !s_axi_bvalid;

    wire        wr_en   = aw_ok && w_ok;
    wire [9:0]  wr_word = aw_addr[11:2];

    always @(posedge clk) begin
        if (rst) begin
            aw_ok        <= 1'b0;
            w_ok         <= 1'b0;
            s_axi_bvalid <= 1'b0;
        end else begin
            if (s_axi_awvalid && s_axi_awready) begin
                aw_ok   <= 1'b1;
                aw_addr <= s_axi_awaddr;
            end
            if (s_axi_wvalid && s_axi_wready) begin
                w_ok   <= 1'b1;
                w_data <= s_axi_wdata;
            end
            if (wr_en) begin
                aw_ok        <= 1'b0;
                w_ok         <= 1'b0;
                s_axi_bvalid <= 1'b1;
            end
            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
        end
    end

    // ---------------------------------------------------------------- control registers
    wire        busy, done_pulse, sat_flag;
    wire [31:0] cycles;
    reg         irq_en, done_sticky;
    reg  [15:0] n_iter;

    wire start = wr_en && wr_word == 10'h000 && w_data[0] && !busy;

    always @(posedge clk) begin
        if (rst) begin
            irq_en      <= 1'b0;
            done_sticky <= 1'b0;
            n_iter      <= N_ITER_DEFAULT;
        end else begin
            if (wr_en && wr_word == 10'h000) irq_en <= w_data[1];
            if (wr_en && wr_word == 10'h002 && !busy) n_iter <= w_data[15:0];
            if (start) done_sticky <= 1'b0;
            else if (done_pulse) done_sticky <= 1'b1;
        end
    end

    assign irq = done_sticky && irq_en;

    // ---------------------------------------------------------------- core
    wire [NL*32-1:0] z_flat, g_flat;
    wire [31:0]      in_rdata;

    mpc_core #(
        .NL(NL), .FRAC_MX(FRAC_MX), .FRAC_MR(FRAC_MR), .FRAC_W(FRAC_W), .FRAC_P(FRAC_P),
        .COEF_FILE(COEF_FILE), .ZMIN_FILE(ZMIN_FILE), .ZMAX_FILE(ZMAX_FILE), .PARAM_FILE(PARAM_FILE)
    ) u_core (
        .clk(clk),
        .rst(rst),
        .start(start),
        .n_iter(n_iter),
        .busy(busy),
        .done(done_pulse),
        .sat_flag(sat_flag),
        .cycles(cycles),
        .in_we(wr_en && wr_word[9:7] == 3'b010 && !busy),             // 0x400 .. 0x5FC
        .in_waddr(wr_word[6:0]),
        .in_wdata(w_data),
        .in_raddr(s_axi_araddr[8:2]),
        .in_rdata(in_rdata),
        .z_flat(z_flat),
        .g_flat(g_flat)
    );

    // ---------------------------------------------------------------- read channel
    wire [9:0] rd_word = s_axi_araddr[11:2];
    assign s_axi_arready = !s_axi_rvalid;

    reg [31:0] rd_mux;
    always @(*) begin
        rd_mux = 32'd0;
        case (rd_word[9:6])
            4'h0: case (rd_word[5:0])
                6'h00: rd_mux = {30'd0, irq_en, 1'b0};
                6'h01: rd_mux = {29'd0, sat_flag, done_sticky, busy};
                6'h02: rd_mux = {16'd0, n_iter};
                6'h03: rd_mux = cycles;
                6'h04: rd_mux = ID_VALUE;
                6'h05: rd_mux = {FRAC_P[7:0], FRAC_MR[7:0], FRAC_MX[7:0], FRAC_W[7:0]};
                default: ;
            endcase
            4'h4, 4'h5: rd_mux = in_rdata;                                   // 0x400 .. 0x5FC
            4'h8: if (rd_word[5:0] < NL) rd_mux = z_flat[rd_word[4:0]*32 +: 32];   // 0x800 ..
            4'h9: if (rd_word[5:0] < NL) rd_mux = g_flat[rd_word[4:0]*32 +: 32];   // 0x900 ..
            default: ;
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            s_axi_rvalid <= 1'b0;
        end else if (s_axi_arvalid && s_axi_arready) begin
            s_axi_rvalid <= 1'b1;
            s_axi_rdata  <= rd_mux;
        end else if (s_axi_rvalid && s_axi_rready) begin
            s_axi_rvalid <= 1'b0;
        end
    end
endmodule

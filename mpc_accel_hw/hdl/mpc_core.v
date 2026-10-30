// MPC accelerator core: Control FSM + coefficient ROM (BRAM) + input buffer + 20 MAC lanes.
//
// Per solve (start pulse):
//   Prediction Engine   g  = R(MX x, FRAC_MX) - R(MR r, FRAC_MR)        12 + 80 MAC cycles
//   N_ITER x ADMM:      rhs = rho (z - y) - g                           Cost Function Unit
//                       u   = W rhs ; u_hat = alpha u + (1-alpha) z     Numerical Solver Kernel (20 MAC cycles)
//                       z   = clamp(u_hat + y) ; y = y + u_hat - z      Constraint Evaluation
// Cold start z = y = 0 every solve. Result z[0..19]; the PS uses z[0..3].
//
// Coefficient ROM word a (640 bit, lane i in bits [32i+31:32i]):
//   a = 0..11  column a of MX,  a = 12..91  column a-12 of MR,  a = 92..111  column a-92 of W.
// The input buffer uses the same address for x (0..11) and r (12..91), so one address drives both.
module mpc_core #(
    parameter NL         = 20,              // lanes = NU_TOTAL
    parameter NX         = 12,
    parameter NR         = 80,
    parameter ACC_W      = 76,
    parameter FRAC_MX    = 27,
    parameter FRAC_MR    = 31,
    parameter FRAC_W     = 29,
    parameter FRAC_P     = 30,
    parameter COEF_FILE  = "coef_col.mem",
    parameter ZMIN_FILE  = "zmin.mem",
    parameter ZMAX_FILE  = "zmax.mem",
    parameter PARAM_FILE = "params.mem"
) (
    input  wire               clk,
    input  wire               rst,
    input  wire               start,
    input  wire [15:0]        n_iter,
    output reg                busy,
    output reg                done,          // one-cycle pulse
    output wire               sat_flag,      // any int32 saturation during the last solve
    output reg  [31:0]        cycles,        // clock cycles of the last solve
    // input buffer (x0..x11, r0..r79): write port + asynchronous read port for the bus
    input  wire               in_we,
    input  wire [6:0]         in_waddr,
    input  wire [31:0]        in_wdata,
    input  wire [6:0]         in_raddr,
    output wire [31:0]        in_rdata,
    output wire [NL*32-1:0]   z_flat,
    output wire [NL*32-1:0]   g_flat
);
    `include "mpc_ops.vh"

    localparam NIN    = NX + NR;           // 92
    localparam NCOEF  = NIN + NL;          // 112
    localparam W_BASE = NIN;
    localparam GAP    = 3;                 // slots between a lane-state write and its first read
    localparam DRAIN  = 6;                 // cycles until the last op has retired

    // ---------------------------------------------------------------- BRAM / constants
    (* rom_style = "block" *) reg [NL*32-1:0] coef_rom [0:NCOEF-1];
    (* ram_style = "distributed" *) reg [31:0] in_buf [0:NIN-1];
    reg [31:0] zmin_rom [0:NL-1];
    reg [31:0] zmax_rom [0:NL-1];
    reg [31:0] param_rom [0:2];
    initial begin
        $readmemh(COEF_FILE, coef_rom);
        $readmemh(ZMIN_FILE, zmin_rom);
        $readmemh(ZMAX_FILE, zmax_rom);
        $readmemh(PARAM_FILE, param_rom);
    end

    always @(posedge clk)
        if (in_we && in_waddr < NIN) in_buf[in_waddr] <= in_wdata;
    assign in_rdata = (in_raddr < NIN) ? in_buf[in_raddr] : 32'd0;

    // ---------------------------------------------------------------- Control FSM
    localparam [3:0] ST_IDLE = 4'd0,  ST_PX   = 4'd1,  ST_GX  = 4'd2,  ST_PR  = 4'd3,
                     ST_GR   = 4'd4,  ST_RHS  = 4'd5,  ST_OMZ = 4'd6,  ST_MW  = 4'd7,
                     ST_RU   = 4'd8,  ST_AU   = 4'd9,  ST_UPD = 4'd10, ST_WAIT = 4'd11,
                     ST_DRAIN = 4'd12;

    reg [3:0]  state, ret_state;
    reg [6:0]  cnt;
    reg [3:0]  wcnt;
    reg [15:0] it, n_iter_r;
    reg        clear;

    // issued this cycle (combinational from state)
    reg [3:0] op_f;
    reg [6:0] addr_f;
    reg       sel_rhs_f;

    always @(*) begin
        op_f      = OP_NOP;
        addr_f    = 7'd0;
        sel_rhs_f = 1'b0;
        case (state)
            ST_PX:  begin op_f = (cnt == 0) ? OP_MAC0 : OP_MAC; addr_f = cnt; end
            ST_GX:  op_f = OP_RND_GX;
            ST_PR:  begin op_f = (cnt == 0) ? OP_MAC0 : OP_MAC; addr_f = NX + cnt; end
            ST_GR:  op_f = OP_RND_GR;
            ST_RHS: op_f = OP_RHS;
            ST_OMZ: op_f = OP_OMZ;
            ST_MW:  begin op_f = (cnt == 0) ? OP_MAC0 : OP_MAC; addr_f = W_BASE + cnt; sel_rhs_f = 1'b1; end
            ST_RU:  op_f = OP_RND_U;
            ST_AU:  op_f = OP_AU;
            ST_UPD: op_f = OP_UPD;
            default: ;
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            state <= ST_IDLE;
            busy  <= 1'b0;
            done  <= 1'b0;
            clear <= 1'b0;
            cycles <= 32'd0;
        end else begin
            done  <= 1'b0;
            clear <= 1'b0;
            if (busy) cycles <= cycles + 32'd1;
            case (state)
                ST_IDLE: if (start) begin
                    busy     <= 1'b1;
                    clear    <= 1'b1;
                    cycles   <= 32'd1;
                    n_iter_r <= n_iter;
                    it       <= 16'd0;
                    cnt      <= 7'd0;
                    state    <= ST_PX;
                end
                ST_PX: begin
                    cnt <= (cnt == NX - 1) ? 7'd0 : cnt + 7'd1;
                    if (cnt == NX - 1) state <= ST_GX;
                end
                ST_GX: state <= ST_PR;
                ST_PR: begin
                    cnt <= (cnt == NR - 1) ? 7'd0 : cnt + 7'd1;
                    if (cnt == NR - 1) state <= ST_GR;
                end
                ST_GR: begin
                    wcnt  <= DRAIN;
                    state <= (n_iter_r == 0) ? ST_DRAIN : ST_RHS;
                end
                ST_RHS: state <= ST_OMZ;
                ST_OMZ: begin                  // OMZ fills one of the GAP slots before the first MAC
                    wcnt      <= GAP - 1;
                    ret_state <= ST_MW;
                    state     <= ST_WAIT;
                end
                ST_MW: begin
                    cnt <= (cnt == NL - 1) ? 7'd0 : cnt + 7'd1;
                    if (cnt == NL - 1) state <= ST_RU;
                end
                ST_RU: begin
                    wcnt      <= GAP;
                    ret_state <= ST_AU;
                    state     <= ST_WAIT;
                end
                ST_AU:  state <= ST_UPD;
                ST_UPD: begin
                    it <= it + 16'd1;
                    if (it == n_iter_r - 1) begin
                        wcnt  <= DRAIN;
                        state <= ST_DRAIN;
                    end else begin
                        wcnt      <= GAP;
                        ret_state <= ST_RHS;
                        state     <= ST_WAIT;
                    end
                end
                ST_WAIT: begin
                    wcnt <= wcnt - 4'd1;
                    if (wcnt == 1) state <= ret_state;
                end
                ST_DRAIN: begin
                    wcnt <= wcnt - 4'd1;
                    if (wcnt == 1) begin
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        state <= ST_IDLE;
                    end
                end
                default: state <= ST_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------- issue stage (aligns op with ROM read)
    reg  [NL*32-1:0] coef_q;
    reg  [31:0]      in_q;
    reg  [3:0]       op_q;
    reg              sel_rhs_q;
    reg  [4:0]       rhs_idx_q;
    always @(posedge clk) begin
        coef_q    <= coef_rom[addr_f];
        in_q      <= in_buf[addr_f];
        op_q      <= rst ? OP_NOP : op_f;
        sel_rhs_q <= sel_rhs_f;
        rhs_idx_q <= cnt[4:0];
    end

    // ---------------------------------------------------------------- lanes
    wire [NL*32-1:0] rhs_flat;
    wire [NL-1:0]    lane_sat;
    wire [31:0]      rhs_bcast = rhs_flat[rhs_idx_q*32 +: 32];
    wire [31:0]      data_bcast = sel_rhs_q ? rhs_bcast : in_q;

    genvar i;
    generate
        for (i = 0; i < NL; i = i + 1) begin : lane
            mpc_lane #(
                .ACC_W(ACC_W), .FRAC_MX(FRAC_MX), .FRAC_MR(FRAC_MR), .FRAC_W(FRAC_W), .FRAC_P(FRAC_P)
            ) u_lane (
                .clk(clk),
                .clear(clear),
                .op(op_q),
                .coef_in(coef_q[i*32 +: 32]),
                .data_in(data_bcast),
                .rho(param_rom[0]),
                .alpha(param_rom[1]),
                .one_m_alpha(param_rom[2]),
                .zmin(zmin_rom[i]),
                .zmax(zmax_rom[i]),
                .g(g_flat[i*32 +: 32]),
                .z(z_flat[i*32 +: 32]),
                .y(),
                .rhs(rhs_flat[i*32 +: 32]),
                .sat_flag(lane_sat[i])
            );
        end
    endgenerate

    assign sat_flag = |lane_sat;
endmodule

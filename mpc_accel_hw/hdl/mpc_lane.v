// One MAC lane of the MPC accelerator. Lane i owns row i of MX, MR, W and element i of g, z, y.
// All 20 lanes run in lockstep from the same op stream (mpc_core).
//
// Pipeline (one op enters per cycle, results retire in issue order):
//   S1  operand select + register   (a = coefficient, b = data)
//   S2  m = a * b                   (signed 32x32 -> 64, maps to DSP48E2)
//   S3  p = m                       (DSP output register)
//   S4  retire: accumulate / round / update state
// An op that reads z, y, u or rhs in S1 must be issued >= GAP (=3) slots after the op that writes it in S4.
//
// Arithmetic matches the golden model (fixed_point.h) bit-exactly:
//   products summed exactly in an ACC_W-bit accumulator, one round-half-up (+2^(F-1)) >>> F, saturate to int32.
module mpc_lane #(
    parameter ACC_W   = 76,
    parameter FRAC_MX = 27,
    parameter FRAC_MR = 31,
    parameter FRAC_W  = 29,
    parameter FRAC_P  = 30
) (
    input  wire               clk,
    input  wire               clear,       // start of a solve: z = y = 0, sat_flag = 0
    input  wire [3:0]         op,
    input  wire signed [31:0] coef_in,     // ROM word for this lane (MAC ops)
    input  wire signed [31:0] data_in,     // broadcast x[j], r[j] or rhs[j] (MAC ops)
    input  wire signed [31:0] rho,
    input  wire signed [31:0] alpha,
    input  wire signed [31:0] one_m_alpha,
    input  wire signed [31:0] zmin,
    input  wire signed [31:0] zmax,
    output reg  signed [31:0] g,
    output reg  signed [31:0] z,
    output reg  signed [31:0] y,
    output reg  signed [31:0] rhs,
    output reg                sat_flag
);
    `include "mpc_ops.vh"

    // ---------------------------------------------------------------- helpers
    function ovf_w;                      // value does not fit in int32
        input signed [ACC_W-1:0] v;
        ovf_w = !((&v[ACC_W-1:31]) || !(|v[ACC_W-1:31]));
    endfunction

    function signed [31:0] sat_w;
        input signed [ACC_W-1:0] v;
        sat_w = ovf_w(v) ? (v[ACC_W-1] ? 32'sh80000000 : 32'sh7FFFFFFF) : v[31:0];
    endfunction

    function ovf_33;
        input signed [32:0] v;
        ovf_33 = v[32] ^ v[31];
    endfunction

    function signed [31:0] sat_33;
        input signed [32:0] v;
        sat_33 = ovf_33(v) ? (v[32] ? 32'sh80000000 : 32'sh7FFFFFFF) : v[31:0];
    endfunction

    function signed [ACC_W-1:0] round_shift;   // (v + 2^(sh-1)) >>> sh
        input signed [ACC_W-1:0] v;
        input integer sh;
        reg signed [ACC_W-1:0] half;
        begin
            half = {{(ACC_W-1){1'b0}}, 1'b1} << (sh - 1);
            round_shift = (v + half) >>> sh;
        end
    endfunction

    // ---------------------------------------------------------------- S1: operand select
    reg signed [31:0] u;                 // W * rhs, rounded
    reg signed [31:0] t_omz;             // (1 - alpha) * z
    reg signed [31:0] u_hat;             // alpha * u + (1 - alpha) * z
    reg signed [31:0] gx;                // MX * x, rounded

    wire signed [32:0] zy_diff = {z[31], z} - {y[31], y};

    reg signed [31:0] a1, b1;
    reg        [3:0]  op1, op2, op3;
    reg               ovf1;              // saturation of z - y in S1

    always @(posedge clk) begin
        op1  <= op;
        ovf1 <= 1'b0;
        case (op)
            OP_MAC0, OP_MAC: begin a1 <= coef_in;     b1 <= data_in; end
            OP_RHS:          begin a1 <= rho;         b1 <= sat_33(zy_diff); ovf1 <= ovf_33(zy_diff); end
            OP_OMZ:          begin a1 <= one_m_alpha; b1 <= z; end
            OP_AU:           begin a1 <= alpha;       b1 <= u; end
            default:         begin a1 <= 32'sd0;      b1 <= 32'sd0; end
        endcase
    end

    // ---------------------------------------------------------------- S2, S3: multiply
    reg signed [63:0] m2, p3;
    reg               ovf2, ovf3;
    always @(posedge clk) begin
        m2   <= a1 * b1;
        op2  <= op1;
        ovf2 <= ovf1;
        p3   <= m2;
        op3  <= op2;
        ovf3 <= ovf2;
    end

    // ---------------------------------------------------------------- S4: retire
    reg signed [ACC_W-1:0] acc;
    wire signed [ACC_W-1:0] p_ext = {{(ACC_W-64){p3[63]}}, p3};

    wire signed [ACC_W-1:0] p_rnd   = round_shift(p_ext, FRAC_P);   // coef_p * data
    wire signed [ACC_W-1:0] acc_mx  = round_shift(acc, FRAC_MX);
    wire signed [ACC_W-1:0] acc_mr  = round_shift(acc, FRAC_MR);
    wire signed [ACC_W-1:0] acc_w   = round_shift(acc, FRAC_W);
    wire signed [31:0]      p_sat   = sat_w(p_rnd);
    wire signed [31:0]      mr_sat  = sat_w(acc_mr);

    wire signed [32:0] g_sum    = {gx[31], gx} - {mr_sat[31], mr_sat};
    wire signed [32:0] rhs_sum  = {p_sat[31], p_sat} - {g[31], g};
    wire signed [32:0] uhat_sum = {p_sat[31], p_sat} + {t_omz[31], t_omz};

    // Constraint evaluation: cand = clamp(u_hat + y), y' = (y + u_hat) - cand
    wire signed [32:0] uy_sum = {y[31], y} + {u_hat[31], u_hat};
    wire signed [31:0] uy_sat = sat_33(uy_sum);
    wire signed [31:0] cand   = (uy_sat < zmin) ? zmin : (uy_sat > zmax) ? zmax : uy_sat;
    wire signed [32:0] y_sum  = {uy_sat[31], uy_sat} - {cand[31], cand};

    always @(posedge clk) begin
        if (clear) begin
            z        <= 32'sd0;
            y        <= 32'sd0;
            sat_flag <= 1'b0;
        end else begin
            case (op3)
                OP_MAC0:   acc <= p_ext;
                OP_MAC:    acc <= acc + p_ext;
                OP_RND_GX: begin
                    gx <= sat_w(acc_mx);
                    sat_flag <= sat_flag | ovf_w(acc_mx);
                end
                OP_RND_GR: begin
                    g <= sat_33(g_sum);
                    sat_flag <= sat_flag | ovf_w(acc_mr) | ovf_33(g_sum);
                end
                OP_RHS: begin                      // cost-function gradient: rho (z - y) - g
                    rhs <= sat_33(rhs_sum);
                    sat_flag <= sat_flag | ovf3 | ovf_w(p_rnd) | ovf_33(rhs_sum);
                end
                OP_OMZ: begin
                    t_omz <= p_sat;
                    sat_flag <= sat_flag | ovf_w(p_rnd);
                end
                OP_RND_U: begin
                    u <= sat_w(acc_w);
                    sat_flag <= sat_flag | ovf_w(acc_w);
                end
                OP_AU: begin
                    u_hat <= sat_33(uhat_sum);
                    sat_flag <= sat_flag | ovf_w(p_rnd) | ovf_33(uhat_sum);
                end
                OP_UPD: begin
                    z <= cand;
                    y <= sat_33(y_sum);
                    sat_flag <= sat_flag | ovf_33(uy_sum) | ovf_33(y_sum);
                end
                default: ;
            endcase
        end
    end
endmodule

#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include "admm_solver.h"
#include "quadcopter_model.h"

namespace quad {

// Four-stage MPC pipeline, templated on the number format (FmtDouble, FmtQ16_16, FmtQ8_24).
template <class Fmt>
class MpcController {
public:
    using T = typename Fmt::data;
    using TMx = typename Fmt::coef_mx;
    using TMr = typename Fmt::coef_mr;

    AdmmSolver<Fmt> solver;
    TMx Mx[NU_TOTAL][NX];
    TMr Mr[NU_TOTAL][NR];
    int n_iter;

    explicit MpcController(int iters = N_ITER) : n_iter(iters) {
        for (int i = 0; i < NU_TOTAL; ++i) {
            for (int j = 0; j < NX; ++j) Mx[i][j] = from_real<TMx>(solver.qp.MX[i][j]);
            for (int j = 0; j < NR; ++j) Mr[i][j] = from_real<TMr>(solver.qp.MR[i][j]);
        }
    }

    // Stage 1: Current State -- scale (power of two) and quantize the measured state and reference.
    void stage1_acquire_state(const double x_meas[NX], T x_out[NX]) const {
        for (int i = 0; i < NX; ++i) x_out[i] = from_real<T>(x_meas[i] * solver.qp.x_scale[i]);
    }
    void stage1_acquire_reference(const double r[NR], T r_out[NR]) const {
        for (int k = 0; k < NR; ++k) r_out[k] = from_real<T>(r[k] * solver.qp.r_scale[k]);
    }

    // Stage 2: Prediction -- online gradient g = Mx x0 - Mr r (H and W are LTI, precomputed offline).
    void stage2_prediction(const T x0[NX], const T r[NR], T g[NU_TOTAL]) const {
        for (int i = 0; i < NU_TOTAL; ++i) g[i] = dot(Mx[i], x0, NX) - dot(Mr[i], r, NR);
    }

    // Stage 3: Optimization -- fixed-iteration ADMM.
    void stage3_optimization(const T g[NU_TOTAL], T z[NU_TOTAL]) const { solver.solve_fixed(g, z, n_iter); }

    // Stage 4: Optimal Control -- unscale the first move and add hover thrust.
    void stage4_optimal_control(const T z[NU_TOTAL], double u_final[NU]) const {
        for (int i = 0; i < NU; ++i) u_final[i] = solver.qp.u_scale[i] * to_real(z[i]);
        u_final[0] += HOVER_THRUST;
    }

    void compute(const double x_meas[NX], const double r[NR], double u_final[NU],
                 T* g_out = nullptr, T* z_out = nullptr) const {
        T x[NX], r_q[NR], g[NU_TOTAL], z[NU_TOTAL];
        stage1_acquire_state(x_meas, x);
        stage1_acquire_reference(r, r_q);
        stage2_prediction(x, r_q, g);
        stage3_optimization(g, z);
        stage4_optimal_control(z, u_final);
        if (g_out) std::copy(g, g + NU_TOTAL, g_out);
        if (z_out) std::copy(z, z + NU_TOTAL, z_out);
    }
};

} // namespace quad

#endif // MPC_CONTROLLER_H

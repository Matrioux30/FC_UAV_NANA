#ifndef ADMM_SOLVER_H
#define ADMM_SOLVER_H

#include "number_formats.h"
#include <algorithm>
#include <cmath>

namespace quad {

// Over-relaxed ADMM for the box-constrained scaled QP  min 0.5 z'Hz + g'z,  Z_MIN <= z <= Z_MAX:
//   rhs   = rho (z - y) - g
//   u     = W rhs                    (W = (H + rho I)^-1, precomputed offline)
//   u_hat = alpha u + (1 - alpha) z
//   z     = clamp(u_hat + y)
//   y     = y + u_hat - z
template <class Fmt>
class AdmmSolver {
public:
    using T = typename Fmt::data;
    using TW = typename Fmt::coef_w;
    using TP = typename Fmt::coef_p;

    const QpConstants& qp;
    TW W[NU_TOTAL][NU_TOTAL];
    T z_min[NU_TOTAL];
    T z_max[NU_TOTAL];
    TP rho;
    TP alpha;
    TP one_minus_alpha;

    AdmmSolver()
        : qp(Fmt::qp()), rho(from_real<TP>(qp.rho)), alpha(from_real<TP>(ALPHA)),
          one_minus_alpha(from_real<TP>(1.0 - ALPHA)) {
        for (int i = 0; i < NU_TOTAL; ++i) {
            for (int j = 0; j < NU_TOTAL; ++j) W[i][j] = from_real<TW>(qp.W[i][j]);
            z_min[i] = from_real<T>(qp.z_min[i]);
            z_max[i] = from_real<T>(qp.z_max[i]);
        }
    }

    // Fixed iteration count: deterministic latency, used by the accelerator.
    void solve_fixed(const T g[NU_TOTAL], T z[NU_TOTAL], int n_iter = N_ITER) const {
        T y[NU_TOTAL];
        for (int i = 0; i < NU_TOTAL; ++i) {
            z[i] = T();
            y[i] = T();
        }
        for (int it = 0; it < n_iter; ++it) iterate(g, z, y, nullptr, nullptr);
    }

    // Runs until primal and dual residuals (scaled units) drop below tol; returns iterations used.
    // warm_z / warm_y (optional) seed the iterates.
    int solve_to_tolerance(const T g[NU_TOTAL], T z[NU_TOTAL], double tol, int max_iter,
                           const T* warm_z = nullptr, const T* warm_y = nullptr, T* y_out = nullptr) const {
        T y[NU_TOTAL];
        for (int i = 0; i < NU_TOTAL; ++i) {
            z[i] = warm_z ? warm_z[i] : T();
            y[i] = warm_y ? warm_y[i] : T();
        }
        int it = 0;
        while (it < max_iter) {
            double pri_sq = 0.0, dual_sq = 0.0;
            iterate(g, z, y, &pri_sq, &dual_sq);
            ++it;
            if (std::sqrt(pri_sq) < tol && qp.rho * std::sqrt(dual_sq) < tol) break;
        }
        if (y_out) std::copy(y, y + NU_TOTAL, y_out);
        return it;
    }

private:
    void iterate(const T g[NU_TOTAL], T z[NU_TOTAL], T y[NU_TOTAL], double* pri_sq, double* dual_sq) const {
        T rhs[NU_TOTAL];
        for (int i = 0; i < NU_TOTAL; ++i) rhs[i] = mul(rho, z[i] - y[i]) - g[i];
        for (int i = 0; i < NU_TOTAL; ++i) {
            const T u = dot(W[i], rhs, NU_TOTAL);
            const T u_hat = mul(alpha, u) + mul(one_minus_alpha, z[i]);
            T cand = u_hat + y[i];
            if (cand < z_min[i]) cand = z_min[i];
            else if (cand > z_max[i]) cand = z_max[i];
            if (pri_sq) {
                const double dp = to_real(u) - to_real(cand);
                const double dd = to_real(cand) - to_real(z[i]);
                *pri_sq += dp * dp;
                *dual_sq += dd * dd;
            }
            y[i] = y[i] + u_hat - cand;
            z[i] = cand;
        }
    }
};

} // namespace quad

#endif // ADMM_SOLVER_H

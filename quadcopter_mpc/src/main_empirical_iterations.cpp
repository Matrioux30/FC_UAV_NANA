#include "mpc_controller.h"
#include "scenarios.h"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <vector>

using namespace quad;

constexpr double RES_TOL = 1e-5;      // primal/dual residual tolerance (scaled units)
constexpr int MAX_ITER = 2000;
constexpr double ACC_TOL = 1e-3;      // |du0| / u_max, same as golden model
constexpr double EXACT_TOL = 1e-11;

struct IterStats {
    double avg = 0;
    int p95 = 0;
    int max = 0;
};

IterStats stats(std::vector<int> v) {
    IterStats s;
    s.avg = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    std::sort(v.begin(), v.end());
    s.p95 = v[static_cast<size_t>(0.95 * (v.size() - 1))];
    s.max = v.back();
    return s;
}

int main() {
    const MpcController<FmtDouble> mpc;
    std::cout << "==========================================================================================\n"
              << " ANALISIS EMPIRIS ITERASI ADMM (skenario & plant sama dengan golden model)\n"
              << " Konvergen: residu primal & dual < " << RES_TOL << "; akurasi: |du0|/u_max <= " << ACC_TOL << "\n"
              << "==========================================================================================\n";
    std::cout << std::left << std::setw(12) << "Skenario" << std::setw(18) << "Cold avg/p95/max"
              << std::setw(18) << "Warm avg/p95/max" << std::setw(26) << "N_ITER=" + std::to_string(N_ITER) + " akurat"
              << "Error maks N_ITER\n";
    std::cout << "------------------------------------------------------------------------------------------\n";

    int worst_p95 = 0;
    bool all_accurate = true;
    for (const Scenario& sc : golden_scenarios()) {
        std::vector<int> cold, warm;
        int accurate = 0;
        double worst_err = 0;
        double warm_z[NU_TOTAL] = {}, warm_y[NU_TOTAL] = {};
        bool has_warm = false;

        QuadState x = sc.x0;
        const int n = sc.num_steps();
        for (int k = 0; k < n; ++k) {
            double r[NR], g[NU_TOTAL], z[NU_TOTAL], y[NU_TOTAL], z_ex[NU_TOTAL], z_fixed[NU_TOTAL];
            sc.horizon(k * TS, r);
            double xq[NX], rq[NR];
            mpc.stage1_acquire_state(x.val.data(), xq);
            mpc.stage1_acquire_reference(r, rq);
            mpc.stage2_prediction(xq, rq, g);

            cold.push_back(mpc.solver.solve_to_tolerance(g, z, RES_TOL, MAX_ITER));
            warm.push_back(mpc.solver.solve_to_tolerance(g, z, RES_TOL, MAX_ITER, has_warm ? warm_z : nullptr,
                                                         has_warm ? warm_y : nullptr, y));
            // Shift the horizon by one move for the next cycle's warm start. Each z_i has its own scale
            // (U_i = u_scale_i * z_i), so the shift is done in physical units.
            const double* es = mpc.solver.qp.u_scale;
            for (int i = 0; i < NU_TOTAL; ++i) {
                const int src = std::min(i + NU, NU_TOTAL - NU + i % NU);
                warm_z[i] = z[src] * es[src] / es[i];
                warm_y[i] = y[src] * es[src] / es[i];
            }
            has_warm = true;

            mpc.solver.solve_to_tolerance(g, z_ex, EXACT_TOL, 200000);
            mpc.stage3_optimization(g, z_fixed);
            double u_ex[NU], u_fx[NU];
            mpc.stage4_optimal_control(z_ex, u_ex);
            mpc.stage4_optimal_control(z_fixed, u_fx);
            double e = 0;
            for (int c = 0; c < NU; ++c) e = std::max(e, std::abs(u_fx[c] - u_ex[c]) / U_MAX_CH[c]);
            worst_err = std::max(worst_err, e);
            accurate += e <= ACC_TOL;

            x = quadcopter_step(x, QuadControl{u_fx[0], u_fx[1], u_fx[2], u_fx[3]}, TS);
        }
        const IterStats c = stats(cold), w = stats(warm);
        worst_p95 = std::max(worst_p95, c.p95);
        all_accurate &= accurate == n;
        auto fmt = [](const IterStats& s) {
            std::ostringstream o;
            o << std::fixed << std::setprecision(1) << s.avg << "/" << s.p95 << "/" << s.max;
            return o.str();
        };
        std::cout << std::left << std::setw(12) << sc.name << std::setw(18) << fmt(c) << std::setw(18) << fmt(w)
                  << std::setw(26) << (std::to_string(accurate) + "/" + std::to_string(n) + " siklus")
                  << std::scientific << std::setprecision(2) << worst_err << "\n";
    }
    std::cout << "==========================================================================================\n"
              << "Kesimpulan FSM Hardware:\n";
    if (all_accurate) {
        std::cout << "N_ITER = " << N_ITER << " (cold start, tanpa state antar-siklus) memenuhi akurasi " << ACC_TOL
                  << " di SEMUA siklus.\nIterasi sampai residu < " << RES_TOL << " (p95 cold) = " << worst_p95
                  << "; kriteria residu lebih ketat dari yang dibutuhkan akurasi u0.\n";
    } else {
        std::cout << "PERINGATAN: N_ITER = " << N_ITER << " belum memenuhi akurasi " << ACC_TOL
                  << " di semua siklus -- naikkan N_ITER atau tuning rho.\n";
    }
    return 0;
}

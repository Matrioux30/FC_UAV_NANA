#include "mpc_controller.h"
#include "scenarios.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

using namespace quad;

constexpr int N_WARMUP = 200;
constexpr int N_MEASURE = 5000;
constexpr double TS_US = TS * 1e6;

struct TimingStats {
    double avg, min, max, p99, stddev, p2p, ratio_avg, ratio_max;
};

TimingStats calc(std::vector<double> v) {
    TimingStats s{};
    s.avg = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    double var = 0;
    for (double t : v) var += (t - s.avg) * (t - s.avg);
    s.stddev = std::sqrt(var / v.size());
    std::sort(v.begin(), v.end());
    s.min = v.front();
    s.max = v.back();
    s.p99 = v[static_cast<size_t>(0.99 * (v.size() - 1))];
    s.p2p = s.max - s.min;
    s.ratio_avg = 100.0 * s.avg / TS_US;
    s.ratio_max = 100.0 * s.max / TS_US;
    return s;
}

int main() {
    using Clock = std::chrono::steady_clock;
    const MpcController<FmtDouble> mpc;
    const Scenario sc = golden_scenarios()[2];  // trajectory: continuously changing reference

    std::vector<double> t1, t2, t3, t4, tot;
    for (auto* v : {&t1, &t2, &t3, &t4, &tot}) v->reserve(N_MEASURE);

    QuadState x = sc.x0;
    const int period = sc.num_steps();
    for (int k = 0; k < N_WARMUP + N_MEASURE; ++k) {
        double r[NR];
        sc.horizon((k % period) * TS, r);
        if (k % period == 0) x = sc.x0;

        double xq[NX], rq[NR], g[NU_TOTAL], z[NU_TOTAL], u[NU];
        const auto a = Clock::now();
        mpc.stage1_acquire_state(x.val.data(), xq);
        mpc.stage1_acquire_reference(r, rq);
        const auto b = Clock::now();
        mpc.stage2_prediction(xq, rq, g);
        const auto c = Clock::now();
        mpc.stage3_optimization(g, z);
        const auto d = Clock::now();
        mpc.stage4_optimal_control(z, u);
        const auto e = Clock::now();

        x = quadcopter_step(x, QuadControl{u[0], u[1], u[2], u[3]}, TS);
        if (k < N_WARMUP) continue;
        auto us = [](auto p, auto q) { return std::chrono::duration<double, std::micro>(q - p).count(); };
        t1.push_back(us(a, b));
        t2.push_back(us(b, c));
        t3.push_back(us(c, d));
        t4.push_back(us(d, e));
        tot.push_back(us(a, e));
    }

    // Effective timer resolution: smallest observed non-zero step between consecutive readings.
    double tick_ns = 1e18;
    for (int i = 0; i < 1000; ++i) {
        const auto t0 = Clock::now();
        auto t1 = Clock::now();
        while (t1 == t0) t1 = Clock::now();
        tick_ns = std::min(tick_ns, std::chrono::duration<double, std::nano>(t1 - t0).count());
    }
    const TimingStats s1 = calc(t1), s2 = calc(t2), s3 = calc(t3), s4 = calc(t4), st = calc(tot);

    std::cout << "======================================================================================================\n"
              << " COMPUTATIONAL PROFILING PER TAHAP MPC (CPU baseline, double)  N = " << N_MEASURE
              << " siklus, Ts = " << TS_US << " us, resolusi timer = " << tick_ns << " ns\n"
              << " ADMM: " << NU_TOTAL << " variabel, N_ITER = " << N_ITER << "\n"
              << "======================================================================================================\n";
    std::cout << std::left << std::setw(30) << "Tahap" << std::setw(10) << "Avg" << std::setw(9) << "Min"
              << std::setw(10) << "P99" << std::setw(10) << "Max" << std::setw(11) << "Jitter-sd" << std::setw(11)
              << "Jitter-pp" << std::setw(10) << "Avg/Ts%" << std::setw(10) << "Max/Ts%" << "Partisi\n";
    std::cout << "------------------------------------------------------------------------------------------------------\n";
    auto row = [](const std::string& n, const TimingStats& s, const std::string& part) {
        std::cout << std::left << std::setw(30) << n << std::fixed << std::setprecision(2) << std::setw(10) << s.avg
                  << std::setw(9) << s.min << std::setw(10) << s.p99 << std::setw(10) << s.max << std::setw(11)
                  << s.stddev << std::setw(11) << s.p2p << std::setprecision(4) << std::setw(10) << s.ratio_avg
                  << std::setw(10) << s.ratio_max << part << "\n";
    };
    row("1. Current State", s1, "PS");
    row("2. Prediction (g = Mx x - Mr r)", s2, "PL (MAC)");
    row("3. Optimization (ADMM)", s3, "PL (Solver)");
    row("4. Optimal Control", s4, "PS");
    std::cout << "------------------------------------------------------------------------------------------------------\n";
    row("TOTAL 1 siklus kendali", st, st.max < TS_US ? "Real-time OK" : "MELANGGAR Ts");
    std::cout << "(waktu dalam us; H dan W dihitung offline sehingga 0 us online; Max = worst-case teramati di OS non-RT)\n\n"
              << "Porsi waktu: Prediction " << std::setprecision(1) << 100.0 * s2.avg / st.avg << "%, Optimization "
              << 100.0 * s3.avg / st.avg << "% -> kandidat akselerasi PL: tahap 2 dan 3.\n";

    std::ofstream csv("results/profiling_summary.csv");
    csv << "Tahap,Avg (us),Min (us),P99 (us),Max (us),Jitter StdDev (us),Jitter Peak-Peak (us),Avg/Ts (%),Max/Ts (%),Partisi\n";
    auto crow = [&](const std::string& n, const TimingStats& s, const std::string& p) {
        csv << n << "," << s.avg << "," << s.min << "," << s.p99 << "," << s.max << "," << s.stddev << "," << s.p2p
            << "," << s.ratio_avg << "," << s.ratio_max << "," << p << "\n";
    };
    crow("1. Current State", s1, "PS");
    csv << "2a. Precompute H/W (offline),0,0,0,0,0,0,0,0,Offline/BRAM\n";
    crow("2b. Prediction (online gradient)", s2, "PL");
    crow("3. Optimization (ADMM)", s3, "PL");
    crow("4. Optimal Control", s4, "PS");
    crow("Total", st, "-");
    std::cout << "Disimpan ke results/profiling_summary.csv\n";
    return 0;
}

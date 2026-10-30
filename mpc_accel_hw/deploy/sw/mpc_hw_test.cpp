// On-board verification and profiling of the MPC accelerator.
//   sudo ./mpc_hw_test tv.hex [n_vectors] [out.csv]
// For every test vector (mem/<fmt>/tv.hex): write x, r over AXI, start, poll DONE, read z and g, compare bit-exactly.
// Reports the PL latency (CYCLES register) and the PS-measured end-to-end time (AXI writes + solve + reads),
// as average, min, P99, max, jitter (std dev, peak-peak) and ratio to Ts = 5 ms.
#include "mpc_accel.hpp"

#include <algorithm>
#include <cinttypes>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace {

constexpr double TS_US = 5000.0;
constexpr double PL_CLK_MHZ = 100.0;

double now_us() {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC_RAW, &t);
    return t.tv_sec * 1e6 + t.tv_nsec * 1e-3;
}

struct Stats {
    double avg, min, p99, max, sd, pp;
};

Stats stats(std::vector<double> v) {
    Stats s{};
    if (v.empty()) return s;
    std::sort(v.begin(), v.end());
    double sum = 0, sq = 0;
    for (double x : v) { sum += x; sq += x * x; }
    s.avg = sum / v.size();
    s.sd = std::sqrt(std::max(0.0, sq / v.size() - s.avg * s.avg));
    s.min = v.front();
    s.max = v.back();
    s.p99 = v[std::min(v.size() - 1, static_cast<size_t>(0.99 * v.size()))];
    s.pp = s.max - s.min;
    return s;
}

void print_row(const char* name, const Stats& s) {
    std::printf("%-28s %9.3f %9.3f %9.3f %9.3f %9.3f %9.3f %8.4f %8.4f\n", name, s.avg, s.min, s.p99, s.max, s.sd,
                s.pp, 100.0 * s.avg / TS_US, 100.0 * s.max / TS_US);
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: sudo %s tv.hex [n_vectors] [out.csv]\n", argv[0]);
        return 2;
    }
    const size_t n_max = argc > 2 ? std::stoul(argv[2]) : SIZE_MAX;
    const std::string csv_path = argc > 3 ? argv[3] : "hw_profiling.csv";

    std::ifstream tv(argv[1]);
    if (!tv) { std::fprintf(stderr, "cannot open %s\n", argv[1]); return 2; }
    std::vector<std::vector<uint32_t>> vecs;
    for (std::string line; std::getline(tv, line) && vecs.size() < n_max;) {
        std::istringstream ss(line);
        std::vector<uint32_t> v;
        for (std::string w; ss >> w;) v.push_back(static_cast<uint32_t>(std::stoul(w, nullptr, 16)));
        if (v.size() == MpcAccel::NIN + 2 * MpcAccel::NL) vecs.push_back(std::move(v));
    }

    try {
        MpcAccel acc;
        std::printf("accelerator found, format %s, N_ITER %" PRIu32 ", %zu vectors\n", acc.format().name,
                    acc.rd(MpcAccel::N_ITER), vecs.size());

        std::ofstream csv(csv_path);
        csv << "Vector,Pass,Saturated,PL_cycles,PL_us,EndToEnd_us\n";
        std::vector<double> t_pl, t_e2e;
        size_t pass = 0;
        int shown = 0;
        int32_t in[MpcAccel::NIN], z[MpcAccel::NL], g[MpcAccel::NL];

        for (size_t k = 0; k < vecs.size(); ++k) {
            const auto& v = vecs[k];
            for (int i = 0; i < MpcAccel::NIN; ++i) in[i] = static_cast<int32_t>(v[i]);

            const double t0 = now_us();
            acc.write_inputs(in);
            acc.start();
            const bool ok = acc.wait_done();
            acc.read_z(z);
            const double t1 = now_us();
            acc.read_g(g);

            if (!ok) { std::fprintf(stderr, "timeout at vector %zu\n", k); return 1; }
            bool match = !acc.saturated();
            for (int i = 0; i < MpcAccel::NL; ++i) {
                const bool gm = g[i] == static_cast<int32_t>(v[MpcAccel::NIN + i]);
                const bool zm = z[i] == static_cast<int32_t>(v[MpcAccel::NIN + MpcAccel::NL + i]);
                if ((!gm || !zm) && shown < 10) {
                    std::printf("  vec %zu idx %d: g %d/%d  z %d/%d\n", k, i, g[i],
                                static_cast<int32_t>(v[MpcAccel::NIN + i]), z[i],
                                static_cast<int32_t>(v[MpcAccel::NIN + MpcAccel::NL + i]));
                    ++shown;
                }
                match = match && gm && zm;
            }
            pass += match;
            const uint32_t cyc = acc.cycles();
            t_pl.push_back(cyc / PL_CLK_MHZ);
            t_e2e.push_back(t1 - t0);
            csv << k << ',' << match << ',' << acc.saturated() << ',' << cyc << ',' << t_pl.back() << ','
                << t_e2e.back() << '\n';
        }

        std::printf("\n%zu/%zu vectors bit-exact (g and z)\n\n", pass, vecs.size());
        std::printf("%-28s %9s %9s %9s %9s %9s %9s %8s %8s\n", "Tahap (us)", "Avg", "Min", "P99", "Max", "JitSD",
                    "JitPP", "Avg/Ts%", "Max/Ts%");
        print_row("PL solve (CYCLES @100MHz)", stats(t_pl));
        print_row("PS end-to-end (AXI+solve)", stats(t_e2e));
        std::printf("\nper-vector timings -> %s\n", csv_path.c_str());
        return pass == vecs.size() ? 0 : 1;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}

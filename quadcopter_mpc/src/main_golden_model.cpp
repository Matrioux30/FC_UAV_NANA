#include "mpc_controller.h"
#include "scenarios.h"
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace quad;

// Acceptance thresholds, as a fraction of each actuator limit U_MAX_CH (0.5 N, 0.02 Nm).
constexpr double TOL_ALGO = 1e-3;   // float ADMM (N_ITER) vs exact QP optimum
constexpr double TOL_QUANT = 1e-3;  // fixed-point vs float ADMM (N_ITER)
constexpr double EXACT_TOL = 1e-11;
constexpr int EXACT_MAX_ITER = 200000;
const char REF_TAG[NREF] = {'x', 'y', 'z', 'w'};

struct ErrStats {
    double sum_sq[NU] = {};
    double max_abs[NU] = {};
    long n = 0;
    void add(const double a[NU], const double b[NU]) {
        for (int c = 0; c < NU; ++c) {
            const double e = std::abs(a[c] - b[c]) / U_MAX_CH[c];
            sum_sq[c] += e * e;
            max_abs[c] = std::max(max_abs[c], e);
        }
        ++n;
    }
    double worst() const { return *std::max_element(max_abs, max_abs + NU); }
    double rms() const {
        double s = 0;
        for (double v : sum_sq) s += v;
        return n ? std::sqrt(s / (n * NU)) : 0.0;
    }
};

struct Tracking {
    double rmse[3] = {};
};

struct Report {
    std::string name;
    int cycles = 0;
    int active_cycles = 0;
    int max_exact_iter = 0;
    ErrStats algo, q16, q824;
    long long ovf_q16 = 0, ovf_q824 = 0;
    Tracking trk_float, trk_q16, trk_q824;
};

template <typename T>
Tracking closed_loop(const Scenario& sc, const MpcController<T>& mpc) {
    QuadState x = sc.x0;
    double sq[3] = {};
    const int n = sc.num_steps();
    for (int k = 0; k < n; ++k) {
        const double t = k * TS;
        double r[NR], u[NU];
        sc.horizon(t, r);
        for (int i = 0; i < 3; ++i) sq[i] += (r[i] - x[i]) * (r[i] - x[i]);
        mpc.compute(x.val.data(), r, u);
        x = quadcopter_step(x, QuadControl{u[0], u[1], u[2], u[3]}, TS);
    }
    Tracking tr;
    for (int i = 0; i < 3; ++i) tr.rmse[i] = std::sqrt(sq[i] / n);
    return tr;
}

template <typename V>
void dump_mem(const std::string& path, const V* data, int count) {
    std::ofstream f(path);
    char buf[16];
    for (int i = 0; i < count; ++i) {
        std::snprintf(buf, sizeof(buf), "%08X", static_cast<uint32_t>(data[i].raw_value));
        f << buf << "\n";
    }
}

template <class Fmt>
void write_rom(const std::string& dir, const MpcController<Fmt>& mpc) {
    std::filesystem::create_directories(dir);
    dump_mem(dir + "/W.mem", &mpc.solver.W[0][0], NU_TOTAL * NU_TOTAL);
    dump_mem(dir + "/MX.mem", &mpc.Mx[0][0], NU_TOTAL * NX);
    dump_mem(dir + "/MR.mem", &mpc.Mr[0][0], NU_TOTAL * NR);
    dump_mem(dir + "/ZMIN.mem", mpc.solver.z_min, NU_TOTAL);
    dump_mem(dir + "/ZMAX.mem", mpc.solver.z_max, NU_TOTAL);
    const typename Fmt::coef_p params[3] = {mpc.solver.rho, mpc.solver.alpha, mpc.solver.one_minus_alpha};
    dump_mem(dir + "/PARAMS_rho_alpha_1malpha.mem", params, 3);
    using D = typename Fmt::data;
    using W = typename Fmt::coef_w;
    using X = typename Fmt::coef_mx;
    using R = typename Fmt::coef_mr;
    using P = typename Fmt::coef_p;
    std::ofstream f(dir + "/formats.txt");
    f << "data   Q" << 32 - D::FRACTIONAL_BITS << "." << D::FRACTIONAL_BITS << "  (x, r, g, rhs, u, z, y, ZMIN, ZMAX)\n"
      << "W      Q" << 32 - W::FRACTIONAL_BITS << "." << W::FRACTIONAL_BITS << "\n"
      << "MX     Q" << 32 - X::FRACTIONAL_BITS << "." << X::FRACTIONAL_BITS << "\n"
      << "MR     Q" << 32 - R::FRACTIONAL_BITS << "." << R::FRACTIONAL_BITS << "\n"
      << "PARAMS Q" << 32 - P::FRACTIONAL_BITS << "." << P::FRACTIONAL_BITS << "  (rho, alpha, 1-alpha)\n"
      << "N_ITER " << N_ITER << "\nX_SCALE";
    for (int i = 0; i < NX; ++i) f << " " << mpc.solver.qp.x_scale[i];
    f << "\nR_SCALE";
    for (int i = 0; i < NREF; ++i) f << " " << mpc.solver.qp.r_scale[i];
    f << "  (x_ref y_ref z_ref psi_ref, repeated for p = 0..NP-1)\nU_SCALE";
    for (int i = 0; i < NU; ++i) f << " " << std::setprecision(17) << mpc.solver.qp.u_scale[i];
    f << "  (u_k = U_SCALE_k * z_k, F_total += HOVER_THRUST)\n";
}

void write_raw_header(std::ofstream& f) {
    f << "Cycle,Scenario";
    for (int i = 0; i < NX; ++i) f << ",x" << i;
    for (int p = 0; p < NP; ++p)
        for (int c = 0; c < NREF; ++c) f << ",r" << p << "_" << REF_TAG[c];
    for (int i = 0; i < NU_TOTAL; ++i) f << ",g" << i;
    for (int i = 0; i < NU_TOTAL; ++i) f << ",z" << i;
    f << "\n";
}

// Inputs are written exactly as the accelerator receives them (after the power-of-two input scaling).
template <class Fmt, typename T = typename Fmt::data>
void write_raw_row(std::ofstream& f, int cycle, const std::string& sc, const MpcController<Fmt>& mpc,
                   const double x[NX], const double r[NR], const T g[NU_TOTAL], const T z[NU_TOTAL]) {
    T xq[NX], rq[NR];
    mpc.stage1_acquire_state(x, xq);
    mpc.stage1_acquire_reference(r, rq);
    f << cycle << "," << sc;
    for (int i = 0; i < NX; ++i) f << "," << xq[i].raw_value;
    for (int k = 0; k < NR; ++k) f << "," << rq[k].raw_value;
    for (int i = 0; i < NU_TOTAL; ++i) f << "," << g[i].raw_value;
    for (int i = 0; i < NU_TOTAL; ++i) f << "," << z[i].raw_value;
    f << "\n";
}

bool at_bound(const double z[NU_TOTAL]) {
    for (int i = 0; i < NU_TOTAL; ++i) {
        const double tol = 1e-9 * QP_DEPLOYED.z_max[i];
        if (z[i] <= QP_DEPLOYED.z_min[i] + tol || z[i] >= QP_DEPLOYED.z_max[i] - tol) return true;
    }
    return false;
}

int main() {
    std::filesystem::create_directories("results");
    std::filesystem::create_directories("test_vectors");

    const MpcController<FmtDouble> mpc_f;
    const MpcController<FmtQ16_16> mpc_16;
    const MpcController<FmtQ8_24> mpc_24;
    const AdmmSolver<FmtDouble> exact_solver;

    write_rom("test_vectors/rom_q16_16", mpc_16);
    write_rom("test_vectors/rom_q8_24", mpc_24);

    std::ofstream tv("test_vectors/test_vectors.csv");
    std::ofstream raw16("test_vectors/test_vectors_q16_16.csv");
    std::ofstream raw24("test_vectors/test_vectors_q8_24.csv");
    tv << std::setprecision(std::numeric_limits<double>::max_digits10);
    tv << "Cycle,Scenario,t";
    for (int i = 0; i < NX; ++i) tv << ",x" << i;
    for (int p = 0; p < NP; ++p)
        for (int c = 0; c < NREF; ++c) tv << ",r" << p << "_" << REF_TAG[c];
    for (const char* tag : {"u_exact", "u_float", "u_q16_", "u_q824_"})
        for (int c = 0; c < NU; ++c) tv << "," << tag << c;
    tv << ",Constraint_Active\n";
    write_raw_header(raw16);
    write_raw_header(raw24);

    std::vector<Report> reports;
    int cycle = 0;

    for (const Scenario& sc : golden_scenarios()) {
        Report rep;
        rep.name = sc.name;
        Q16_16::overflow_count = 0;
        Q8_24::overflow_count = 0;

        std::ofstream log("results/" + sc.name + "_mpc.csv");
        log << std::setprecision(std::numeric_limits<double>::max_digits10);
        log << "time,x_ref,y_ref,z_ref,psi_ref,x,y,z,phi,theta,psi,p,q,r";
        for (const char* tag : {"u_exact", "u_float", "u_q16_", "u_q824_"})
            for (int c = 0; c < NU; ++c) log << "," << tag << c;
        log << ",Constraint_Active\n";

        QuadState x = sc.x0;
        const int n = sc.num_steps();
        for (int k = 0; k < n; ++k, ++cycle) {
            const double t = k * TS;
            double r[NR];
            sc.horizon(t, r);

            double u_f[NU], u_16[NU], u_24[NU], u_ex[NU];
            double g_f[NU_TOTAL], z_ex[NU_TOTAL];
            Q16_16 g16[NU_TOTAL], z16[NU_TOTAL];
            Q8_24 g24[NU_TOTAL], z24[NU_TOTAL];
            mpc_f.compute(x.val.data(), r, u_f, g_f);
            mpc_16.compute(x.val.data(), r, u_16, g16, z16);
            mpc_24.compute(x.val.data(), r, u_24, g24, z24);
            rep.max_exact_iter = std::max(rep.max_exact_iter,
                                          exact_solver.solve_to_tolerance(g_f, z_ex, EXACT_TOL, EXACT_MAX_ITER));
            mpc_f.stage4_optimal_control(z_ex, u_ex);

            const bool active = at_bound(z_ex);
            rep.active_cycles += active;
            rep.algo.add(u_f, u_ex);
            rep.q16.add(u_16, u_f);
            rep.q824.add(u_24, u_f);

            tv << cycle << "," << sc.name << "," << t;
            for (int i = 0; i < NX; ++i) tv << "," << x[i];
            for (int j = 0; j < NR; ++j) tv << "," << r[j];
            for (const double* u : {u_ex, u_f, u_16, u_24})
                for (int c = 0; c < NU; ++c) tv << "," << u[c];
            tv << "," << active << "\n";
            write_raw_row(raw16, cycle, sc.name, mpc_16, x.val.data(), r, g16, z16);
            write_raw_row(raw24, cycle, sc.name, mpc_24, x.val.data(), r, g24, z24);

            log << t;
            for (int c = 0; c < NREF; ++c) log << "," << r[c];
            for (int i : {0, 1, 2, 6, 7, 8, 9, 10, 11}) log << "," << x[i];
            for (const double* u : {u_ex, u_f, u_16, u_24})
                for (int c = 0; c < NU; ++c) log << "," << u[c];
            log << "," << active << "\n";

            x = quadcopter_step(x, QuadControl{u_f[0], u_f[1], u_f[2], u_f[3]}, TS);
        }
        rep.cycles = n;

        rep.trk_float = closed_loop(sc, mpc_f);
        rep.trk_q16 = closed_loop(sc, mpc_16);
        rep.trk_q824 = closed_loop(sc, mpc_24);
        rep.ovf_q16 = Q16_16::overflow_count;
        rep.ovf_q824 = Q8_24::overflow_count;
        reports.push_back(rep);
    }

    const std::string line(104, '-');
    std::cout << std::string(104, '=') << "\n GOLDEN MODEL MPC (ADMM, N_ITER = " << N_ITER << ", rho(unit) = " << RHO_UNIT
              << ", alpha = " << ALPHA << ")  --  error dinormalisasi terhadap batas aktuator\n"
              << std::string(104, '=') << "\n";

    std::cout << "\n1. TRACKING CLOSED-LOOP (RMSE posisi, m) -- tiap format menjalankan plant-nya sendiri\n" << line << "\n";
    std::cout << std::left << std::setw(12) << "Skenario" << std::setw(31) << "Float  (x / y / z)"
              << std::setw(31) << "Q16.16 (x / y / z)" << "Q8.24  (x / y / z)\n" << line << "\n";
    auto trk = [](const Tracking& t) {
        std::ostringstream s;
        s << std::fixed << std::setprecision(4) << t.rmse[0] << " / " << t.rmse[1] << " / " << t.rmse[2];
        return s.str();
    };
    for (const auto& r : reports)
        std::cout << std::left << std::setw(12) << r.name << std::setw(31) << trk(r.trk_float)
                  << std::setw(31) << trk(r.trk_q16) << trk(r.trk_q824) << "\n";

    std::cout << "\n2. AKURASI PER SIKLUS (max |du| / u_max, RMS dalam kurung)\n" << line << "\n";
    std::cout << std::left << std::setw(12) << "Skenario" << std::setw(10) << "Aktif" << std::setw(26)
              << "Float(N) vs Eksak" << std::setw(26) << "Q16.16 vs Float(N)" << "Q8.24 vs Float(N)\n" << line << "\n";
    auto err = [](const ErrStats& e) {
        std::ostringstream s;
        s << std::scientific << std::setprecision(2) << e.worst() << " (" << e.rms() << ")";
        return s.str();
    };
    for (const auto& r : reports)
        std::cout << std::left << std::setw(12) << r.name << std::setw(10)
                  << (std::to_string(r.active_cycles) + "/" + std::to_string(r.cycles)) << std::setw(26)
                  << err(r.algo) << std::setw(26) << err(r.q16) << err(r.q824) << "\n";

    std::cout << "\n3. OVERFLOW / SATURASI FIXED-POINT (semua operasi, termasuk nilai antara)\n" << line << "\n";
    for (const auto& r : reports)
        std::cout << std::left << std::setw(12) << r.name << "Q16.16: " << std::setw(10) << r.ovf_q16
                  << "Q8.24: " << r.ovf_q824 << "\n";

    std::cout << "\n4. VERDIK (toleransi algoritma " << TOL_ALGO << ", kuantisasi " << TOL_QUANT << ", overflow 0)\n" << line << "\n";
    bool all16 = true, all24 = true;
    for (const auto& r : reports) {
        const bool algo_ok = r.algo.worst() <= TOL_ALGO;
        const bool ok16 = algo_ok && r.q16.worst() <= TOL_QUANT && r.ovf_q16 == 0;
        const bool ok24 = algo_ok && r.q824.worst() <= TOL_QUANT && r.ovf_q824 == 0;
        all16 &= ok16;
        all24 &= ok24;
        std::cout << std::left << std::setw(12) << r.name << "Algoritma: " << std::setw(6) << (algo_ok ? "PASS" : "FAIL")
                  << "Q16.16: " << std::setw(6) << (ok16 ? "PASS" : "FAIL") << "Q8.24: " << (ok24 ? "PASS" : "FAIL")
                  << "   (iterasi solusi eksak maks: " << r.max_exact_iter << ")\n";
    }
    std::cout << line << "\nKESELURUHAN  Q16.16: " << (all16 ? "PASS" : "FAIL") << "   Q8.24: " << (all24 ? "PASS" : "FAIL") << "\n";

    std::ofstream sum("results/golden_summary.csv");
    sum << std::setprecision(6);
    sum << "Scenario,Cycles,Active_Cycles,RMSE_x_float,RMSE_y_float,RMSE_z_float,RMSE_x_q16,RMSE_y_q16,RMSE_z_q16,"
           "RMSE_x_q824,RMSE_y_q824,RMSE_z_q824,MaxErr_float_vs_exact,RmsErr_float_vs_exact,MaxErr_q16_vs_float,"
           "RmsErr_q16_vs_float,MaxErr_q824_vs_float,RmsErr_q824_vs_float,Overflow_q16,Overflow_q824\n";
    for (const auto& r : reports) {
        sum << r.name << "," << r.cycles << "," << r.active_cycles;
        for (const Tracking* t : {&r.trk_float, &r.trk_q16, &r.trk_q824})
            for (double v : t->rmse) sum << "," << v;
        for (const ErrStats* e : {&r.algo, &r.q16, &r.q824}) sum << "," << e->worst() << "," << e->rms();
        sum << "," << r.ovf_q16 << "," << r.ovf_q824 << "\n";
    }

    std::cout << "\nTest vectors: test_vectors/test_vectors.csv, test_vectors_q16_16.csv, test_vectors_q8_24.csv ("
              << cycle << " siklus)\nROM BRAM   : test_vectors/rom_q16_16/*.mem, test_vectors/rom_q8_24/*.mem\n"
              << "Ringkasan  : results/golden_summary.csv\n";
    return 0;
}

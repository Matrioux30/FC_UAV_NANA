// PS-side driver for mpc_accel_axi on KV260 (Cortex-A53, Ubuntu). Maps the AXI4-Lite registers through /dev/mem.
// Stage 1 (Current State) and Stage 4 (Optimal Control) run here; Prediction + Optimization run in the PL.
#pragma once

#include "mpc_formats.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

class MpcAccel {
public:
    static constexpr uint64_t BASE = 0xA0000000;   // M_AXI_HPM0_FPD, see vivado/build.tcl
    static constexpr size_t   SPAN = 0x1000;
    static constexpr int NX = 12, NR = 80, NIN = NX + NR, NL = 20, NU = 4;

    enum Reg : uint32_t {
        CTRL = 0x000, STATUS = 0x004, N_ITER = 0x008, CYCLES = 0x00C, ID = 0x010, CONFIG = 0x014,
        IN = 0x400, Z = 0x800, G = 0x900
    };
    static constexpr uint32_t ID_VALUE = 0x4D504331;   // "MPC1"
    static constexpr uint32_t ST_BUSY = 1u << 0, ST_DONE = 1u << 1, ST_SAT = 1u << 2;

    explicit MpcAccel(uint64_t base = BASE) {
        fd_ = ::open("/dev/mem", O_RDWR | O_SYNC);
        if (fd_ < 0) throw std::runtime_error("open /dev/mem failed (run with sudo)");
        void* p = ::mmap(nullptr, SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, static_cast<off_t>(base));
        if (p == MAP_FAILED) throw std::runtime_error("mmap failed");
        regs_ = static_cast<volatile uint32_t*>(p);
        if (rd(ID) != ID_VALUE) {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "ID register = 0x%08X, expected 0x%08X (bitstream not loaded?)",
                          rd(ID), ID_VALUE);
            throw std::runtime_error(buf);
        }
        const uint32_t cfg = rd(CONFIG);
        for (const MpcFormat& f : MPC_FORMATS) {
            if ((cfg & 0xFF) == uint32_t(f.frac_w) && ((cfg >> 8) & 0xFF) == uint32_t(f.frac_mx) &&
                ((cfg >> 16) & 0xFF) == uint32_t(f.frac_mr) && ((cfg >> 24) & 0xFF) == uint32_t(f.frac_p))
                fmt_ = &f;
        }
        if (!fmt_) throw std::runtime_error("CONFIG register does not match a known number format");
    }
    ~MpcAccel() {
        if (regs_) ::munmap(const_cast<uint32_t*>(regs_), SPAN);
        if (fd_ >= 0) ::close(fd_);
    }
    MpcAccel(const MpcAccel&) = delete;
    MpcAccel& operator=(const MpcAccel&) = delete;

    const MpcFormat& format() const { return *fmt_; }
    uint32_t rd(uint32_t off) const { return regs_[off / 4]; }
    void wr(uint32_t off, uint32_t v) { regs_[off / 4] = v; }

    void set_iterations(uint32_t n) { wr(N_ITER, n); }
    void write_inputs(const int32_t in[NIN]) {
        for (int k = 0; k < NIN; ++k) wr(IN + 4 * k, static_cast<uint32_t>(in[k]));
    }
    // AXI reads may overtake the START write, so a poll could see the previous solve's DONE. DONE is cleared by
    // START and set >= 2.7k PL cycles later; seeing it low proves START has landed.
    void start() {
        wr(CTRL, 1);
#ifdef __aarch64__
        asm volatile("dsb sy" ::: "memory");
#endif
        for (int i = 0; i < 1000 && (rd(STATUS) & ST_DONE); ++i) {}
    }
    // Polls STATUS.DONE. Returns false on timeout.
    bool wait_done(long timeout_us = 10000) const {
        timespec t0, t;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        while (!(rd(STATUS) & ST_DONE)) {
            clock_gettime(CLOCK_MONOTONIC, &t);
            if ((t.tv_sec - t0.tv_sec) * 1000000L + (t.tv_nsec - t0.tv_nsec) / 1000 > timeout_us) return false;
        }
        return true;
    }
    void read_z(int32_t z[NL]) const { for (int i = 0; i < NL; ++i) z[i] = static_cast<int32_t>(rd(Z + 4 * i)); }
    void read_g(int32_t g[NL]) const { for (int i = 0; i < NL; ++i) g[i] = static_cast<int32_t>(rd(G + 4 * i)); }
    bool saturated() const { return rd(STATUS) & ST_SAT; }
    uint32_t cycles() const { return rd(CYCLES); }

    // Full control step. x: state [x y z u v w phi theta psi p q r], ref: NP x [x_ref y_ref z_ref psi_ref],
    // u_out: [F_total, tau_phi, tau_theta, tau_psi]. Same scaling and rounding as mpc_controller.h.
    bool compute(const double x[NX], const double ref[NR], double u_out[NU]) {
        int32_t in[NIN], z[NL];
        for (int i = 0; i < NX; ++i) in[i] = quantize(x[i] * fmt_->x_scale[i]);
        for (int k = 0; k < NR; ++k) in[NX + k] = quantize(ref[k] * fmt_->r_scale[k % 4]);
        write_inputs(in);
        start();
        if (!wait_done()) return false;
        read_z(z);
        const double lsb = std::ldexp(1.0, -fmt_->frac_data);
        for (int i = 0; i < NU; ++i) u_out[i] = fmt_->u_scale[i] * (z[i] * lsb);
        u_out[0] += HOVER_THRUST;
        return true;
    }

private:
    // round half away from zero (std::round) and saturate, as FixedPoint(double)
    int32_t quantize(double v) const {
        const double s = std::round(std::ldexp(v, fmt_->frac_data));
        if (s > 2147483647.0) return INT32_MAX;
        if (s < -2147483648.0) return INT32_MIN;
        return static_cast<int32_t>(s);
    }

    int fd_ = -1;
    volatile uint32_t* regs_ = nullptr;
    const MpcFormat* fmt_ = nullptr;
};

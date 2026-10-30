#ifndef FIXED_POINT_H
#define FIXED_POINT_H

#include <cstdint>
#include <cmath>

namespace quad {

#ifdef __SIZEOF_INT128__
__extension__ using wide_acc_t = __int128;
#else
using wide_acc_t = int64_t;
#endif

// Signed 32-bit Q(INT_BITS).(FRAC_BITS). Arithmetic spec the RTL must match bit-exactly:
//  - add/sub: exact, then saturate to int32
//  - mul:     full 64-bit product, round half up (+2^(F-1)) then arithmetic shift right by F, saturate
//  - mul/dot with a coefficient in another Q format (Fc fractional bits): products summed exactly in a wide
//             accumulator, then one round half up + shift right by Fc + saturate to the data format
template <int INT_BITS, int FRAC_BITS>
class FixedPoint {
public:
    static_assert(INT_BITS + FRAC_BITS == 32, "Total bits must be 32 (signed int32_t)");
    static constexpr int FRACTIONAL_BITS = FRAC_BITS;
    static constexpr double SCALE = static_cast<double>(1ULL << FRAC_BITS);
    static constexpr int32_t MAX_VAL = 0x7FFFFFFF;
    static constexpr int32_t MIN_VAL = -0x7FFFFFFF - 1;
    static constexpr double MAX_REAL = MAX_VAL / SCALE;

    // Number of saturations since the last reset (any operation, including intermediates).
    static inline long long overflow_count = 0;

    int32_t raw_value;

    FixedPoint() : raw_value(0) {}
    explicit FixedPoint(double val) {
        const double scaled = std::round(val * SCALE);
        if (scaled > static_cast<double>(MAX_VAL)) {
            raw_value = MAX_VAL;
            ++overflow_count;
        } else if (scaled < static_cast<double>(MIN_VAL)) {
            raw_value = MIN_VAL;
            ++overflow_count;
        } else {
            raw_value = static_cast<int32_t>(scaled);
        }
    }

    static FixedPoint from_raw(int32_t raw) {
        FixedPoint f;
        f.raw_value = raw;
        return f;
    }

    double to_double() const { return static_cast<double>(raw_value) / SCALE; }

    FixedPoint operator+(const FixedPoint& o) const {
        return from_raw(saturate(static_cast<int64_t>(raw_value) + o.raw_value));
    }
    FixedPoint operator-(const FixedPoint& o) const {
        return from_raw(saturate(static_cast<int64_t>(raw_value) - o.raw_value));
    }
    FixedPoint operator-() const { return from_raw(saturate(-static_cast<int64_t>(raw_value))); }
    FixedPoint operator*(const FixedPoint& o) const {
        return from_wide(static_cast<wide_acc_t>(raw_value) * o.raw_value, FRAC_BITS);
    }
    FixedPoint& operator+=(const FixedPoint& o) { return *this = *this + o; }
    FixedPoint& operator-=(const FixedPoint& o) { return *this = *this - o; }

    bool operator<(const FixedPoint& o) const { return raw_value < o.raw_value; }
    bool operator>(const FixedPoint& o) const { return raw_value > o.raw_value; }

    // Round half up, arithmetic shift right by `shift`, saturate: the single rounding point of every MAC.
    static FixedPoint from_wide(wide_acc_t v, int shift) {
        return from_raw(saturate((v + (static_cast<wide_acc_t>(1) << (shift - 1))) >> shift));
    }

private:
    static int32_t saturate(wide_acc_t v) {
        if (v > MAX_VAL) { ++overflow_count; return MAX_VAL; }
        if (v < MIN_VAL) { ++overflow_count; return MIN_VAL; }
        return static_cast<int32_t>(v);
    }
};

using Q16_16 = FixedPoint<16, 16>;
using Q8_24  = FixedPoint<8, 24>;

template <typename T> inline T from_real(double v) { return T(v); }
template <> inline double from_real<double>(double v) { return v; }

inline double to_real(double v) { return v; }
template <int I, int F> inline double to_real(const FixedPoint<I, F>& v) { return v.to_double(); }

// Coefficient (any Q format) times data: result in the data format, rounded once.
inline double mul(double c, double d) { return c * d; }
template <int IC, int FC, int ID, int FD>
inline FixedPoint<ID, FD> mul(const FixedPoint<IC, FC>& c, const FixedPoint<ID, FD>& d) {
    return FixedPoint<ID, FD>::from_wide(static_cast<wide_acc_t>(c.raw_value) * d.raw_value, FC);
}

inline double dot(const double* c, const double* d, int n) {
    double s = 0.0;
    for (int i = 0; i < n; ++i) s += c[i] * d[i];
    return s;
}
template <int IC, int FC, int ID, int FD>
inline FixedPoint<ID, FD> dot(const FixedPoint<IC, FC>* c, const FixedPoint<ID, FD>* d, int n) {
    wide_acc_t acc = 0;
    for (int i = 0; i < n; ++i) acc += static_cast<wide_acc_t>(c[i].raw_value) * d[i].raw_value;
    return FixedPoint<ID, FD>::from_wide(acc, FC);
}

} // namespace quad

#endif // FIXED_POINT_H

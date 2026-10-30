#ifndef NUMBER_FORMATS_H
#define NUMBER_FORMATS_H

#include "fixed_point.h"
#include "mpc_constants.h"

namespace quad {

// Data path type plus one coefficient type per ROM, and the QP scaling that matches them.
template <class Data, class CoefW, class CoefMx, class CoefMr, class CoefP>
struct NumFormat {
    using data = Data;
    using coef_w = CoefW;
    using coef_mx = CoefMx;
    using coef_mr = CoefMr;
    using coef_p = CoefP;
};

template <int F> using Coef = FixedPoint<32 - F, F>;

struct FmtDouble : NumFormat<double, double, double, double, double> {
    static const QpConstants& qp() { return QP_DEPLOYED; }
    static constexpr const char* name = "Float";
};

struct FmtQ16_16 : NumFormat<Q16_16, Coef<qp_q16_16::W_FRAC>, Coef<qp_q16_16::MX_FRAC>,
                             Coef<qp_q16_16::MR_FRAC>, Coef<qp_q16_16::P_FRAC>> {
    static const QpConstants& qp() { return QP_Q16_16; }
    static constexpr const char* name = "Q16.16";
};

struct FmtQ8_24 : NumFormat<Q8_24, Coef<qp_q8_24::W_FRAC>, Coef<qp_q8_24::MX_FRAC>,
                            Coef<qp_q8_24::MR_FRAC>, Coef<qp_q8_24::P_FRAC>> {
    static const QpConstants& qp() { return QP_Q8_24; }
    static constexpr const char* name = "Q8.24";
};

} // namespace quad

#endif // NUMBER_FORMATS_H

#include "pid_controller.h"
#include <algorithm>

namespace quad {

PID::PID() : Kp_(1.0), Ki_(0.0), Kd_(0.0), Ts_(0.005),
             u_min_(-1.0), u_max_(1.0), integral_(0.0), prev_error_(0.0) {}

PID::PID(double Kp, double Ki, double Kd, double Ts, double u_min, double u_max)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd), Ts_(Ts), u_min_(u_min), u_max_(u_max),
      integral_(0.0), prev_error_(0.0) {}

double PID::update(double ref, double meas) {
    const double e = ref - meas;

    const double p_term = Kp_ * e;
    const double i_candidate = integral_ + Ki_ * Ts_ * e;
    const double d_term = Kd_ * (e - prev_error_) / Ts_;

    const double u_unsat = p_term + i_candidate + d_term;
    double u_sat = u_unsat;
    if (u_sat > u_max_) u_sat = u_max_;
    else if (u_sat < u_min_) u_sat = u_min_;

    if (u_unsat == u_sat) {
        integral_ = i_candidate;
    }

    const double int_max = 0.05;
    if (integral_ > int_max) {
        integral_ = int_max;
    } else if (integral_ < -int_max) {
        integral_ = -int_max;
    }

    prev_error_ = e;
    return u_sat;
}

double PID::update_with_rate(double ref, double meas, double rate) {
    const double e = ref - meas;

    const double p_term = Kp_ * e;
    const double i_candidate = integral_ + Ki_ * Ts_ * e;
    // Gunakan rate (kecepatan sudut/linier) langsung sebagai turunan dari measurement
    // d/dt (ref - meas) = 0 - d(meas)/dt = -rate
    const double d_term = -Kd_ * rate;

    const double u_unsat = p_term + i_candidate + d_term;
    double u_sat = u_unsat;
    if (u_sat > u_max_) u_sat = u_max_;
    else if (u_sat < u_min_) u_sat = u_min_;

    if (u_unsat == u_sat) {
        integral_ = i_candidate;
    }

    const double int_max = 0.05;
    if (integral_ > int_max) {
        integral_ = int_max;
    } else if (integral_ < -int_max) {
        integral_ = -int_max;
    }

    prev_error_ = e;
    return u_sat;
}

void PID::setGains(double Kp, double Ki, double Kd) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}

void PID::setLimits(double u_min, double u_max) {
    u_min_ = u_min;
    u_max_ = u_max;
}

void PID::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
}

} // namespace quad

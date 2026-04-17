#pragma once

#include <cmath>
#include <stdexcept>
#include <utility>

#include "runge_kutta/rk4.hpp"

namespace runge_kutta {

template <class State, class F>
State integrate(F&& f, double t0, State y0, double t_end, double h) {
    if (!(h > 0.0)) {
        throw std::invalid_argument(
            "runge_kutta::integrate: step size h must be > 0");
    }
    if (t_end < t0) {
        throw std::invalid_argument(
            "runge_kutta::integrate: t_end must be >= t0");
    }

    const double total = t_end - t0;
    const double tol = 1e-9 * std::max(1.0, std::abs(total));

    long n_full = static_cast<long>(total / h);
    double remaining = total - static_cast<double>(n_full) * h;
    if (remaining > h - tol) {
        n_full += 1;
        remaining = 0.0;
    }

    State y = std::move(y0);
    double t = t0;
    for (long i = 0; i < n_full; ++i) {
        y = RK4<State>::step(f, t, y, h);
        t = t0 + static_cast<double>(i + 1) * h;
    }
    if (remaining > tol) {
        y = RK4<State>::step(f, t, y, remaining);
    }
    return y;
}

}  // namespace runge_kutta

#pragma once

#include <cstddef>
#include <stdexcept>

namespace runge_kutta {
namespace detail {

template <class State>
inline State scale(const State& a, double s) {
    State out = a;
    const std::size_t n = static_cast<std::size_t>(a.size());
    for (std::size_t i = 0; i < n; ++i) out[i] = s * a[i];
    return out;
}

template <class State>
inline State add(const State& a, const State& b) {
    State out = a;
    const std::size_t n = static_cast<std::size_t>(a.size());
    for (std::size_t i = 0; i < n; ++i) out[i] = a[i] + b[i];
    return out;
}

template <class State>
inline State axpy(const State& y, double a, const State& x) {
    State out = y;
    const std::size_t n = static_cast<std::size_t>(y.size());
    for (std::size_t i = 0; i < n; ++i) out[i] = y[i] + a * x[i];
    return out;
}

}  // namespace detail

template <class State>
class RK4 {
public:
    template <class F>
    static State step(F&& f, double t, const State& y, double h) {
        if (!(h > 0.0)) {
            throw std::invalid_argument(
                "runge_kutta::RK4::step: step size h must be > 0");
        }
        const State k1 = f(t, y);
        const State k2 = f(t + 0.5 * h, detail::axpy(y, 0.5 * h, k1));
        const State k3 = f(t + 0.5 * h, detail::axpy(y, 0.5 * h, k2));
        const State k4 = f(t + h,       detail::axpy(y, h,       k3));

        State sum = detail::add(k1, detail::scale(k2, 2.0));
        sum = detail::add(sum, detail::scale(k3, 2.0));
        sum = detail::add(sum, k4);
        return detail::axpy(y, h / 6.0, sum);
    }
};

}  // namespace runge_kutta

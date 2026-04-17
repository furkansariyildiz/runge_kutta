#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "runge_kutta/integrate.hpp"

using State = std::vector<double>;
using runge_kutta::integrate;

static double decay_error(double h) {
    auto f = [](double, const State& y) -> State { return State{-y[0]}; };
    State y0{1.0};
    State y_end = integrate<State>(f, 0.0, y0, 1.0, h);
    return std::abs(y_end[0] - std::exp(-1.0));
}

TEST(Convergence, ExponentialDecayFourthOrder) {
    const double h = 0.1;
    const double e_h     = decay_error(h);
    const double e_h_2   = decay_error(h / 2.0);
    const double e_h_4   = decay_error(h / 4.0);

    const double ratio1 = e_h   / e_h_2;
    const double ratio2 = e_h_2 / e_h_4;

    EXPECT_GT(ratio1, 12.0);
    EXPECT_LT(ratio1, 20.0);
    EXPECT_GT(ratio2, 12.0);
    EXPECT_LT(ratio2, 20.0);
}

static double oscillator_error(double h) {
    auto f = [](double, const State& y) -> State {
        return State{y[1], -y[0]};
    };
    State y0{1.0, 0.0};
    const double t_end = 2.0 * M_PI;
    State y_end = integrate<State>(f, 0.0, y0, t_end, h);
    // Analytic: y(2π) = (1, 0)
    const double dx = y_end[0] - 1.0;
    const double dv = y_end[1] - 0.0;
    return std::sqrt(dx * dx + dv * dv);
}

TEST(Convergence, HarmonicOscillatorFourthOrder) {
    const double h = 0.05;
    const double e_h   = oscillator_error(h);
    const double e_h_2 = oscillator_error(h / 2.0);
    const double ratio = e_h / e_h_2;
    EXPECT_GT(ratio, 12.0);
    EXPECT_LT(ratio, 20.0);
}

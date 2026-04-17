#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "runge_kutta/rk4.hpp"

using runge_kutta::RK4;

TEST(RK4Step, HarmonicOscillatorStdArray) {
    using State = std::array<double, 2>;
    auto f = [](double, const State& y) -> State {
        return State{y[1], -y[0]};
    };
    State y0{1.0, 0.0};
    const double h = 0.1;

    State y1 = RK4<State>::step(f, 0.0, y0, h);

    EXPECT_NEAR(y1[0], std::cos(h), 1e-6);
    EXPECT_NEAR(y1[1], -std::sin(h), 1e-6);
}

TEST(RK4Step, ExponentialDecayStdVector) {
    using State = std::vector<double>;
    auto f = [](double, const State& y) -> State {
        return State{-y[0]};
    };
    State y0{1.0};
    const double h = 0.05;

    State y1 = RK4<State>::step(f, 0.0, y0, h);

    EXPECT_NEAR(y1[0], std::exp(-h), 1e-8);
}

TEST(RK4Step, RejectsZeroStep) {
    using State = std::vector<double>;
    auto f = [](double, const State& y) -> State { return y; };
    State y0{1.0, 2.0};
    EXPECT_THROW(RK4<State>::step(f, 0.0, y0, 0.0), std::invalid_argument);
}

TEST(RK4Step, RejectsNegativeStep) {
    using State = std::vector<double>;
    auto f = [](double, const State& y) -> State { return y; };
    State y0{1.0};
    EXPECT_THROW(RK4<State>::step(f, 0.0, y0, -0.01), std::invalid_argument);
}

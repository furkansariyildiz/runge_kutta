#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "runge_kutta/integrate.hpp"

using State = std::vector<double>;
using runge_kutta::integrate;

TEST(Integrate, ExponentialDecayEvenStepCount) {
    auto f = [](double, const State& y) -> State { return State{-y[0]}; };
    State y0{1.0};
    State y_end = integrate<State>(f, 0.0, y0, 1.0, 0.1);
    EXPECT_NEAR(y_end[0], std::exp(-1.0), 1e-5);
}

TEST(Integrate, ExponentialDecayUnevenStepCount) {
    auto f = [](double, const State& y) -> State { return State{-y[0]}; };
    State y0{1.0};
    // h=0.3 should give 3 full steps + 1 clamped step of 0.1
    State y_end = integrate<State>(f, 0.0, y0, 1.0, 0.3);
    EXPECT_NEAR(y_end[0], std::exp(-1.0), 1e-3);
}

TEST(Integrate, StepCountMatchesEvenCase) {
    int f_calls = 0;
    auto f = [&](double, const State& y) -> State {
        ++f_calls;
        return State{-y[0]};
    };
    State y0{1.0};
    integrate<State>(f, 0.0, y0, 1.0, 0.1);
    // RK4 calls f 4 times per step. With exactly 10 steps, expect 40 calls.
    EXPECT_EQ(f_calls, 40);
}

TEST(Integrate, StepCountMatchesUnevenCase) {
    int f_calls = 0;
    auto f = [&](double, const State& y) -> State {
        ++f_calls;
        return State{-y[0]};
    };
    State y0{1.0};
    integrate<State>(f, 0.0, y0, 1.0, 0.3);
    // 3 full + 1 partial = 4 RK4 steps -> 16 f calls
    EXPECT_EQ(f_calls, 16);
}

TEST(Integrate, RejectsZeroStep) {
    auto f = [](double, const State& y) -> State { return y; };
    State y0{1.0};
    EXPECT_THROW(integrate<State>(f, 0.0, y0, 1.0, 0.0), std::invalid_argument);
}

TEST(Integrate, RejectsNegativeStep) {
    auto f = [](double, const State& y) -> State { return y; };
    State y0{1.0};
    EXPECT_THROW(integrate<State>(f, 0.0, y0, 1.0, -0.1), std::invalid_argument);
}

TEST(Integrate, RejectsEndBeforeStart) {
    auto f = [](double, const State& y) -> State { return y; };
    State y0{1.0};
    EXPECT_THROW(integrate<State>(f, 1.0, y0, 0.0, 0.1), std::invalid_argument);
}

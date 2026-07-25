# runge_kutta

A small, header-only ROS1 (catkin) library providing a classical 4th-order Runge-Kutta (RK4) integrator, templated on the state type. It ships as two headers you can drop into any node — `RK4<State>::step(...)` for a single integration step, and `runge_kutta::integrate(...)` for driving a state from `t0` to `t_end` at a fixed step size — plus a runnable demo node, launch file, and gtest suite.

## Adding the dependency

Add the package to your consumer's `package.xml`:

```xml
<depend>runge_kutta</depend>
```

And list it as a catkin component in `CMakeLists.txt`:

```cmake
find_package(catkin REQUIRED COMPONENTS
  runge_kutta
  # ...your other components
)
```

`runge_kutta` is header-only: `catkin_package()` exports `INCLUDE_DIRS include` but deliberately has **no `LIBRARIES` entry** — there is nothing to link. Once `${catkin_INCLUDE_DIRS}` is on your include path (the usual `include_directories(${catkin_INCLUDE_DIRS})`), just include the headers:

```cpp
#include <runge_kutta/rk4.hpp>        // RK4<State>::step
#include <runge_kutta/integrate.hpp>  // runge_kutta::integrate
// or:
#include <runge_kutta/runge_kutta.hpp>  // both, in one include
```

No `target_link_libraries(your_target runge_kutta)` is needed for `runge_kutta` itself — `${catkin_LIBRARIES}` won't contain an entry for it, and that's expected for a header-only package.

## API

### `RK4<State>::step` — single step

```cpp
runge_kutta::RK4<State>::step(f, t, y, h)
```

Advances state `y` at time `t` forward by one step of size `h`, given a derivative function `f(t, y) -> State` (any callable — lambda, function pointer, functor). Throws `std::invalid_argument` if `h <= 0`. Use this when you own the time loop yourself, e.g. inside a `ros::Timer` callback:

```cpp
using State = std::vector<double>;

// dx/dt = v, dv/dt = -stiffness*x - damping*v
auto f = [damping, stiffness](double /*t*/, const State& y) {
    return State{y[1], -stiffness * y[0] - damping * y[1]};
};

State y = {1.0, 0.0};
double t = 0.0;
double h = 0.01;

y = runge_kutta::RK4<State>::step(f, t, y, h);
t += h;
```

### `runge_kutta::integrate` — driver to a target time

```cpp
runge_kutta::integrate(f, t0, y0, t_end, h)
```

Repeatedly calls `RK4<State>::step` to advance `y0` from `t0` to `t_end` using fixed step size `h`. If `(t_end - t0)` isn't an exact multiple of `h`, the final sub-step is clamped so the result lands exactly on `t_end` rather than overshooting it — you don't need to compute a remainder step yourself. Throws `std::invalid_argument` if `h <= 0` or `t_end < t0`.

```cpp
State y0 = {1.0, 0.0};
State y_final = runge_kutta::integrate(f, 0.0, y0, 5.0, 0.01);
```

### Supported state types

`State` can be `std::array<double, N>`, `std::vector<double>`, `Eigen::VectorXd`, or any type that supports index access (`operator[]`), a `size()` method, element-wise addition, and scalar multiplication — that's all the integrator needs to combine the four stage evaluations.

## The math: how RK4 works

Classical RK4 estimates the slope of the solution at four points across the step and combines them into a weighted average. Given `f(t, y) = dy/dt` and a step size `h`, computing `y_{n+1}` from `y_n` at time `t`:

```
k1 = f(t,       y_n)
k2 = f(t + h/2, y_n + (h/2) * k1)
k3 = f(t + h/2, y_n + (h/2) * k2)
k4 = f(t + h,   y_n + h     * k3)

y_{n+1} = y_n + (h/6) * (k1 + 2*k2 + 2*k3 + k4)
```

This matches the `k1`..`k4` computation in `include/runge_kutta/rk4.hpp` directly:

- **`k1`** is the slope at the start of the step — the plain derivative at `(t, y_n)`.
- **`k2`** is a slope estimate at the *midpoint* of the step, using the state you'd reach if you moved half a step using `k1`.
- **`k3`** is a second midpoint slope estimate, this time using the state reached by moving half a step using `k2` — refining the midpoint estimate.
- **`k4`** is the slope at the *end* of the step, using the state reached by moving a full step using `k3`.

The update `y_{n+1} = y_n + (h/6)(k1 + 2*k2 + 2*k3 + k4)` is a weighted average of these four slopes: the two midpoint estimates (`k2`, `k3`) are trusted twice as much as the endpoint estimates (`k1`, `k4`), because the midpoint is where a single evaluation captures the most information about how the slope is curving across the step. This is exactly Simpson's-rule-style weighting (1, 2, 2, 1)/6 applied to slope samples instead of function samples.

### Order of accuracy

Because it combines four evaluations this way, RK4 cancels error terms up through third order, leaving a *global* error of `O(h^4)` (local error per step is `O(h^5)`). Practically: halving the step size `h` should reduce the total integration error by roughly a factor of `2^4 = 16`. In practice, due to floating-point round-off, the observed ratio isn't exactly 16 — `test/test_convergence.cpp` checks this empirically by halving `h` on both an exponential-decay and a harmonic-oscillator problem with known analytic solutions and asserting the error ratio falls in `[12, 20]`, rather than asserting exactly 16.

### Choosing a step size

Smaller `h` means better accuracy (error shrinks as `h^4`) at the cost of more derivative evaluations (4 calls to `f` per step). Because the error already falls off so fast with `h`, RK4 typically doesn't need a very small step to get good accuracy — start with a step size a small fraction of the fastest timescale in your system (e.g. `h` well under `1/ω₀` for an oscillator with natural frequency `ω₀`), and use the convergence behavior above to sanity-check whether a given `h` is small enough for your accuracy needs.

## Running the demo

`rk4_demo_node` integrates a damped harmonic oscillator (`ẍ + 2ζω₀ẋ + ω₀²x = 0`, as a 2D first-order system) at a fixed rate and publishes the resulting state:

```bash
roslaunch runge_kutta rk4_demo.launch
```

This loads `config/rk4_demo.yaml` onto the node's private namespace and starts publishing on `~state` (i.e. `/rk4_demo_node/state`) as a `std_msgs/Float64MultiArray`, with dimension labels `position` and `velocity`. Watch it with:

```bash
rostopic echo /rk4_demo_node/state
```

Configurable private parameters (defaults shown, from `config/rk4_demo.yaml`):

| Parameter       | Default        | Meaning                          |
|-----------------|----------------|-----------------------------------|
| `step_size`     | `0.01`         | Integration step `h` (seconds)   |
| `publish_rate`  | `50.0`         | Publish rate (Hz)                |
| `initial_state` | `[1.0, 0.0]`   | Initial `[position, velocity]`   |
| `damping`       | `0.1`          | Damping ratio `ζ`                |
| `stiffness`     | `1.0`          | `ω₀²` in the oscillator equation |

## Testing

Build and run the gtest suite (`test_rk4`, covering single-step correctness, end-to-end `integrate` behavior including step-size validation, and the 4th-order convergence checks described above):

```bash
catkin build runge_kutta --catkin-make-args tests
rosrun runge_kutta test_rk4   # or: catkin_make run_tests_runge_kutta
```

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include <vector>

#include "runge_kutta/rk4.hpp"

using State = std::vector<double>;

int main(int argc, char** argv) {
    ros::init(argc, argv, "rk4_demo_node");
    ros::NodeHandle pnh("~");

    double step_size = 0.01;
    double publish_rate = 50.0;
    double damping = 0.1;
    double stiffness = 1.0;
    std::vector<double> initial_state = {1.0, 0.0};

    pnh.getParam("step_size", step_size);
    pnh.getParam("publish_rate", publish_rate);
    pnh.getParam("damping", damping);
    pnh.getParam("stiffness", stiffness);
    pnh.getParam("initial_state", initial_state);

    if (initial_state.size() != 2) {
        ROS_ERROR("initial_state must have exactly 2 elements (got %zu); using [1.0, 0.0]",
                  initial_state.size());
        initial_state = {1.0, 0.0};
    }
    if (!(step_size > 0.0)) {
        ROS_ERROR("step_size must be > 0 (got %f); using 0.01", step_size);
        step_size = 0.01;
    }
    if (!(publish_rate > 0.0)) {
        ROS_ERROR("publish_rate must be > 0 (got %f); using 50.0", publish_rate);
        publish_rate = 50.0;
    }

    ros::Publisher pub = pnh.advertise<std_msgs::Float64MultiArray>("state", 10);

    // y = [x, v],  dx/dt = v,  dv/dt = -stiffness*x - damping*v
    auto f = [damping, stiffness](double /*t*/, const State& y) {
        return State{y[1], -stiffness * y[0] - damping * y[1]};
    };

    State y = initial_state;
    double t = 0.0;

    ros::Timer timer = pnh.createTimer(
        ros::Duration(1.0 / publish_rate),
        [&](const ros::TimerEvent&) {
            y = runge_kutta::RK4<State>::step(f, t, y, step_size);
            t += step_size;

            std_msgs::Float64MultiArray msg;
            msg.layout.dim.resize(1);
            msg.layout.dim[0].label = "state";
            msg.layout.dim[0].size = static_cast<uint32_t>(y.size());
            msg.layout.dim[0].stride = static_cast<uint32_t>(y.size());
            msg.layout.data_offset = 0;
            msg.data = y;
            pub.publish(msg);
        });

    ROS_INFO("rk4_demo_node started: step=%.4fs, rate=%.1fHz, damping=%.3f, stiffness=%.3f",
             step_size, publish_rate, damping, stiffness);

    ros::spin();
    return 0;
}

//
// Created by ken on 25-2-16.
//

#ifndef SIMULATOR_H
#define SIMULATOR_H

//std
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>
#include <atomic>

//ros2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/string.hpp"

//project
#include "sim_interfaces/msg/debug_rune_angle.hpp"
#include "curve_fitter/curve_fitter.hpp"
#include "rm_interfaces/msg/debug_rune_angle.hpp"

class Simulator : public rclcpp::Node
{
public:
    Simulator(const std::string& name);

private:
    std::vector<std::pair<double, double>> observed_data_;
    double start_time_;
    double last_time_ = 0;
    double last_observed_angle_;

    rclcpp::Time last_timestamp;

    std::unique_ptr<CurveFitter> curve_fitter_;

    std::atomic<bool> sim_running_ = true;
    rclcpp::TimerBase::SharedPtr gimbal_action_timer_;

    rclcpp::Subscription<rm_interfaces::msg::DebugRuneAngle>::SharedPtr observed_angle_sub_;

    rclcpp::Publisher<sim_interfaces::msg::DebugRuneAngle>::SharedPtr predicted_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fitting_info_pub_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    rcl_interfaces::msg::SetParametersResult param_callback(
        const std::vector<rclcpp::Parameter> &params);

    void target_received_callback(rm_interfaces::msg::DebugRuneAngle::SharedPtr msg);
    void predict_callback();
};


#endif //SIMULATOR_H

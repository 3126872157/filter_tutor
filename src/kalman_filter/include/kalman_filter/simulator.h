//
// Created by ken on 25-2-16.
//

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class Simulator : public rclcpp::Node{
  public:
    Simulator(const std::string& name);

    double generateNoise(double mean, double stddev);

  private:
    double simulate_pos_x;
    std::mt19937 generator_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    rclcpp::TimerBase::SharedPtr simulator_timer_;
    rclcpp::TimerBase::SharedPtr kalman_timer_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr raw_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr filtered_pub_;

    void simulator_callback();
    void kalman_callback();
};



#endif //SIMULATOR_H

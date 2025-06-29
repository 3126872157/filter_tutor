//
// Created by ken on 25-2-16.
//

#include "curve_fitter/simulator.h"

//模拟器构造函数
Simulator::Simulator(const std::string &name) : Node(name) {
    //初始化匀加速型卡尔曼滤波器，delta_T设置为0.01即100Hz
    curve_fitter_ = std::make_unique<CurveFitter>(MotionType::UNKNOWN);
    curve_fitter_->setAutoTypeDetermined(false);
    curve_fitter_->setType(MotionType::BIG);

    this->declare_parameter("flying_time", 0.36);

    //动态参数设置
    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&Simulator::param_callback, this, std::placeholders::_1));

    //发布预测角度
    predicted_angle_pub_ = this->create_publisher<sim_interfaces::msg::DebugRuneAngle>(
        "curve_fitter/predicted_angle",
        10);

    //接收处理过的 observed_angle
    observed_angle_sub_ = this->create_subscription<rm_interfaces::msg::DebugRuneAngle>(
        "/rune_solver/observed_angle",
        rclcpp::SensorDataQoS(),
        [this](const rm_interfaces::msg::DebugRuneAngle::SharedPtr msg) {
            target_received_callback(msg);
        });

    //云台响应回调
    gimbal_action_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                             [this]() {
                                                 predict_callback();
                                             });

    //构造成功
    RCLCPP_INFO(this->get_logger(), "%s node initialized!", name.c_str());
}

void Simulator::target_received_callback(const rm_interfaces::msg::DebugRuneAngle::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Observed angle received!");

    double now_time = rclcpp::Time(msg->header.stamp).seconds();

    static bool inited = false;
    if (!inited) {
        start_time_ = now_time;
        inited = true;
    }

    double observed_time = now_time - start_time_;
    observed_data_.emplace_back(observed_time, msg->data);

    curve_fitter_->update(observed_time, msg->data);

    last_time_ = now_time;
    last_observed_angle_ = msg->data;
    last_timestamp = msg->header.stamp;
}

void Simulator::predict_callback() {
    if (last_time_ == 0)
        return;

    double flying_time = this->get_parameter("flying_time").as_double();
    double predict_time = last_time_ + flying_time;

    double t1 = predict_time - start_time_;
    double t0 = last_time_ - start_time_;

    auto predict_angle_diff = curve_fitter_->predict(t1) - curve_fitter_->predict(t0);

    rclcpp::Time predict_timestamp = last_timestamp + rclcpp::Duration::from_seconds(flying_time);
    sim_interfaces::msg::DebugRuneAngle msg;
    msg.header.stamp = predict_timestamp;
    msg.data = predict_angle_diff + last_observed_angle_;
    predicted_angle_pub_->publish(msg);
}

rcl_interfaces::msg::SetParametersResult Simulator::param_callback(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param: params) {
        if (param.get_name() == "flying_time") {
            RCLCPP_INFO(this->get_logger(), "flying time : %f", param.as_double());
        }
    }
    return result;
}

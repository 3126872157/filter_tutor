//
// Created by ken on 25-2-16.
//

#include "kalman_filter/simulator.h"

//模拟器构造函数
Simulator::Simulator(const std::string& name) : Node(name), generator_(std::random_device{}())
{
    //并行回调，设置为同一组别
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    //发布原始数据
    raw_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("raw", 1);
    //发送滤波后数据
    filtered_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("filtered", 1);
    //原始数据生成定时器
    simulator_timer_ = create_wall_timer(std::chrono::microseconds(10000),
                                        [this]() {simulator_callback();},
                                        callback_group_);

    //构造成功
    RCLCPP_INFO(this->get_logger(), "%s node initialized!", name.c_str());
}

//生成搞死噪音
double Simulator::generateNoise(double mean, double stddev)
{
    std::normal_distribution<> distribution(mean, stddev);
    return distribution(generator_);
}

//模拟器生成原始数据，模拟前端传入数据频率
void Simulator::simulator_callback()
{
    static double pos_x = 0;
    static double vec_x = 0;
    static double acc_x = 0.5;
    static double delta_t = 0.01;
    static uint16_t error_injeect_count = 0;

    //RCLCPP_INFO(this->get_logger(), "simulator_callback");

    //运动方程
    if(vec_x > 2)
    {
        acc_x = -0.5;
    }
    else if(vec_x < -2)
    {
        acc_x = 0.5;
    }
    vec_x += acc_x * delta_t;
    pos_x = pos_x + vec_x * delta_t + 0.5 * acc_x * delta_t * delta_t;

    simulate_pos_x = pos_x + generateNoise(0, 0.05);

    //故障注入
    if (error_injeect_count > 150)
    {
        error_injeect_count = 0;
        simulate_pos_x += 2;
    }
    error_injeect_count += 1;

    //发布原始数据
    geometry_msgs::msg::Vector3 raw_data;
    raw_data.x = simulate_pos_x;
    raw_data.y = vec_x;
    raw_data.z = acc_x;
    raw_pub_->publish(raw_data);
}

//生成滤波后数据，以云台的响应频率发出
void Simulator::kalman_callback()
{

}

#pragma once
#include "rclcpp/rclcpp.hpp"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
#include "std_msgs/msg/u_int16.hpp"

class PsCon_Node : public rclcpp::Node{
public:
    PsCon_Node();
    ~PsCon_Node();
    std::thread worker_;
    std::atomic<bool> running_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscription_;
    
private:
    void print_byte_binary(uint8_t value);

    void pscon_thread();
    void callback(const std_msgs::msg::UInt16::SharedPtr msg);

    int flag;
};
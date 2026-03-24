#include "pscon_node/pscon_node.hpp"
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <iostream>
#include "std_msgs/msg/u_int16.hpp"

PsCon_Node::PsCon_Node() : Node("pscon_node"){
  RCLCPP_INFO(this->get_logger(),"pscon node start");
  running_ = true;
  publisher_ =
        this->create_publisher<std_msgs::msg::UInt16>("pscon_data",10);
  subscription_ =
        this->create_subscription<std_msgs::msg::UInt16>("can_data",10,std::bind(&PsCon_Node::callback,this,std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(),"pscon set done");
  worker_ = std::thread(&PsCon_Node::pscon_thread, this);
}

PsCon_Node::~PsCon_Node(){
  RCLCPP_INFO(this->get_logger(),"pscon node end");
  running_ = false;
  if (worker_.joinable()){
    worker_.join();
  }
  RCLCPP_INFO(this->get_logger(),"Thread end");
}

void PsCon_Node::pscon_thread(){
  printf("thred start\r\n");
  std_msgs::msg::UInt16 pub_msg;
  while (1){
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
    if (flag == 1){
      pub_msg.data &= 0x0000;
    }
    publisher_->publish(pub_msg);
  }
}

void PsCon_Node::callback(const std_msgs::msg::UInt16::SharedPtr msg){
  uint16_t sub_data = msg->data;
  if ((sub_data&0x0001)==0x0001){
    flag = 1;
  }
}

void PsCon_Node::print_byte_binary(uint8_t value){
  //bit表示
  for(int i = 7; i >= 0; i--){
    printf("%d", (value >> i) & 0x01);
  }
  printf("\r\n");
}
#include "rclcpp/rclcpp.hpp"
#include <stdlib.h>
#include <math.h>
#include "can_node/can_node.hpp"

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Can_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
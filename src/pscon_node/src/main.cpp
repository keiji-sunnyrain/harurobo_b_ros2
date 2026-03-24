#include "rclcpp/rclcpp.hpp"
#include <stdlib.h>
#include <math.h>
#include "pscon_node/pscon_node.hpp"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PsCon_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
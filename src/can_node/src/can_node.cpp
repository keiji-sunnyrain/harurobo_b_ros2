#include "can_node/harurobo_can_id.h"
#include "can_node/can_node.hpp"

Can_Node::Can_Node() : Node("can_node"){
    RCLCPP_INFO(this->get_logger(),"Can node start");
}

Can_Node::~Can_Node(){
    RCLCPP_INFO(this->get_logger(),"Can node end");
}
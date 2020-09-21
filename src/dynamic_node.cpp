#include "ros-dynamic-reconfigure/dynamic.hpp"


const std::string RosNodeName = "dynamic_node";

int main(int argc, char ** argv)
{
    // ROS initialization
    ros::init(argc, argv, RosNodeName);

    // Instantiate dynamic_class
    dynamic_class node;

    // Start node
    node.start();

    return 0;
}
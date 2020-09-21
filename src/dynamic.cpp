#include "ros-dynamic-reconfigure/dynamic.hpp"


dynamic_class::dynamic_class()
{
    ;
}


dynamic_class::~dynamic_class()
{
    ;
}


void dynamic_class::start()
{
    // Dynamic reconfigure handle
    dynamic_reconfigure::Server<ros_dynamic_reconfigure::RosDynamicReconfigureConfig> server;

    // Set callback function for dynamic reconfigure (using lambda)
    server.setCallback(
        [this](ros_dynamic_reconfigure::RosDynamicReconfigureConfig & config, uint32_t level)
        {
            ROS_INFO_STREAM(
                "\nThe first drop-down: " << config.drop_down_menu <<
                "\nTrue of false: " << ((config.true_or_false) ? "true" : "false") <<
                "\nSlider value: " << config.slider_points << std::endl
            );
        });

    // ROS spin (wait for server to enter callback)
    ros::spin();
}
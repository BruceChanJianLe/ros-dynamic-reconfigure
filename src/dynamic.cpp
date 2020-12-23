#include "ros-dynamic-reconfigure/dynamic.hpp"


dynamic_class::dynamic_class()
:   private_nh_("~"),
    server_(std::make_shared<dynamic_reconfigure::Server<ros_dynamic_reconfigure::RosDynamicReconfigureConfig>> (private_nh_)),
    menu_state_(0),
    points_(0.0),
    true_false_state_(true),
    rate_(5)
{
    // Load params from server
    private_nh_.param("menu_state", menu_state_, 0);
    private_nh_.param("points", points_, 0.0);
    private_nh_.param("true_false_state", true_false_state_, true);
    private_nh_.param("rate", rate_, 5);
}


dynamic_class::~dynamic_class()
{
    ;
}


void dynamic_class::start()
{
    // Set callback function for dynamic reconfigure (using lambda)
    server_->setCallback(
        [this](ros_dynamic_reconfigure::RosDynamicReconfigureConfig & config, uint32_t level)
        {
            // Pass settings to local variables
            this->menu_state_ = config.drop_down_menu;
            this->true_false_state_ = config.true_or_false;
            this->points_ = config.slider_points;

            // Display current settings
            ROS_INFO_STREAM(
                "\nThe first drop-down: " << config.drop_down_menu <<
                "\nTrue of false: " << ((config.true_or_false) ? "true" : "false") <<
                "\nSlider value: " << config.slider_points << std::endl
            );
        });

    ros::Rate r(rate_);
    while(private_nh_.ok())
    {
        // Display vairables value
        ROS_INFO_STREAM(
            "\n menu_state: " << menu_state_ <<
            "\n true_false_state: " << true_false_state_ <<
            "\n points: " << points_
        );

        // ROS spin (wait for server to enter callback)
        ros::spinOnce();

        // Sleep node
        r.sleep();
    }
}
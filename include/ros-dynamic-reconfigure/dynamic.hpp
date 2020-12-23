#ifndef __D_H_
#define __D_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ros-dynamic-reconfigure/RosDynamicReconfigureConfig.h>

#include <string>
#include <memory>

class dynamic_class
{
    private:
        // ROS declaration
        ros::NodeHandle private_nh_;

        // Dynamic reconfigure handle
        std::shared_ptr<dynamic_reconfigure::Server<ros_dynamic_reconfigure::RosDynamicReconfigureConfig>> server_;

        // Private variables
        int menu_state_;
        double points_;
        bool true_false_state_;

        int rate_;

    public:
        // Constructor and destructor
        dynamic_class();
        ~dynamic_class();

        // Public function
        void start();
};

#endif
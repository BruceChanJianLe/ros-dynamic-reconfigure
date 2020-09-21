#ifndef __D_H_
#define __D_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ros-dynamic-reconfigure/RosDynamicReconfigureConfig.h>

#include <string>

class dynamic_class
{
    private:
        // ROS declaration
        ros::NodeHandle nh_;

    public:
        // Constructor and destructor
        dynamic_class();
        ~dynamic_class();

        // Public function
        void start();
};

#endif
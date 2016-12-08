#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <geometry_msgs/Twist.h>

#include "dpr4_base.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "dpr4_base");
    ros::NodeHandle n;

    DPR4Base *base = new DPR4Base();
    ros::Subscriber sub = n.subscribe("cmd_vel", 100, &DPR4Base::moveCallback, base);

    ros::spin();
    return 0;
}

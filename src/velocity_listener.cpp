#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <geometry_msgs/Twist.h>

#include "dpr4_base.h"
#include "odometry_publisher.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "dpr4_base");
    ros::NodeHandle n;
    double updateRate = 50;

    ROS_INFO("HEre 0");
    DPR4Base *base = new DPR4Base(&n);
    ROS_INFO("HEre 1");
    // OdometryPublisher *odomPublisher = new OdometryPublisher(base, n);
    ROS_INFO("HEre 2");
    // ros::Subscriber sub = n.subscribe("cmd_vel", 100, &DPR4Base::moveCallback, base);
    ROS_INFO("HEre 3");

    ros::Rate r(updateRate);
    while(n.ok()) {
        ros::spinOnce();
        // odomPublisher->publishOdometry();
        r.sleep();
    }
    return 0;
}

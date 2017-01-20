#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <geometry_msgs/Twist.h>

#include "dpr4_base.h"
#include "spine_listener.h"
#include "odometry_publisher.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dpr4_base");
    ros::NodeHandle n;
    double updateRate = 50;

    DPR4Base *base = new DPR4Base(&n);
    OdometryPublisher *odomPublisher = new OdometryPublisher(base, n);
    ros::Subscriber sub = n.subscribe("cmd_vel", 100, &DPR4Base::moveCallback, base);

    ros::Rate r(updateRate);
    while(n.ok()) {
        ros::spinOnce();
        odomPublisher->publishOdometry();
        base->spineListener->pollPosition();
        r.sleep();
    }

    ROS_INFO("Shutting down");
    base->spineListener->goToEndPosition();

    return 0;
}

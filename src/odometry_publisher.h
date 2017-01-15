#ifndef ODOMETRY_PUBLISHER_H
#define ODOMETRY_PUBLISHER_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <threemxl/C3mxlROS.h>
#include <threemxl/C3mxl.h>
#include <math.h>
#include "dpr4_base.h"
#include <ros/ros.h>

class OdometryPublisher {
private:
	DPR4Base *base;
    double overflowPoint, overflowThreshold;
	ros::Publisher odom_pub;
	tf::TransformBroadcaster odom_broadcaster;

	// Params that keep track of odom position
	double x, y, th, dy, dx, dTheta, leftPos, rightPos, leftDistance, rightDistance;
    ros::Time lastTime;
    bool logging;

    // Cause double abs is not a thing
    double myAbs(double);
    void updateOdometry();
    void updateOdometry2();
		double OdometryPublisher::calcDistanceTravelled(double, double)
public:
    OdometryPublisher(DPR4Base*, ros::NodeHandle);

    void publishOdometry();
};
#endif //ODOMETRY_PUBLISHER_H

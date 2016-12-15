#include "odometry_publisher.h"

/**
 * Constructor, initialises variables and node stuff;
 */
OdometryPublisher::OdometryPublisher(DPR4Base *baseP, Nodehandle nP) {
    base = baseP;
    n = nP;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    x = 0;
    y = 0;
    th = 0;
    leftPos = 0;
    rightPos = 0;
    leftDistance = 0;
    rightDistance = 0;

    overflowPoint = 32.768d;
    overflowThreshold = 50.0d;
    logging = true;

    lastTime = ros::Time::now();
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
}

/**
 * Publish the odometry/transform messages
 */
void OdometryPublisher::publishOdometry() {
    this->updateOdometry();

    // Calculate time interval
    double curTime = ros::Time::now();
    double dt = (curTime - lastTime).toSec();
    lastTime = curTime;

    // No rotation
    geometry_msgs::Quaternion cam_quat = tf::createQuaternionMsgFromYaw(0.0);

    // Transform for kinect, x is forwards, y left, z upwards.
    geometry_msgs::TransformStamped camera_trans;
    camera_trans.header.stamp = current_time;
    camera_trans.header.frame_id = "base_link";
    camera_trans.child_frame_id = "camera_depth_frame";
    camera_trans.transform.translation.x = 0.08;
    camera_trans.transform.translation.y = 0.0;
    camera_trans.transform.translation.z = 0.2;
    camera_trans.transform.rotation = cam_quat;

    odom_broadcaster.sendTransform(camera_trans);

    // Again, no rotation
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    // Transform for odom/base_link, x and y inverted because different axis
    // used in our calculations of odom.
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = y/100.0;
    odom_trans.transform.translation.y = -x/100.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // The actual odom message
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = y/100.0;
    odom.pose.pose.position.y = -x/100.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = (dy/100.0) / dt;
    odom.twist.twist.linear.y = (-dx/100.0) / dt;
    odom.twist.twist.angular.z = dTheta / dt;

    // publish the message
    odom_pub.publish(odom);
}

/**
 * Returns the absolute value of a double.
 * @param x - the input double
 * @return - the absolute value of the input
 */
double OdometryPublisher::myAbs(double x) {
    if(x < 0) {
        return -1*x;
    }
    return x;
}

/**
 * Update internal odom values;
 */
void OdometryPublisher::updateOdometry() {
    // Get new position of the motors
    double newLeftPos = base->getLeftPos();
    double newRightPos = base->getRightPos();
    double dLeftPos = newLeftPos - leftPos;
    double dRightPos = newRightPos - rightPos;

    // left overflow
    if (myAbs(leftPos - newLeftPos) > overflow_threshold) {
        if(leftPos > 0) {
            dLeftPos = dLeftPos + 2*overflow_point;
        } else {
            dLeftPos = dLeftPos - 2*overflow_point;
        }
    }

    // right overflow
    if (myAbs(rightPos - newRightPos) > overflow_threshold) {
        if(rightPos > 0) {
            dRightPos = dRightPos + 2*overflow_point;
        } else {
            dRightPos = dRightPos - 2*overflow_point;
        }
    }

    // Calculate distances
    leftPos = newLeftPos;
    rightPos = newRightPos;
    double dLeftDistance = dLeftPos*wheel_diameter/2.0d;
    double dRightDistance = dRightPos*wheel_diameter/2.0d;
    leftDistance += dLeftDistance;
    rightDistance += dRightDistance;

    // Calculate radius
    double radius;
    if(myAbs(dLeftDistance - dRightDistance) < 0.00001d) {
        radius = 0;
    } else {
        radius = wheel_base/2.0 * ((dLeftDistance + dRightDistance) / (dLeftDistance - dRightDistance));
    }
    if(radius < 0) radius = -1*radius;

    // Calculate theta from the radius
    double dTheta;
    if(dLeftDistance > 0 || dLeftDistance < 0) {
        if((dRightDistance < dLeftDistance && dLeftDistance > 0) || (dLeftDistance < 0 && dRightDistance > dLeftDistance)) {
            dTheta = dLeftDistance / (radius + (wheel_base/2.0));
            if (logging) std::cout << "BUITENBOCHT" << std::endl;
        } else {
            dTheta = dLeftDistance / (radius - (wheel_base/2.0));
            if (logging) std::cout << "BINNENBOCHT" << std::endl;
        }
    } else {
        dTheta = dRightDistance / (radius + (wheel_base/2.0));
    }

    // Driving backwards, invert the angle
    if (dLeftDistance < 0 || (dRightDistance < 0 && dLeftDistance >= 0)) {
        dTheta = -1*dTheta;
    }

    // No radius, so no dTheta
    if(radius == 0) {
        dTheta = 0;
    }

    // Calculate the difference in dx and dy
    double dx = radius*cos(dTheta) - radius;
    double dy = radius*sin(dTheta);

    // Corner to the right, invert angle and dx
    if(dRightDistance < dLeftDistance) {
        dTheta = -1*dTheta;
        dx = -1*dx;
    }

    // One of the sides driving backwards, invert both axis
    if (dLeftDistance < 0 || dRightDistance < 0) {
        dx = -1*dx;
        dy = -1*dy;
    }

    // Calculate odom coordinates
    x += dx*cos(th) - dy*sin(th);
    y += dx*sin(th) + dy*cos(th);
    th += dTheta;

    // Log all the things
    if(logging) {
        std::cout << "Leftpos: " << leftPos << std::endl;
        std::cout << "Rightpos: " << rightPos << std::endl;
        std::cout << "Left distance: " << leftDistance << std::endl;
        std::cout << "Right distance: " << rightDistance << std::endl;
        std::cout << "Radius: " << radius << std::endl;
        std::cout << "dTheta: " << dTheta << std::endl;
        std::cout << "dx: " << dx << std::endl;
        std::cout << "x " << x << std::endl;
        std::cout << "y " << y << std::endl;
        std::cout << "th " << th << std::endl;
        std::cout << "th " << 360 * (th / (2 * 3.1415)) << std::endl;
        std::cout << "\n";
    }
}
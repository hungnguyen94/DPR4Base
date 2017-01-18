#ifndef SPINE_LISTENER_SPINELISTENER_H
#define SPINE_LISTENER_SPINELISTENER_H

#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <threemxl/C3mxl.h>
#include <geometry_msgs/Twist.h>

class SpineListener {
private:
    C3mxl *spineMotor;
    double height;
public:
    SpineListener(ros::NodeHandle*, LxSerial*);
    ~SpineListener() {
    }
};
#endif //SPINE_LISTENER_SPINELISTENER_H
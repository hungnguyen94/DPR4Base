#ifndef SPINE_LISTENER_SPINELISTENER_H
#define SPINE_LISTENER_SPINELISTENER_H

#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <threemxl/C3mxl.h>
#include <geometry_msgs/Twist.h>
#include <face_detection/FaceDetectionMsg.h>

class SpineListener {
private:
    double height;
    void static faceDetectionCallback(const face_detection::FaceDetectionMsg::ConstPtr&);
    ros::Subscriber faceDetectionSub;

public:
    SpineListener(ros::NodeHandle*, LxSerial*);
    ~SpineListener() {
    }
    C3mxl *spineMotor;
    double getHeight();
    void goToHeight(double);
};
#endif //SPINE_LISTENER_SPINELISTENER_H
#ifndef SPINE_LISTENER_SPINELISTENER_H
#define SPINE_LISTENER_SPINELISTENER_H

#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <threemxl/C3mxl.h>
#include <geometry_msgs/Twist.h>
#include <face_detection/FaceDetectionMsg.h>
#include <std_msgs/String.h>

class SpineListener {
private:
    double height;
    double targetHeight;
    void static faceDetectionCallback(const face_detection::FaceDetectionMsg::ConstPtr&);
    void static rbcStateCallback(const std_msgs::String::ConstPtr&);
    ros::Subscriber faceDetectionSub;
    ros::Subscriber rbcStateSub;

public:
    SpineListener(ros::NodeHandle*, LxSerial*);
    ~SpineListener() {
    }
    C3mxl *spineMotor;
    double getHeight();
    void goToHeight(double);
    void pollPosition();
    void goToEndPosition();
    void goToEndPositionBlocking();
    void goToMiddlePosition();
};
#endif //SPINE_LISTENER_SPINELISTENER_H
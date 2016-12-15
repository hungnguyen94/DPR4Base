#ifndef DPR4_BASE_DPR4_BASE_H
#define DPR4_BASE_DPR4_BASE_H

#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <threemxl/C3mxl.h>
#include <geometry_msgs/Twist.h>

class DPR4Base {
private:
    C3mxl *leftMotor, *rightMotor;
    double wheelDiameter, wheelBase;
public:
    DPR4Base();
    ~DPR4Base() {
        delete leftMotor;
        delete rightMotor;
    }

    void move(double linearX, double angularZ);
    void moveCallback(const geometry_msgs::Twist::ConstPtr &msg);

    double getLeftPos();
    double getRightPos();
};
#endif //DPR4_BASE_DPR4_BASE_H


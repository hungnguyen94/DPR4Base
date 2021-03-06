#include "dpr4_base.h"

#define DXLC_SAFE_CALL(call) \
    do { \
        int ret = call; \
        if (ret != DXL_SUCCESS) { \
        std::cout << "Error:" << std::endl << "  " << C3mxl::translateErrorCode(ret) << " (0x" << std::hex << ret << std::dec << ")" << std::endl; \
        } \
    } while (0)

/**
 * Constructor
 */
DPR4Base::DPR4Base()
{
    // Todo: Set using arguments?
    std::string usbAddress = "/dev/ttyUSB0";

    //Settings garbage robot:
    // wheelBase = 0.537d;
    // wheelDiameter = 0.292d;

    // Settings host robot:
    wheelDiameter = 29.3616564746d;
    wheelBase = 54.3d;

    LxSerial *serialPort = new LxSerial();
    CDxlConfig *leftMotorConfig = new CDxlConfig();
    CDxlConfig *rightMotorConfig = new CDxlConfig();

    serialPort->port_open(usbAddress, LxSerial::RS485_FTDI);
    serialPort->set_speed(LxSerial::S921600);

    leftMotor = new C3mxl();
    rightMotor = new C3mxl();

    leftMotor->setSerialPort(serialPort);
    rightMotor->setSerialPort(serialPort);

    leftMotor->setConfig(leftMotorConfig->setID(106));
    rightMotor->setConfig(rightMotorConfig->setID(107));

    ROS_DEBUG("Init motors");
    while(leftMotor->init(false) != DXL_SUCCESS)
    {
        ROS_ERROR("Failed to init left motor. Retrying.");
        sleep(100);
    }

    while(rightMotor->init(false) != DXL_SUCCESS)
    {
        ROS_ERROR("Failed to init right motor. Retrying.");
        sleep(100);
    }

    ROS_DEBUG("Set 3mxl mode to \"SPEED\"");
    DXLC_SAFE_CALL(leftMotor->set3MxlMode(SPEED_MODE));
    DXLC_SAFE_CALL(rightMotor->set3MxlMode(SPEED_MODE));

    ROS_DEBUG("Set acceleration");
    DXLC_SAFE_CALL(leftMotor->setAcceleration(3));
    DXLC_SAFE_CALL(rightMotor->setAcceleration(3));
}

/**
 * Move the base.
 * @param linearX Linear X movement
 * @param angularZ Angular Z movement
 */
void DPR4Base::move(double linearX, double angularZ)
{
    double linearVelocity  = linearX / (wheelDiameter / 2.0);
    double angularVelocity = angularZ * (wheelBase / wheelDiameter);

    double leftVelocity = linearVelocity - angularVelocity;
    double rightVelocity = linearVelocity + angularVelocity;

    // Todo: limit to max speed?
    double leftSpeed = leftVelocity;
    double rightSpeed = rightVelocity;

    ROS_INFO("DPR4 base -> set speed, leftVelocity: %f, rightVelocity: %f", leftSpeed, rightSpeed);

    DXLC_SAFE_CALL(leftMotor->setSpeed(leftSpeed));
    DXLC_SAFE_CALL(rightMotor->setSpeed(rightSpeed));
}


/**
 * Callback function when a msg is published.
 * @param msg Twist message
 */
void DPR4Base::moveCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    double linearX = msg->linear.x;
    double angularZ = msg->angular.z;
    ROS_INFO("DPR4 base -> received linear x: %f, angular.z: %f", linearX, angularZ);
    move(linearX, angularZ);
}

/**
 * Get the position of the left motor.
 * @return - position of the left motor in radians.
 */
double DPR4Base::getLeftPos() {
   leftMotor->getPos();
   return leftMotor->presentPos();
}

/**
 * Get the position of the right motor.
 * @return - position of the right motor in radians.
 */
double DPR4Base::getRightPos() {
   rightMotor->getPos();
   return rightMotor->presentPos();
}

/**
 * Get the wheel diameter of the base.
 * @return  - the diameter of the wheels
 */
double DPR4Base::getWheelDiameter() {
    return wheelDiameter;
}

/**
 * Get the wheel base length.
 * @return  - the length of the wheel base
 */
double DPR4Base::getWheelBase() {
    return wheelBase;
}

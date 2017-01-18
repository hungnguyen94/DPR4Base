#include "spine_listener.h"

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
SpineListener::SpineListener(ros::NodeHandle *nh, LxSerial *serialPort)
{
    CDxlConfig *spineMotorConfig = new CDxlConfig();

    spineMotor = new C3mxl();
    spineMotor->setSerialPort(serialPort);
    spineMotor->setConfig(spineMotorConfig->setID(110));

    ROS_INFO("Init spine motor");
    while(spineMotor->init(false) != DXL_SUCCESS)
    {
        ROS_ERROR("Failed to init spine motor. Retrying.");
        sleep(100);
    }

    ROS_INFO("External init spine motor");
    DXLC_SAFE_CALL(spineMotor->set3MxlMode(EXTERNAL_INIT));
    DXLC_SAFE_CALL(spineMotor->setSpeed(-10));
    DXLC_SAFE_CALL(spineMotor->setAcceleration(5));
    DXLC_SAFE_CALL(spineMotor->setTorque(-2));
    while(spineMotor->getStatus() != M3XL_STATUS_INIT_DONE) {
        ROS_INFO("Waiting for external init");
        sleep(100);
    }
    ROS_INFO("Spine motor has been initialized");
    height = 0;

    DXLC_SAFE_CALL(spineMotor->set3MxlMode(POS_MODE));
    DXLC_SAFE_CALL(spineMotor->setLinearAcceleration(0.1));
    DXLC_SAFE_CALL(spineMotor->setLinearSpeed(0.1));
    ROS_INFO("Values have been set for spine motor");
}
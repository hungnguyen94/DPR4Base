#include "spine_listener.h"

#define DXLC_SAFE_CALL(call) \
    do { \
        int ret = call; \
        if (ret != DXL_SUCCESS) { \
        std::cout << "Error:" << std::endl << "  " << C3mxl::translateErrorCode(ret) << " (0x" << std::hex << ret << std::dec << ")" << std::endl; \
        } \
    } while (0)

// Pointer to self to use in (non-member) callbacks
SpineListener *spineListener;

/**
 * Constructor
 */
SpineListener::SpineListener(ros::NodeHandle *nh, LxSerial *serialPort) {
    spineListener = this;
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
    // while(spineMotor->getStatus() != M3XL_STATUS_INIT_DONE) {
        ROS_INFO("Waiting for external init");
        // std::cout << spineMotor->getStatus() << std::endl;
        // std::cout << C3mxl::translateErrorCode(spineMotor->getStatus()) << std::endl;
        sleep(15);
        // std::cout << C3mxl::translateErrorCode(spineMotor->getStatus()) << std::endl;
        
    // }
    ROS_INFO("Spine motor has been initialized");
    height = 0;

    DXLC_SAFE_CALL(spineMotor->set3MxlMode(POSITION_MODE));
    DXLC_SAFE_CALL(spineMotor->setLinearAcceleration(0.1));
    DXLC_SAFE_CALL(spineMotor->setLinearSpeed(0.1));
    ROS_INFO("Values have been set for spine motor");


    faceDetectionSub = nh->subscribe("face_detection", 1000, &faceDetectionCallback);
    ROS_INFO("SUBSCRIBED");
}

double SpineListener::getHeight() {
    DXLC_SAFE_CALL(spineMotor->getLinearPos());
    height = spineMotor->presentLinearPos();
    return height;
}

void SpineListener::goToHeight(double targetHeight) {

}

void SpineListener::faceDetectionCallback(const face_detection::FaceDetectionMsg::ConstPtr& msg) {
    ROS_INFO("IN CALLBACK");
    bool looking = msg->looking;
    float height = msg->height;
    double currentHeight = spineListener->getHeight();
    std::cout << looking << ", " << height << ", " << currentHeight;
    ROS_INFO("%d, %f, %f\n", looking, height, currentHeight);


    if (looking) {
        if(currentHeight + height < 0) {
            ROS_INFO("GOING TO 0");
            // Go to lowest position
            DXLC_SAFE_CALL(spineListener->spineMotor->setLinearPos(0));
        } else if(currentHeight + height > 0.43) {
            ROS_INFO("GOING TO 0.43");
            // Go to heighest position
            DXLC_SAFE_CALL(spineListener->spineMotor->setLinearPos(0.43));
        } else {
            ROS_INFO("GOING TO %f\n", currentHeight+height);
            // Go to position
            DXLC_SAFE_CALL(spineListener->spineMotor->setLinearPos(currentHeight + height));
        }
    }
}


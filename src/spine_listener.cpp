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
        usleep(100*1000);
    }

    ROS_INFO("External init spine motor");
    DXLC_SAFE_CALL(spineMotor->set3MxlMode(EXTERNAL_INIT));
    DXLC_SAFE_CALL(spineMotor->getStatus());
    if(spineMotor->presentStatus() != M3XL_STATUS_INIT_DONE) {
        DXLC_SAFE_CALL(spineMotor->setSpeed(-10));
        DXLC_SAFE_CALL(spineMotor->setAcceleration(5));
        DXLC_SAFE_CALL(spineMotor->setTorque(-2));
    }
    while(spineMotor->presentStatus() != M3XL_STATUS_INIT_DONE && ros::ok()) {
        ROS_INFO("Waiting for external init");
        std::cout << spineMotor->presentStatus() << std::endl;
        usleep(1000*1000);
        DXLC_SAFE_CALL(spineMotor->getStatus());
    }
    ROS_INFO("Spine motor has been initialized");
    height = 0;
    targetHeight = 0;

    DXLC_SAFE_CALL(spineListener->spineMotor->set3MxlMode(POSITION_MODE));
    DXLC_SAFE_CALL(spineListener->spineMotor->setLinearAcceleration(0.1));
    DXLC_SAFE_CALL(spineListener->spineMotor->setLinearSpeed(0.1));

    faceDetectionSub = nh->subscribe("face_detection", 1000, &faceDetectionCallback);
    rbcStateSub = nh->subscribe("RBC/state", 1000, &rbcStateCallback);
    ROS_INFO("Subscribed to /face_detection");
}

double SpineListener::getHeight() {
    DXLC_SAFE_CALL(spineMotor->getLinearPos());
    height = spineMotor->presentLinearPos();
    return height;
}

void SpineListener::goToHeight(double newTargetHeight) {
    if(std::fabs(targetHeight - newTargetHeight) > 0.1) {
        spineMotor->get3MxlMode();
        if(spineMotor->present3MxlMode() != POSITION_MODE)
            spineMotor->set3MxlMode(POSITION_MODE);

        targetHeight = newTargetHeight;
        ROS_INFO("GOING TO %f\n", targetHeight);
        DXLC_SAFE_CALL(spineMotor->setLinearPos(newTargetHeight));
    }
}

void SpineListener::faceDetectionCallback(const face_detection::FaceDetectionMsg::ConstPtr& msg) {
    bool looking = msg->looking;
    float height = msg->height;
    height = height - 0.15;
    double currentHeight = spineListener->getHeight();

    spineListener->spineMotor->getStatus();
    spineListener->spineMotor->get3MxlMode();

    if (looking) {
        if(currentHeight + height < 0) {
            // Go to lowest position
            spineListener->goToHeight(0);
        } else if(currentHeight + height > 0.43) {
            // Go to heighest position
            spineListener->goToHeight(0.43);
        } else {
            // Go to position
            spineListener->goToHeight(currentHeight + height);
        }
    }
}

void SpineListener::rbcStateCallback(const std_msgs::String::ConstPtr& msg) {
    std::string data = msg->data.c_str();
    if(data.compare("DRIVING") == 0) {
        spineListener->goToEndPosition();
    } else if(data.compare("IDLE") == 0 || data.compare("INTERACTING") == 0) {
        spineListener->goToMiddlePosition();
    }
}

void SpineListener::pollPosition() {
    spineListener->spineMotor->getStatus();
    spineListener->spineMotor->get3MxlMode();
}

void SpineListener::goToEndPosition() {
    targetHeight = 0.02;
    ROS_INFO("Going to end position");
    DXLC_SAFE_CALL(spineMotor->setLinearPos(targetHeight));
}

void SpineListener::goToEndPositionBlocking() {
    targetHeight = 0.02;
    ROS_INFO("Going to end position");
    DXLC_SAFE_CALL(spineMotor->setLinearPos(targetHeight));
    do {
        usleep(100*1000);
        DXLC_SAFE_CALL(spineMotor->getStatus());
    } while (spineMotor->presentStatus() == M3XL_STATUS_MOVING);
}

void SpineListener::goToMiddlePosition() {
    targetHeight = 0.215;
    ROS_INFO("GOING TO middle");
    DXLC_SAFE_CALL(spineMotor->setLinearPos(targetHeight));
}

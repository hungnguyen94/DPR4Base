#include "odometry_publisher.h"

/**
 * Constructor, initialises variables and node stuff;
 */
OdometryPublisher::OdometryPublisher(DPR4Base *baseP, ros::NodeHandle n) {
    base = baseP;

    x = 0;
    y = 0;
    th = 0;
    leftPos = base->getLeftPos();
    rightPos = base->getRightPos();
    leftDistance = 0;
    rightDistance = 0;

    overflowPoint = 32.768;
    overflowThreshold = 50.0;
    logging;
    n.param<bool>("logging", logging, false);

    lastTime = ros::Time::now();
    odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
}

/**
 * Publish the odometry/transform messages
 */
void OdometryPublisher::publishOdometry() {
    this->updateOdometry2();

    // Calculate time interval
    ros::Time curTime = ros::Time::now();
    double dt = (curTime - lastTime).toSec();
    lastTime = curTime;

    // The rotation of the odom frame
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    // Transform for odom/base_link, x and y inverted because different axis
    // used in our calculations of odom.
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = curTime;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // The actual odom message
    nav_msgs::Odometry odom;
    odom.header.stamp = curTime;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = dx / dt;
    odom.twist.twist.linear.y = dy / dt;
    odom.twist.twist.angular.z = dTheta / dt;

    // publish the message
    odom_pub.publish(odom);
}

void OdometryPublisher::updateOdometry2() {
  // Get new position of the motors
  double newLeftPos = base->getLeftPos();
  double newRightPos = base->getRightPos();

  double dLeftDistance = this->calcDistanceTravelled(leftPos,newLeftPos);
  double dRightDistance = this->calcDistanceTravelled(rightPos,newRightPos);
  leftPos = newLeftPos;
  rightPos = newRightPos;
leftDistance += dLeftDistance;
rightDistance += dRightDistance;

  double turningRadius = 0 ;
  //Check if base is roting in place or driving straight to prevent dividing by zero
  if(myAbs(dLeftDistance - dRightDistance) > 0.00001d) {
      //The (optimalized) magic formula to calulate the turning radius
      double turningRadius = (dRightDistance/2*dLeftDistance + 0.5d) * base->getWheelBase() / (dRightDistance/dLeftDistance - 1);
  }

  //The (optimalized) magic formula to calulate the angle the base travelled
  double dTheta = dRightDistance / (turningRadius + base->getWheelBase()/2);
  //Multiply the turning angle with -1 when the robot is driving backwards
  if(dRightDistance < 0 && dLeftDistance<0) {dTheta *= -1;}

  double dx = turningRadius * sin(dTheta);
  double dy = turningRadius - turningRadius * cos(dTheta);
  x += dx;
  y += dy;
  th += dTheta;
}

/**
* Calculates the distance travelled by an wheel based on the current en previous wheel position.
* TODO check if this method works as intended, copied from previous version.
*
*/
double OdometryPublisher::calcDistanceTravelled(double prevPos, double curPos) {
  double dpos = curPos - prevPos;
  if (myAbs(dpos) > overflowThreshold) {
      if(curPos > 0) {
          dpos = dpos + 2*overflowPoint;
      } else {
          dpos = dpos - 2*overflowPoint;
      }
  }
  return dpos * base->getWheelDiameter() / 2.0d;
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

    bool turnInPlace = false;

    std::cout << "dRightPos: " << dRightPos << std::endl;

    // left overflow
    if (myAbs(leftPos - newLeftPos) > overflowThreshold) {
        if(leftPos > 0) {
            dLeftPos = dLeftPos + 2*overflowPoint;
        } else {
            dLeftPos = dLeftPos - 2*overflowPoint;
        }
    }

    // right overflow
    if (myAbs(rightPos - newRightPos) > overflowThreshold) {
        if(rightPos > 0) {
            dRightPos = dRightPos + 2*overflowPoint;
        } else {
            dRightPos = dRightPos - 2*overflowPoint;
        }
    }

    std::cout << "dRightPos: " << dRightPos << std::endl;
    std::cout << "getWheelDiameter: " << base->getWheelDiameter();

    // Calculate distances
    leftPos = newLeftPos;
    rightPos = newRightPos;
    double dLeftDistance = dLeftPos * base->getWheelDiameter() / 2.0d;
    double dRightDistance = dRightPos * base->getWheelDiameter() / 2.0d;
    std::cout << "dRightDistance: " << dRightDistance << std::endl;
    leftDistance += dLeftDistance;
    rightDistance += dRightDistance;

    // Calculate radius
    double radius;
    if(myAbs(dLeftDistance - dRightDistance) < 0.00001d) {
        radius = 0;
    } else {
        radius = myAbs(base->getWheelBase() / 2.0 * ((dLeftDistance + dRightDistance) / (dLeftDistance - dRightDistance)));
        if (radius == 0){
        	turnInPlace = true;
        }
    }

    // Calculate theta from the radius
    if(dLeftDistance > 0 || dLeftDistance < 0) {
        if((dRightDistance < dLeftDistance && dLeftDistance > 0) || (dLeftDistance < 0 && dRightDistance > dLeftDistance)) {
            dTheta = dLeftDistance / (radius + (base->getWheelBase() /2.0));
            if (logging) std::cout << "BUITENBOCHT" << std::endl;
        } else {
            dTheta = dLeftDistance / (radius - (base->getWheelBase() /2.0));
            if (logging) std::cout << "BINNENBOCHT" << std::endl;
        }
    } else {
        dTheta = dRightDistance / (radius + (base->getWheelBase() /2.0));
    }

    if (dLeftDistance < 0 || dRightDistance < 0) {
        dTheta = -1*dTheta;
    }
    if(radius == 0 && !turnInPlace) {
       dTheta = 0;
    }

	//if (dLeftDistance > 0 && dRightDistance < 0){
    if(dLeftDistance > 0 && dRightDistance < 0 && !turnInPlace) {
       dTheta = -1*dTheta;
    }

    // Calculate the difference in dx and dy
    dx = radius*cos(dTheta) - radius;
    dy = radius*sin(dTheta);

    // Corner to the right, invert angle and dx
    //if(dRightDistance < dLeftDistance) {
    if(dRightDistance < dLeftDistance && !turnInPlace) {
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
        std::cout << "Leftpos: " << leftPos << "\n";
        std::cout << "Rightpos: " << rightPos <<  "\n";
        std::cout << "Left distance: " << leftDistance <<  "\n";
        std::cout << "Right distance: " << rightDistance <<  "\n";
        std::cout << "Radius: " << radius <<  "\n";
        std::cout << "dTheta: " << dTheta <<  "\n";
        std::cout << "dx: " << dx <<  "\n";
        std::cout << "x " << x <<  "\n";
        std::cout << "y " << y <<  "\n";
        std::cout << "th " << th <<  "\n";
        std::cout << "th " << 360 * (th / (2 * 3.1415)) << "\n" << std::endl;
    }
}

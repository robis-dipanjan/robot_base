/**
 * @file DifferentialDrive.cpp
 * @author Dipanjan Maji (dipanjan.maji@motherson.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "DifferentialDrive.h"

DifferentialDrive::DifferentialDrive() {

}

DifferentialDrive::DifferentialDrive(std::shared_ptr<KincoMotorClient> *m, float base, float radius, float ratio) {
    motors = *m;
    setParams(base, radius, ratio);
}

DifferentialDrive::~DifferentialDrive() {

}

DriveFeedback DifferentialDrive::getFeedback() {
    std::vector<int32_t> ticks = motors->getEncodersData();
    std::vector<float> rpm = motors->getRPMData();
    dFeedback.lVel = rpm[0] * RPM_TO_RADS/GEAR_RATIO;
    dFeedback.rVel = rpm[1] * RPM_TO_RADS/GEAR_RATIO;
    dFeedback.lTicks = ticks[0];
    dFeedback.rTicks = ticks[1];
    return dFeedback;
}


void DifferentialDrive::setParams(float base, float radius, float ratio) {
    wheelBase = base;
    wheelRadius = radius;
    GEAR_RATIO = ratio;
    MS_TO_RPM = GEAR_RATIO * 60 / (2 * M_PI * wheelRadius); //
    ODOM_FACTOR = (2 * M_PI * wheelRadius) / (GEAR_RATIO * motors->getEncoderResolution());
}

void DifferentialDrive::setVelocity(float v, float w) {
    float rVel = v + w * wheelBase / 2;     // unit: m/s
	float lVel = v - w * wheelBase / 2;

    rVel *= MS_TO_RPM;
    lVel *= MS_TO_RPM;

    motors->setVelocitiesRPM({lVel, rVel});
}

/**
 * @brief reset pose to (0,0,0)
 * 
 */
void DifferentialDrive::resetOdom() {
    setOdom({0,0,0});
}

/**
 * @brief Set the Odom object
 * 
 * @param newPose 
 */
void DifferentialDrive::setOdom(Pose2D newPose) {
    pose = newPose;
    poseStamped.pose = newPose;
    poseStamped.time = std::chrono::system_clock::now();
}

/**
 * @brief Get the Odom object
 * 
 * @return Pose2D 
 */
Pose2D DifferentialDrive::getOdom() {
    return pose;
}

/**
 * @brief Get the Stamped Odom object
 * 
 * @return PoseStamped2D 
 */
PoseStamped2D DifferentialDrive::getStampedOdom() {
    return poseStamped;
}

/**
 * @brief 
 * 
 * @param leftTicks 
 * @param rightTicks 
 * @return Pose2D 
 */
Pose2D DifferentialDrive::
updateOdom(int32_t leftTicks, int32_t rightTicks) {
	float d_left = (leftTicks - prevFeedback.lTicks) * ODOM_FACTOR;
	float d_right = (rightTicks - prevFeedback.rTicks) * ODOM_FACTOR;
    prevFeedback = dFeedback;

	// distance traveled is the average of the two wheels
	float d = (d_right + d_left) / 2;
	// this approximation works (in radians) for small angles
	float th = (d_right - d_left) / wheelBase;
	if (d != 0) {
		// calculate distance traveled in x and y
		float x = cos(th) * d;
		float y = -sin(th) * d;
		// calculate the final position of the robot
		pose.x += (cos(pose.theta) * x - sin(pose.theta) * y);
		pose.y += (sin(pose.theta) * x + cos(pose.theta) * y);
	}
	pose.theta += th;
    pose.theta = std::fmod(pose.theta + M_PI, 2 * M_PI) - M_PI;	
    poseStamped.pose = pose;
    poseStamped.time = std::chrono::system_clock::now();
    return pose;
}

/**
 * @brief 
 * 
 * @return Pose2D 
 */
Pose2D DifferentialDrive::calculateOdom() {
    getFeedback();
    return updateOdom(dFeedback.lTicks, dFeedback.rTicks);
}
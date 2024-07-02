/**
 * @file DifferentialDrive.h
 * @author Dipanjan Maji (dipanjan.maji@motherson.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <math.h>
#include <memory>
#include "KincoMotor.h"
#include "DataStructures.h"


typedef struct {
    float lVel=0.0;
    float rVel=0.0;
    int32_t lTicks;
    int32_t rTicks;
} DriveFeedback;

class DifferentialDrive {
public:
    DifferentialDrive(std::shared_ptr<KincoMotorClient> *m, float base, float radius, float ratio);
    DifferentialDrive();
    ~DifferentialDrive();

    void setVelocity(float v, float w);
    void setParams(float base, float radius, float ratio);
    DriveFeedback getFeedback();
    void resetOdom();
    void setOdom(Pose2D newPose);
    Pose2D getOdom();
    PoseStamped2D getStampedOdom();
    Pose2D updateOdom(int32_t leftTicks, int32_t rightTicks);
    Pose2D calculateOdom();

private:
    Pose2D pose;
    PoseStamped2D poseStamped;
    float wheelBase;
    float wheelRadius;
    float MS_TO_RPM;
    float GEAR_RATIO;
    float RPM_TO_RADS = (2 * M_PI) / 60.0;
    float ODOM_FACTOR;
    DriveFeedback dFeedback;
    DriveFeedback prevFeedback;
    std::shared_ptr<KincoMotorClient> motors;
};
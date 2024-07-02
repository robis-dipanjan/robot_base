#include <iostream>
#include <ros/ros.h>
#include "DeviceManager.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>


DeviceManager dm;

void setKincoVelocity(const geometry_msgs::Twist::ConstPtr &msg) {
    float linearVelocity{0.0};
    float angularVelocity{0.0};

    linearVelocity = msg->linear.x;
    angularVelocity = msg->angular.z;

    dm.motorDriver->setDeviceStates({OPERATIONAL, OPERATIONAL});
    dm.motorDriver->setModes({SPEED_CTRL, SPEED_CTRL});
    dm.diffDrive->setVelocity(linearVelocity, angularVelocity);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "KincoMotorClient");
    ROS_INFO("Kinco Motor Client executing");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 100, setKincoVelocity);
    ros::Rate loop_rate(10);

    while(ros::ok()){
        nav_msgs::Odometry odomMsg;
        Pose2D poseAMR = dm.diffDrive->calculateOdom();
        odomMsg.pose.pose.position.x = poseAMR.x;
        odomMsg.pose.pose.position.y = poseAMR.y;
        odomMsg.pose.pose.orientation.w = poseAMR.theta;
        pub.publish(odomMsg);
        //pub.publish(dm.diffDrive->calculateOdom());
        ros::spinOnce();
        loop_rate.sleep();
    }

    dm.motorDriver->setModes({MOTOR_OFF, MOTOR_OFF});   
    dm.motorDriver->stopAll();
    ROS_INFO("Kinco Motor Client Stopped");

    ros::spin();
    return 0;
}
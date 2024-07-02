#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <cmath>
#include <chrono>
#include "RollingMeanAccumulator.h"
#include "DataStructures.h"
class Odometry
{
public:
  explicit Odometry(size_t velocityRollingWindowSize = 10);
  void init(const std::chrono::system_clock::time_point& time);//time
  bool update(double leftPos, double rightPos, const std::chrono::system_clock::time_point& time);
  bool update(int32_t leftPos, int32_t rightPos);
  bool updateFromVelocity(double leftVel, double rightVel, const std::chrono::system_clock::time_point& time);
  void updateOpenLoop(double linear, double angular, const std::chrono::system_clock::time_point& time);
  void resetOdometry();
  void setOdometry(double x,double y,double theta);
  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return std::fmod(heading_ + M_PI, 2 * M_PI) - M_PI; }
  double getLinear() const { return linear_; }
  double getAngular() const { return angular_; }
  Pose2D getOdometry();
  void setWheelParams(double wheelSeparation, double leftWheelRadius, double rightWheelRadius);
  void setVelocityRollingWindowSize(size_t velocityRollingWindowSize);
private:
  //RollingMeanAccumulator =RollingMeanAccumulator<double>;

  void integrateRungeKutta2(double linear, double angular);
  void integrateExact(double linear, double angular);
  void resetAccumulators();

  // Current timestamp:
  std::chrono::system_clock::time_point timeStamp_;

  // Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  // Current velocity:
  double linear_;   //   [m/s]
  double angular_;  // [rad/s]

  // Wheel kinematic parameters [m]:
  double wheel_separation_;
  double left_wheel_radius_;
  double right_wheel_radius_;

  // Previous wheel position/state [rad]:
  int32_t left_wheel_old_pos_;
  int32_t right_wheel_old_pos_;

  // Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator<double> linear_accumulator_;
  RollingMeanAccumulator<double> angular_accumulator_;
};
#endif  // ODOMETRY_H_
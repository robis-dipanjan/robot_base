#ifndef PID_H
#define PID_H
#include <cmath>
/// Very basic, mostly educational PID controller with derivative filter.
class PID {
  public:
    /// @param  kp  Proportional gain   @f$ K_p @f$
    /// @param  ki  Integral gain       @f$ K_i @f$
    /// @param  kd  Derivative gain     @f$ K_d @f$
    /// @param  fc  Cutoff frequency    @f$ f_c @f$ of derivative filter in Hz
    /// @param  Ts  Controller sampling time    @f$ T_s @f$ in seconds
    /// The derivative filter can be disabled by setting `fc` to zero.
   
    void setupPID(double kp, double ki, double kd, double fc, double Ts,double max_out);
    /// Compute the weight factor α for an exponential moving average filter
    /// with a given normalized cutoff frequency `fn`.
    static double calcAlphaEMA(double fn);

    double update(double error);

  private:
    double kp, ki, kd, alpha, Ts;
    double max_output =5.0;
    double integral = 0;
    double old_ef = 0;
};


#endif
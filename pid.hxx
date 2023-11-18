
#ifndef PID_HXX
#define PID_HXX

#include <iostream>

class PID
{
public:
    PID() {}
    ~PID() {}

    void init(double kp, double ki, double kd, double tau, double T, double minOutput, double maxOutput)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        tau_ = tau;
        T_ = T;
        minOutput_ = minOutput;
        maxOutput_ = maxOutput;
        prevError_ = 0;
        integral_ = 0;
    }

    double update(double setpoint, double processVariable, double dt)
    {
        double error = setpoint - processVariable;
        integral_ += (error * dt);

        // Anti-windup: Limit the integral term
        if (integral_ > maxOutput_)
            integral_ = maxOutput_;
        else if (integral_ < minOutput_)
            integral_ = minOutput_;

        double derivative = (error - prevError_) / dt;

        double output = kp_ * (1 + dt / (tau_ + T_)) * error + ki_ * (dt / T_) * integral_ - kd_ * derivative;

        // Output clamping: Limit the output within the specified range
        if (output > maxOutput_)
            output = maxOutput_;
        else if (output < minOutput_)
            output = minOutput_;

        prevError_ = error;

        return output;
    }

    void reset()
    {
        prevError_ = 0;
        integral_ = 0;
    }

    void set_gains(double kp, double ki, double kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double tau_;
    double T_;
    double minOutput_;
    double maxOutput_;
    double prevError_;
    double integral_;
};

#endif // PID_HXX
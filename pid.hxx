
#ifndef PID_HXX
#define PID_HXX

#include <cmath>

class PID
{
public:
    PID() {}
    ~PID() {}

    void init(float kp, float ki, float kd, float tau, float T, float minOutput, float maxOutput, float deadband)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        tau_ = tau;
        T_ = T;
        minOutput_ = minOutput;
        maxOutput_ = maxOutput;
        deadband_ = deadband;
        prevError_ = 0.0f;
        integral_ = 0.0f;
    }

    float update(float setpoint, float processVariable, float dt)
    {
        if (dt == 0.0f)
            return 0.0f;

        float error = setpoint - processVariable;



        integral_ += (error * dt);

        if(std::isnan(integral_))
        {
        	integral_ = 0.0f;
        }

        // Anti-windup: Limit the integral term
        if (integral_ > maxOutput_)
            integral_ = maxOutput_;
        else if (integral_ < minOutput_)
            integral_ = minOutput_;

        // Deadband: If the error is within the deadband, set the output to zero
        if(std::abs(error) < deadband_)
        {
        	integral_ = 0.0f; // Reset integral term within the deadband
        }

        float derivative = (error - prevError_) / dt;

        float output = kp_ * (1 + dt / (tau_ + T_)) * error + ki_ * (dt / T_) * integral_ - kd_ * derivative;

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
        prevError_ = 0.0f;
        integral_ = 0.0f;
    }

    void set_gains(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

private:
    float kp_;
    float ki_;
    float kd_;
    float tau_;
    float T_;
    float minOutput_;
    float maxOutput_;
    float deadband_;
    float prevError_;
    float integral_;
};

#endif // PID_HXX

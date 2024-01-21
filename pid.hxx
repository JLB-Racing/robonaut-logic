#ifndef PID_HXX
#define PID_HXX

#include <algorithm>
#include <cmath>
#include <limits>

struct DebugOutput
{
    float derivative;
    float integral;
    float prev_error;
};

class PID
{
public:
    PID(float kp, float ki, float kd, float tau, float T, float minOutput, float maxOutput, float deadband, float derivativeFilterAlpha)
        : kp_(kp),
          ki_(ki),
          kd_(kd),
          tau_(tau),
          T_(T),
          minOutput_(minOutput),
          maxOutput_(maxOutput),
          deadband_(deadband),
          derivativeFilterAlpha_(derivativeFilterAlpha)
    {
    }

    ~PID() {}

    void init(float kp, float ki, float kd, float tau, float T, float minOutput, float maxOutput, float deadband, float derivativeFilterAlpha)
    {
        kp_                    = kp;
        ki_                    = ki;
        kd_                    = kd;
        tau_                   = tau;
        T_                     = T;
        minOutput_             = minOutput;
        maxOutput_             = maxOutput;
        deadband_              = deadband;
        derivativeFilterAlpha_ = derivativeFilterAlpha;
        prevError_             = 0.0f;
        integral_              = 0.0f;
        derivative_            = 0.0f;
    }

    void update_params(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    float update(float setpoint, float processVariable, float dt)
    {
        // Small value to avoid division by zero
        const float epsilon = std::numeric_limits<float>::epsilon();

        // Check if the time step is very close to zero, return zero output to avoid numerical issues
        if (std::abs(dt) < epsilon) { return 0.0f; }

        // Calculate the current error between the setpoint and the process variable
        float error = setpoint - processVariable;

        // Check if a zero-crossing has occurred (change in sign of error)
        bool zeroCrossed = (prevError_ * error) < 0.0f;

        // Update the integral term if no zero-crossing has occurred
        if (!zeroCrossed)
        {
            // Calculate the integral term without considering saturation
            float unsaturatedIntegral = integral_;

            // Integrate positive error if within bounds
            if (error > 0.0f && unsaturatedIntegral < maxOutput_) { unsaturatedIntegral += (error * dt); }
            // Integrate negative error if within bounds
            else if (error < 0.0f && unsaturatedIntegral > minOutput_) { unsaturatedIntegral += (error * dt); }

            // Handle NaN case: Reset unsaturated integral term to zero
            if (std::isnan(unsaturatedIntegral)) { unsaturatedIntegral = 0.0f; }

            // Calculate the adjustment needed due to saturation
            [[maybe_unused]] float deltaIntegral = unsaturatedIntegral - integral_;

            // Back-calculation: Adjust the integral term based on the impact of saturation
            integral_ += (error * dt) /* - deltaIntegral*/;
        }
        else
        {
            // Zero-crossing deadband: Do not update the integral term
        }

        // Anti-windup: Limit the integral term to the specified range
        integral_ = std::clamp(integral_, minOutput_, maxOutput_);

        // Deadband: Scale the integral term based on the proximity to the deadband
        // float deadbandFactor = 1.0f - std::min(1.0f, std::abs(error) / (deadband_ + epsilon));
        // integral_ *= deadbandFactor;

        // Calculate the derivative term with low-pass filtering
        derivative_ = (1.0f - derivativeFilterAlpha_) * derivative_ + derivativeFilterAlpha_ * (error - prevError_) / dt;

        // Calculate the PID controller output using proportional, integral, and derivative terms
        float output = kp_ * (1 + dt / (tau_ + T_ + epsilon)) * error + ki_ * (dt / (T_ + epsilon)) * integral_ - kd_ * derivative_;

        // Output clamping: Limit the output within the specified range
        output = std::clamp(output, minOutput_, maxOutput_);

        // Update the previous error for the next iteration
        prevError_ = error;

        // Return the calculated PID controller output
        return output;
    }

    void reset()
    {
        prevError_  = 0.0f;
        integral_   = 0.0f;
        derivative_ = 0.0f;
    }

    void set_gains(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    DebugOutput get_debug() { return {derivative_, integral_, prevError_}; }

private:
    // params
    float kp_                    = 1.0f;
    float ki_                    = 0.0f;
    float kd_                    = 0.0f;
    float tau_                   = 0.05f;
    float T_                     = 0.005f;
    float minOutput_             = 0.0f;
    float maxOutput_             = 1.0f;
    float deadband_              = 0.0f;
    float derivativeFilterAlpha_ = 0.0f;

    // vars
    float prevError_  = 0.0f;
    float integral_   = 0.0f;
    float derivative_ = 0.0f;
};

#endif  // PID_HXX

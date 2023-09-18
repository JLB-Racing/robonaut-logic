#ifndef LOGIC_HXX
#define LOGIC_HXX

#include "vehicle_model.hxx"

namespace jlb
{
    class Controller
    {
    public:
        Controller() {}
        ~Controller() {}

        double lateral_control([[maybe_unused]] const rsim::vmodel::State &state, [[maybe_unused]] bool line_sensor[rsim::smodel::SENSOR_WIDTH])
        {
            // Constants for the control law
            constexpr double Kp = 0.8;  // Proportional gain
            constexpr double Ki = 0.01; // Integral gain
            constexpr double Kd = 0.6;  // Derivative gain

            // closest false to the center
            int closest = 0;
            for (int i = 0; i < rsim::smodel::SENSOR_WIDTH; i++)
            {
                if (!line_sensor[i] && std::abs(i - 8) < std::abs(closest - 8))
                {
                    closest = i;
                }
            }

            // rightmost false
            int rightmost = 0;
            for (int i = 0; i < rsim::smodel::SENSOR_WIDTH; i++)
            {
                if (!line_sensor[i] && i > rightmost)
                {
                    rightmost = i;
                }
            }

            int leftmost = 15;
            for (int i = 0; i < rsim::smodel::SENSOR_WIDTH; i++)
            {
                if (!line_sensor[i] && i < leftmost)
                {
                    leftmost = i;
                }
            }

            double error = (leftmost - rsim::smodel::SENSOR_WIDTH / 2) / static_cast<double>(rsim::smodel::SENSOR_WIDTH / 2.0);

            // Initialize PID control variables
            static double integral = 0.0;
            static double prev_error = 0.0;

            // Calculate PID terms
            double proportional_term = Kp * error;
            integral += Ki * error * rsim::vmodel::DELTA_T;
            double derivative_term = Kd * (error - prev_error) / rsim::vmodel::DELTA_T;

            // Calculate the target wheel angle based on the control law
            double target_angle = proportional_term + integral + derivative_term;

            std::cout << "target_angle: " << target_angle << std::endl;

            // Limit the target angle to the maximum allowed value
            // target_angle = std::max(-MAX_WHEEL_ANGLE, std::min(MAX_WHEEL_ANGLE, target_angle));

            // Update the previous error
            prev_error = error;

            return target_angle;
        }
    };

} // namespace

#endif // LOGIC_HXX
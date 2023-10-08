#ifndef LOGIC_HXX
#define LOGIC_HXX

#include "odometry.hxx"
#include "controller.hxx"
#include "common.hxx"
#include "signals.hxx"

namespace jlb
{

    class Logic
    {
    public:
        Controller controller;
        Odometry odometry;

        Logic(Direction direction_ = Direction::STRAIGHT, const float x_t_ = 0.0f, const float y_t_ = 0.0f, const float theta_t_ = 0.0f) : controller(direction_), odometry(x_t_, y_t_, theta_t_) {}

        void send_telemetry()
        {
            char msg[5] = {0};
            // Value::packet_from_value(msg, 5, signal_library[TARGET_ANGLE_ID], controller.target_angle);
            // signal_sender.send(msg, 5);
            // Value::packet_from_value(msg, 5, signal_library[TARGET_SPEED_ID], controller.target_speed);
            // signal_sender.send(msg, 5);
            Value::packet_from_value(msg, 5, signal_library[POSITION_X_ID], odometry.x_t);
            signal_sender.send(msg, 5);
            Value::packet_from_value(msg, 5, signal_library[POSITION_Y_ID], odometry.y_t);
            signal_sender.send(msg, 5);
            Value::packet_from_value(msg, 5, signal_library[POSITION_THETA_ID], odometry.theta_t);
            signal_sender.send(msg, 5);
            // Value::packet_from_value(msg, 5, signal_library[ODOM_LINEAR_VELOCITY_X_ID], odometry.vx_t);
            // signal_sender.send(msg, 5);
            // Value::packet_from_value(msg, 5, signal_library[ODOM_ANGULAR_VELOCITY_Z_ID], odometry.w_t);
            // signal_sender.send(msg, 5);
            // Value::packet_from_value(msg, 5, signal_library[MEAS_MOTOR_RPM_ID], odometry.meas_motor_rpm);
            // signal_sender.send(msg, 5);
            // Value::packet_from_value(msg, 5, signal_library[MEAS_ANGULAR_VELOCITY_Z_ID], odometry.meas_ang_vel_z);
            // signal_sender.send(msg, 5);

            for (uint8_t i = 0; i < SENSOR_WIDTH; i++)
            {
                if (i == controller.selected)
                {
                    Value::packet_from_value(msg, 5, signal_library[LINE_SENSOR_1_ID + i], controller.detection[i] + 2.0f);
                }
                else
                {
                    Value::packet_from_value(msg, 5, signal_library[LINE_SENSOR_1_ID + i], controller.detection[i]);
                }
                signal_sender.send(msg, 5);
            }
        }

    private:
        SignalLibrary &signal_library = SignalLibrary::get_instance();
        SignalSender &signal_sender = SignalSender::get_instance();
    };

} // namespace jlb

#endif // LOGIC_HXX
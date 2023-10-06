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
            Value::packet_from_value(msg, 5, signal_library[TARGET_ANGLE_ID], controller.target_angle);
            signal_sender.send(msg, 5);
            Value::packet_from_value(msg, 5, signal_library[TARGET_SPEED_ID], controller.target_speed);
            signal_sender.send(msg, 5);
        }

    private:
        SignalLibrary &signal_library = SignalLibrary::get_instance();
        SignalSender &signal_sender = SignalSender::get_instance();
    };

} // namespace jlb

#endif // LOGIC_HXX
#ifndef LOGIC_HXX
#define LOGIC_HXX

#include "odometry.hxx"
#include "controller.hxx"
#include "as_state.hxx"
#include "common.hxx"
#include "signals.hxx"

namespace jlb
{

    class Logic
    {
    public:
        Odometry odometry;
        Controller controller;
        ASState as_state;
        SignalSender signal_sender = SignalSender(odometry, controller, as_state);

        Logic(Direction direction_ = Direction::STRAIGHT, const float x_t_ = 0.0f, const float y_t_ = 0.0f, const float theta_t_ = 0.0f) : odometry(x_t_, y_t_, theta_t_), controller(direction_) {}

    private:
    };

} // namespace jlb

#endif // LOGIC_HXX
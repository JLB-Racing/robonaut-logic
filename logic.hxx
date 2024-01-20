#ifndef LOGIC_HXX
#define LOGIC_HXX

#include <stdexcept>

#include "as_state.hxx"
#include "common.hxx"
#include "controller.hxx"
#include "graph.hxx"
#include "measurements.hxx"
#include "odometry.hxx"
#include "signals.hxx"

namespace jlb
{
    class Logic
    {
    public:
        Logic(Direction direction_ = Direction::STRAIGHT, const float x_t_ = 0.0f, const float y_t_ = 0.0f, const float theta_t_ = 0.0f)
            : odometry(x_t_, y_t_, theta_t_), controller(direction_)
        {
        }

        ControlSignal update()
        {
            auto [vx, x, y, theta] = odometry.update_odom();
            controller.set_current_velocity(vx);

            auto [mission, labyrinth_state, fast_state, reference_speed] = as_state.update();
            controller.set_reference_speed(reference_speed);

            if (as_state.labyrinth_state == LabyrinthState::MISSION_SWITCH)
            {
                auto [target_angle, target_speed] = controller.update_mission_switch();
                return ControlSignal{target_angle, target_speed};
            }
            else if (as_state.labyrinth_state == LabyrinthState::STANDBY) { return ControlSignal{0.0f, 0.0f}; }
            else if (as_state.labyrinth_state == LabyrinthState::ERROR) { return ControlSignal{0.0f, 0.0f}; }
            else if (as_state.labyrinth_state == LabyrinthState::REVERSE_ESCAPE)
            {
                if (controller.target_speed < 0.0f) { controller.swap_front_rear(); }
                auto [target_angle, target_speed] = controller.update();
                return ControlSignal{target_angle, target_speed};
            }
            else
            {
                auto [target_angle, target_speed] = controller.update();
                return ControlSignal{target_angle, target_speed};
            }
        }

        void set_detection_front(bool *detection_front_, std::vector<float> line_positions_front_)
        {
            as_state.current_number_of_lines = static_cast<uint8_t>(line_positions_front_.size());
            controller.set_detection_front(detection_front_, line_positions_front_);
        }
        void set_detection_rear(bool *detection_rear_, std::vector<float> line_positions_rear_)
        {
            controller.set_detection_rear(detection_rear_, line_positions_rear_);
        }
        void set_under_gate(const bool under_gate_) { as_state.under_gate = under_gate_; }
        void set_at_cross_section(const bool at_cross_section_) { as_state.at_cross_section = at_cross_section_; }
        void imu_callback(const float yaw_rate_) { odometry.imu_callback(yaw_rate_); }
        void rpm_callback(const float motor_rpm_) { odometry.rpm_callback(motor_rpm_); }
        void pirate_callback(const char prev_node_, const char next_node_, const char after_next_node_, const int section_percentage_)
        {
            as_state.pirate_callback(prev_node_, next_node_, after_next_node_, section_percentage_);
        }
        void set_object_range(const float object_range_) { controller.set_object_range(object_range_); }
        void set_states(const CompositeState state_) { as_state.set_states(state_); }
        void send_telemetry() { signal_sender.send_telemetry(); }
        void set_measurements(const Measurements &measurements_) { measurements = measurements_; }
        void set_flood(const bool flood_)
        {
            if (flood_ && as_state.labyrinth_state == LabyrinthState::EXPLORING)
            {
                Edge::flood              = true;
                as_state.labyrinth_state = LabyrinthState::FLOOD_TO_BALANCER;
            }
            else if (!flood_ && as_state.labyrinth_state == LabyrinthState::FLOOD_SOLVING)
            {
                Edge::flood              = false;
                as_state.labyrinth_state = LabyrinthState::EXPLORING;
            }
        }
        Odom get_odometry() { return {odometry.vx_t, odometry.x_t, odometry.y_t, odometry.theta_t}; }

    private:
        Odometry     odometry;
        Controller   controller;
        Graph        graph;
        Measurements measurements;
        ASState      as_state      = ASState(odometry, controller, graph);
        SignalSender signal_sender = SignalSender(odometry, controller, as_state, graph, measurements);
    };

}  // namespace jlb

#endif  // LOGIC_HXX

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
        Logic() : odometry(START_X, START_Y, START_ORIENTATION) {}

        ControlSignal update()
        {
            auto [vx, x, y, theta, distance_local] = odometry.update_odom();
            controller.set_current_velocity(vx);

            auto [mission, labyrinth_state, mission_switch_state, fast_state, reference_speed] = as_state.update();
            controller.set_target_speed(reference_speed);

            switch (mission)
            {
                case Mission::LABYRINTH:
                {
                    if (labyrinth_state == LabyrinthState::MISSION_SWITCH)
                    {
                        controller.set_direction(Direction::STRAIGHT);

                        if (mission_switch_state == MissionSwitchState::STANDBY)
                        {
                            controller.set_target_speed(0.0f);
                            auto [target_angle, target_speed] = controller.update(as_state.follow_car);
                            return ControlSignal{target_angle, target_speed};
                        }
                        else if (mission_switch_state == MissionSwitchState::FIRST_FORWARD)
                        {
                            auto [target_angle, target_speed] = controller.update(as_state.follow_car);
                            return ControlSignal{target_angle, target_speed};
                        }
                        else if (mission_switch_state == MissionSwitchState::FIRST_TURN)
                        {
                            auto [target_angle, target_speed] = controller.update(as_state.follow_car);
                            if (MISSION_SWITCH_DIRECTION == Direction::RIGHT)
                            {
                                return ControlSignal{as_state.mission_switch_steering_angle, target_speed};
                            }
                            else if (MISSION_SWITCH_DIRECTION == Direction::LEFT)
                            {
                                return ControlSignal{-as_state.mission_switch_steering_angle, target_speed};
                            }
                        }
                        else if (mission_switch_state == MissionSwitchState::SECOND_TURN)
                        {
                            auto [target_angle, target_speed] = controller.update(as_state.follow_car);

                            if (controller.line_positions_front.size() > 0)
                            {
                                auto [target_angle, target_speed] = controller.update(as_state.follow_car);
                                return ControlSignal{target_angle, target_speed};
                            }

                            if (MISSION_SWITCH_DIRECTION == Direction::RIGHT)
                            {
                                return ControlSignal{-as_state.mission_switch_steering_angle, target_speed};
                            }
                            else if (MISSION_SWITCH_DIRECTION == Direction::LEFT)
                            {
                                return ControlSignal{as_state.mission_switch_steering_angle, target_speed};
                            }
                        }
                        else if (mission_switch_state == MissionSwitchState::SECOND_FORWARD)
                        {
                            auto [target_angle, target_speed] = controller.update(as_state.follow_car);
                            return ControlSignal{target_angle, target_speed};
                        }
                        else { return ControlSignal{0.0f, 0.0f}; }
                    }
                    else if (labyrinth_state == LabyrinthState::FLOOD_SOLVING)
                    {
                        auto [target_angle, target_speed] = controller.update_balancer();
                        return ControlSignal{target_angle, target_speed};
                    }
                    else if (labyrinth_state == LabyrinthState::STANDBY || labyrinth_state == LabyrinthState::ERROR)
                    {
                        controller.set_target_speed(0.0f);
                        auto [target_angle, target_speed] = controller.update(as_state.follow_car);

                        return ControlSignal{target_angle, target_speed};
                    }
                    else if (labyrinth_state == LabyrinthState::REVERSE_ESCAPE || labyrinth_state == LabyrinthState::FLOOD_TO_LABYRINTH)
                    {
                        if (controller.target_speed < 0.0f && odometry.vx_t < 0.0f) { controller.swap_front_rear(); }
                        auto [target_angle, target_speed] = controller.update(as_state.follow_car);
                        return ControlSignal{target_angle, target_speed};
                    }
                    else
                    {
                        auto [target_angle, target_speed] = controller.update(as_state.follow_car);
                        return ControlSignal{target_angle, target_speed};
                    }
                    break;
                }
                case Mission::FAST:
                {
#ifdef TEST_REVERSE
                    controller.set_direction(Direction::STRAIGHT);
                	controller.swap_front_rear();
					controller.target_speed = -LABYRINTH_SPEED_REVERSE;
					auto [target_angle, target_speed] = controller.update(as_state.follow_car, true);
					return ControlSignal{target_angle, target_speed};
#endif
                    controller.set_direction(Direction::STRAIGHT);

                    if (fast_state == FastState::OUT_BRAKE_ZONE)
                    {
                        auto [target_angle, target_speed] = controller.update(as_state.follow_car, false);
                        return ControlSignal{target_angle, target_speed};
                    }
                    else
                    {
                        auto [target_angle, target_speed] = controller.update(as_state.follow_car, true);
                        return ControlSignal{target_angle, target_speed};
                    }

                    break;
                }
                case Mission::STANDBY:
                {
                    return ControlSignal{0.0f, 0.0f};
                    break;
                }
                default:
                {
                    auto [target_angle, target_speed] = controller.update(as_state.follow_car);
                    return ControlSignal{target_angle, target_speed};
                    break;
                }
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
        void imu_callback(
            const float ang_vel_x, const float ang_vel_y, const float ang_vel_z, const float lin_acc_x, const float lin_acc_y, const float lin_acc_z)
        {
            odometry.imu_callback(ang_vel_x, ang_vel_y, ang_vel_z, lin_acc_x, lin_acc_y, lin_acc_z);
        }
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
            Edge::flood    = flood_;
            as_state.flood = flood_;
        }
        Odom get_odometry() { return {odometry.vx_t, odometry.x_t, odometry.y_t, odometry.theta_t, odometry.distance_local}; }
        void start_signal()
        {
#if defined(TEST_FAST) || defined(TEST_REVERSE)
            if (as_state.mission == Mission::STANDBY) { as_state.mission = Mission::FAST; }
#else
            if (as_state.mission == Mission::STANDBY) { as_state.mission = Mission::LABYRINTH; }
#endif
        }
        void reset_signal(const CompositeState state_)
        {
            graph.reset();
            as_state.reset(state_);
            as_state.mission = Mission::STANDBY;
            controller       = Controller{};
            odometry         = Odometry{START_X, START_Y, START_ORIENTATION};
        }

        Odometry     odometry;
        Controller   controller;
        Graph        graph;
        Measurements measurements;
        ASState      as_state      = ASState(odometry, controller, graph);
        SignalSender signal_sender = SignalSender(odometry, controller, as_state, graph, measurements);
    };

}  // namespace jlb

#endif  // LOGIC_HXX

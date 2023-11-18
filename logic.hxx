#ifndef LOGIC_HXX
#define LOGIC_HXX

#include "odometry.hxx"
#include "controller.hxx"
#include "as_state.hxx"
#include "graph.hxx"
#include "common.hxx"
#include "signals.hxx"
#include <stdexcept>

namespace jlb
{
    class Logic
    {
    public:
        Odometry odometry;
        Controller controller;
        ASState as_state;
        Graph graph;

        SignalSender signal_sender = SignalSender(odometry, controller, as_state, graph);

        Logic(Direction direction_ = Direction::STRAIGHT, const float x_t_ = 0.0f, const float y_t_ = 0.0f, const float theta_t_ = 0.0f) : odometry(x_t_, y_t_, theta_t_), controller(direction_) {}

        ControlSignal update()
        {
            auto [vx, x, y, theta] = odometry.update_odom();
            controller.set_current_velocity(vx);

            bool at_decision_point = as_state.under_gate || as_state.at_cross_section;

            if (!as_state.prev_at_decision_point && at_decision_point)
            {
                float inaccuracy = 0.25f;
                if (std::sqrt(std::pow(graph[as_state.next_node].x - odometry.x_t, 2) + std::pow(graph[as_state.next_node].y - odometry.y_t, 2)) < inaccuracy)
                {
                    auto at_node = as_state.next_node;

                    while (true)
                    {
                        unsigned long num_neighbors = graph[at_node].edges.size();
                        auto selected_edge = rand() % num_neighbors;

                        if (graph[at_node].edges[selected_edge].node == 'P' ||
                            graph[at_node].edges[selected_edge].node == 'U' ||
                            graph[at_node].edges[selected_edge].node == 'X')
                        {
                            continue;
                        }

                        auto prev_nodes = graph[at_node].edges[selected_edge].prev_nodes;
                        if (std::find(prev_nodes.begin(), prev_nodes.end(), as_state.previous_node) != prev_nodes.end())
                        {
                            as_state.next_node = graph[at_node].edges[selected_edge].node;
                            as_state.previous_node = at_node;

                            controller.set_direction(graph[at_node].edges[selected_edge].direction);
                            odometry.correction(graph[as_state.previous_node].x, graph[as_state.previous_node].y);

                            break;
                        }
                    }
#ifndef STM32
                    switch (controller.direction)
                    {
                    case Direction::LEFT:
                        std::cout << "[C] at: " << as_state.previous_node << " to: " << as_state.next_node << " dir: left" << std::endl;
                        break;
                    case Direction::RIGHT:
                        std::cout << "[C] at: " << as_state.previous_node << " to: " << as_state.next_node << " dir: right" << std::endl;
                        break;
                    case Direction::STRAIGHT:
                        std::cout << "[C] at: " << as_state.previous_node << " to: " << as_state.next_node << " dir: straight" << std::endl;
                        break;
                    default:
                        break;
                    }
#endif
                }
            }

            as_state.prev_at_decision_point = at_decision_point;

            auto [target_angle, target_speed] = controller.update();
            return ControlSignal{target_angle, target_speed};
        }

        void set_under_gate(const bool under_gate_)
        {
            as_state.under_gate = under_gate_;
        }

        void set_at_cross_section(const bool at_cross_section_)
        {
            as_state.at_cross_section = at_cross_section_;
        }

    private:
    };

} // namespace jlb

#endif // LOGIC_HXX

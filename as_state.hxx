#ifndef AS_STATE_HXX
#define AS_STATE_HXX

#include <chrono>

#include "common.hxx"
#include "controller.hxx"
#include "graph.hxx"
#include "odometry.hxx"

namespace jlb
{

    struct CompositeState
    {
        Mission        mission;
        LabyrinthState labyrinth_state;
        FastState      fast_state;
        float          reference_speed = 0.0f;

        CompositeState(FastState fast_state_) : mission{Mission::FAST}, labyrinth_state{LabyrinthState::START}, fast_state{fast_state_}, reference_speed{0.0f} {}
        CompositeState(LabyrinthState labyrinth_state_) : mission{Mission::LABYRINTH}, labyrinth_state{labyrinth_state_}, fast_state{FastState::FOLLOW_SAFETY_CAR}, reference_speed{0.0f} {}
        CompositeState(Mission mission_, LabyrinthState labyrinth_state_, FastState fast_state_, float reference_speed_)
            : mission{mission_}, labyrinth_state{labyrinth_state_}, fast_state{fast_state_}, reference_speed{reference_speed_}
        {
        }
    };

    class ASState
    {
    public:
        Mission        mission         = Mission::LABYRINTH;
        LabyrinthState labyrinth_state = LabyrinthState::START;
        FastState      fast_state      = FastState::FOLLOW_SAFETY_CAR;
        float          reference_speed = 0.0f;

        bool    under_gate               = false;
        bool    at_cross_section         = false;
        bool    prev_at_decision_point   = false;
        uint8_t current_number_of_lines  = 0u;
        float   state_time               = 0.0f;
        float   state_transition_time    = 0.0f;
        bool    started_state_transition = false;
        uint32_t tick_counter = 0u;
        uint32_t tick_counter_prev = 0u;

        [[maybe_unused]] char          previous_node = 'U';
        [[maybe_unused]] char          next_node     = 'U';
        [[maybe_unused]] unsigned long selected_edge = 0u;

        ASState(Odometry& odometry_, Controller& controller_, Graph& graph_) : odometry{odometry_}, controller{controller_}, graph{graph_} {}

        void set_states(const CompositeState state_)
        {
            mission         = state_.mission;
            labyrinth_state = state_.labyrinth_state;
            fast_state      = state_.fast_state;
        }

        CompositeState update()
        {
#ifndef SIMULATION
            // TODO: add timestamp
        	tick_counter_prev = tick_counter;
        	tick_counter = HAL_GetTick();
            float dt = (((float)tick_counter) - ((float)(tick_counter_prev))) / 1000.0f;
#else
            auto                   update_timestamp_ = std::chrono::steady_clock::now();
            [[maybe_unused]] float dt                = std::chrono::duration_cast<std::chrono::milliseconds>(update_timestamp_ - prev_update_timestamp_).count() / 1000.0f;
            prev_update_timestamp_                   = update_timestamp_;
#endif

            state_transition_time += dt;
            state_time += dt;

            switch (mission)
            {
                case Mission::LABYRINTH:
                {
                    bool at_decision_point = under_gate || at_cross_section;

                    if (!prev_at_decision_point && at_decision_point)
                    {
                        auto distance = graph[previous_node].edges[selected_edge].weight;
                        if (std::fabs(distance - odometry.distance_local) < LOCALIZATION_INACCURACY || labyrinth_state == LabyrinthState::START)
                        {
                            if (labyrinth_state == LabyrinthState::START) { labyrinth_state = LabyrinthState::EXPLORING; }
                            auto at_node = next_node;

                            while (true)
                            {
                                unsigned long num_neighbors           = graph[at_node].edges.size();
                                auto          selected_edge_candidate = rand() % num_neighbors;

                                if (graph[at_node].edges[selected_edge_candidate].node == 'P' || graph[at_node].edges[selected_edge_candidate].node == 'U' ||
                                    graph[at_node].edges[selected_edge_candidate].node == 'X')
                                {
                                    continue;
                                }

                                auto prev_nodes = graph[at_node].edges[selected_edge_candidate].prev_nodes;
                                if (std::find(prev_nodes.begin(), prev_nodes.end(), previous_node) != prev_nodes.end())
                                {
                                    selected_edge = selected_edge_candidate;
                                    next_node     = graph[at_node].edges[selected_edge].node;
                                    previous_node = at_node;

                                    controller.set_direction(graph[at_node].edges[selected_edge].direction);
                                    odometry.correction(graph[previous_node].x, graph[previous_node].y);

#ifdef SIMULATION
                                    switch (controller.direction)
                                    {
                                        case Direction::LEFT:
                                            std::cout << "[C] at: " << previous_node << " to: " << next_node << " dir: left" << std::endl;
                                            break;
                                        case Direction::RIGHT:
                                            std::cout << "[C] at: " << previous_node << " to: " << next_node << " dir: right" << std::endl;
                                            break;
                                        case Direction::STRAIGHT:
                                            std::cout << "[C] at: " << previous_node << " to: " << next_node << " dir: straight" << std::endl;
                                            break;
                                        default:
                                            break;
                                    }
#endif
                                    break;
                                }
                            }
                        }
                    }

                    prev_at_decision_point = at_decision_point;

                    reference_speed = LABYRINTH_SPEED;

                    break;
                }
                case Mission::FAST:
                {
                    switch (fast_state)
                    {
                        case FastState::FOLLOW_SAFETY_CAR:
                        {
                            reference_speed = FAST_SPEED_SAFETY_CAR;
                            break;
                        }
                        case FastState::OVERTAKE_SAFETY_CAR_START:
                        {
                            reference_speed = FAST_SPEED_OVERTAKE;
                            break;
                        }
                        case FastState::OVERTAKE_SAFETY_CAR_END:
                        {
                            reference_speed = FAST_SPEED;
                            break;
                        }
                        case FastState::IN_ACCEL_ZONE:
                        {
                            if (current_number_of_lines == 1u && !started_state_transition)
                            {
                                started_state_transition = true;
                                state_transition_time    = 0.0f;
                            }
                            else if (current_number_of_lines != 1u && started_state_transition) { started_state_transition = false; }

                            if (started_state_transition && state_transition_time > STATE_TRANSITION_TIME_LIMIT && state_time > STATE_MIN_TIME)
                            {
                                fast_state = FastState::OUT_ACCEL_ZONE;
                                state_time = 0.0f;
                                break;
                            }

                            reference_speed = FAST_SPEED;
                            break;
                        }
                        case FastState::OUT_ACCEL_ZONE:
                        {
                            if (current_number_of_lines == 3u && !started_state_transition)
                            {
                                started_state_transition = true;
                                state_transition_time    = 0.0f;
                            }
                            else if (current_number_of_lines != 3u && started_state_transition) { started_state_transition = false; }

                            if (started_state_transition && state_transition_time > STATE_TRANSITION_TIME_LIMIT && state_time > STATE_MIN_TIME)
                            {
                                fast_state = FastState::IN_BRAKE_ZONE;
                                state_time = 0.0f;
                                break;
                            }

                            reference_speed = FAST_SPEED;
                            break;
                        }
                        case FastState::IN_BRAKE_ZONE:
                        {
                            if (current_number_of_lines == 1u && !started_state_transition)
                            {
                                started_state_transition = true;
                                state_transition_time    = 0.0f;
                            }
                            else if (current_number_of_lines != 1u && started_state_transition) { started_state_transition = false; }

                            if (started_state_transition && state_transition_time > STATE_TRANSITION_TIME_LIMIT && state_time > STATE_MIN_TIME)
                            {
                                fast_state = FastState::OUT_BRAKE_ZONE;
                                state_time = 0.0f;
                                break;
                            }

                            reference_speed = FAST_SPEED_TURN;
                            break;
                        }
                        case FastState::OUT_BRAKE_ZONE:
                        {
                            if (current_number_of_lines == 3u && !started_state_transition)
                            {
                                started_state_transition = true;
                                state_transition_time    = 0.0f;
                            }
                            else if (current_number_of_lines != 3u && started_state_transition) { started_state_transition = false; }

                            if (started_state_transition && state_transition_time > STATE_TRANSITION_TIME_LIMIT && state_time > STATE_MIN_TIME)
                            {
                                fast_state = FastState::IN_ACCEL_ZONE;
                                state_time = 0.0f;
                                break;
                            }

                            reference_speed = FAST_SPEED_TURN;
                            break;
                        }
                        default:
                        {
                            // this should never happen
                            break;
                        }
                    }

                    break;
                }
                default:
                {
                    // this should never happen
                    break;
                }
            }

            return CompositeState{mission, labyrinth_state, fast_state, reference_speed};
        }

    private:
        [[maybe_unused]] Odometry&   odometry;
        [[maybe_unused]] Controller& controller;
        [[maybe_unused]] Graph&      graph;

#ifndef SIMULATION
        // TODO: add timestamp
#else
        std::chrono::time_point<std::chrono::steady_clock> prev_update_timestamp_ = std::chrono::steady_clock::now();
#endif
    };

}  // namespace jlb

#endif  // AS_STATE_HXX

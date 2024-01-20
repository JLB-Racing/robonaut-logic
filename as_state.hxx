#ifndef AS_STATE_HXX
#define AS_STATE_HXX

#include <chrono>
#include <iostream>

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

        CompositeState(FastState fast_state_)
            : mission{Mission::FAST}, labyrinth_state{LabyrinthState::START}, fast_state{fast_state_}, reference_speed{0.0f}
        {
        }
        CompositeState(LabyrinthState labyrinth_state_)
            : mission{Mission::LABYRINTH}, labyrinth_state{labyrinth_state_}, fast_state{FastState::FOLLOW_SAFETY_CAR}, reference_speed{0.0f}
        {
        }
        CompositeState(Mission mission_, LabyrinthState labyrinth_state_, FastState fast_state_, float reference_speed_)
            : mission{mission_}, labyrinth_state{labyrinth_state_}, fast_state{fast_state_}, reference_speed{reference_speed_}
        {
        }
    };

    class ASState
    {
    public:
        Mission        mission              = Mission::LABYRINTH;
        LabyrinthState labyrinth_state      = LabyrinthState::START;
        LabyrinthState prev_labyrinth_state = LabyrinthState::START;
        FastState      fast_state           = FastState::FOLLOW_SAFETY_CAR;
        float          reference_speed      = 0.0f;

        bool     under_gate               = false;
        bool     at_cross_section         = false;
        bool     prev_at_decision_point   = false;
        uint8_t  current_number_of_lines  = 0u;
        float    state_time               = 0.0f;
        float    state_transition_time    = 0.0f;
        bool     started_state_transition = false;
        uint32_t tick_counter             = 0u;
        uint32_t tick_counter_prev        = 0u;

        char          at_node       = 'U';
        char          previous_node = 'U';
        char          next_node     = 'U';
        char          goal_node     = 'U';
        unsigned long selected_edge = 0u;

        Direction reverse_saved_dir = Direction::STRAIGHT;

        ASState(Odometry& odometry_, Controller& controller_, Graph& graph_) : odometry{odometry_}, controller{controller_}, graph{graph_} {}

        void set_states(const CompositeState state_)
        {
            mission              = state_.mission;
            prev_labyrinth_state = labyrinth_state;
            labyrinth_state      = state_.labyrinth_state;
            fast_state           = state_.fast_state;
        }

        void pirate_callback(const char prev_node_, const char next_node_, const char after_next_node_, const int section_percentage_)
        {
            graph.pirate_callback(prev_node_, next_node_, after_next_node_, section_percentage_);
        }

        void apply_path(const DijkstraResult& result)
        {
            goal_node = result.node;
            next_node = result.path[1];

            for (unsigned long i = 0; i < graph[at_node].edges.size(); i++)
                if (graph[at_node].edges[i].to == next_node) selected_edge = i;

            controller.set_direction(graph[at_node].edges[selected_edge].direction);
            odometry.correction(graph[at_node].x, graph[at_node].y);

            previous_node = at_node;
        }

        void apply_path_reverse(const DijkstraResult& result)
        {
            goal_node = result.node;
            next_node = result.path[1];

            for (unsigned long i = 0; i < graph[at_node].edges.size(); i++)
                if (graph[at_node].edges[i].to == next_node) selected_edge = i;

            if (controller.target_speed < 0.0f && odometry.distance_local < WHEELBASE)
            {
                controller.set_direction(graph[at_node].edges[selected_edge].direction);
            }

            if (controller.target_speed < 0.0f && odometry.distance_local >= WHEELBASE) { controller.set_direction(reverse_saved_dir, true); }

            previous_node = at_node;
        }

        void exploring_callback()
        {
            if (std::find(std::begin(GATE_NAMES), std::end(GATE_NAMES), at_node) != std::end(GATE_NAMES) &&
                std::find(graph.collected_nodes.begin(), graph.collected_nodes.end(), at_node) == graph.collected_nodes.end())
            {
                graph.collected_nodes.push_back(at_node);
            }

            auto result = graph.Dijkstra(previous_node, at_node);

            if (result.weight == std::numeric_limits<float>::infinity())
            {
                if (graph.collected_nodes.size() == NUMBER_OF_GATES)
                {
                    for (auto& node : graph.nodes)
                    {
                        if (node.name == MISSION_SWITCH_NODE ||
                            std::find(std::begin(MISSION_SWITCH_PREV_NODES), std::end(MISSION_SWITCH_PREV_NODES), node.name) !=
                                std::end(MISSION_SWITCH_PREV_NODES))
                        {
                            continue;
                        }
                        node.remove_edge(MISSION_SWITCH_NODE);
                    }

                    prev_labyrinth_state = labyrinth_state;
                    labyrinth_state      = LabyrinthState::FINISHED;
                    finished_callback();
                }
                else
                {
                    prev_labyrinth_state = labyrinth_state;
                    labyrinth_state      = LabyrinthState::ESCAPE;
                    escape_callback();
                }
            }
            else if (result.weight >= WEIGHT_PENALTY * SAFETY_MARGIN && !Edge::pirate_intersecting(at_node))
            {
                if (result.path.size() > 1)
                {
                    auto new_result = graph.Dijkstra(at_node, result.path[1]);
                    if (std::fabs(result.weight - new_result.weight) < WEIGHT_PENALTY * SAFETY_MARGIN)
                    {
                        apply_path(result);
                        return;
                    }
                }

                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::STANDBY;
                standby_callback();
            }
            else if (result.weight >= WEIGHT_PENALTY * SAFETY_MARGIN && Edge::pirate_intersecting(at_node))
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::ESCAPE;
                escape_callback();
            }
            else { apply_path(result); }
        }

        void finished_callback()
        {
            auto result = graph.Dijkstra(previous_node, at_node, MISSION_SWITCH_NODE);

            if (result.weight == std::numeric_limits<float>::infinity())
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::ESCAPE;
                escape_callback();
            }
            else if (result.weight >= WEIGHT_PENALTY * SAFETY_MARGIN && !Edge::pirate_intersecting(at_node))
            {
                if (result.path.size() > 1)
                {
                    auto new_result = graph.Dijkstra(at_node, result.path[1], MISSION_SWITCH_NODE);
                    if (std::fabs(result.weight - new_result.weight) < WEIGHT_PENALTY * SAFETY_MARGIN)
                    {
                        apply_path(result);

                        if (at_node == goal_node)
                        {
                            prev_labyrinth_state = labyrinth_state;
                            labyrinth_state      = LabyrinthState::MISSION_SWITCH;
                        }

                        return;
                    }
                }

                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::STANDBY;
                standby_callback();
            }
            else if (result.weight >= WEIGHT_PENALTY * SAFETY_MARGIN && Edge::pirate_intersecting(at_node))
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::ESCAPE;
                escape_callback();
            }
            else
            {
                apply_path(result);

                if (at_node == goal_node)
                {
                    prev_labyrinth_state = labyrinth_state;
                    labyrinth_state      = LabyrinthState::MISSION_SWITCH;
                }
            }
        }

        void error_callback() {}

        void standby_callback()
        {
            if (Edge::pirate_intersecting(at_node))
            {
                labyrinth_state = LabyrinthState::ESCAPE;
                escape_callback();
            }
            else
            {
                if (prev_labyrinth_state == LabyrinthState::EXPLORING)
                {
                    auto result = graph.Dijkstra(previous_node, at_node);

                    if (result.weight == std::numeric_limits<float>::infinity())
                    {
                        labyrinth_state = LabyrinthState::ESCAPE;
                        escape_callback();
                    }
                    else if (result.weight < WEIGHT_PENALTY * SAFETY_MARGIN)
                    {
                        prev_labyrinth_state = labyrinth_state;
                        labyrinth_state      = LabyrinthState::EXPLORING;
                        exploring_callback();
                    }
                }
                else if (prev_labyrinth_state == LabyrinthState::FINISHED)
                {
                    auto result = graph.Dijkstra(previous_node, at_node, MISSION_SWITCH_NODE);

                    if (result.weight == std::numeric_limits<float>::infinity())
                    {
                        labyrinth_state = LabyrinthState::ESCAPE;
                        escape_callback();
                    }
                    else if (result.weight < WEIGHT_PENALTY * SAFETY_MARGIN)
                    {
                        prev_labyrinth_state = labyrinth_state;
                        labyrinth_state      = LabyrinthState::FINISHED;
                        finished_callback();
                    }
                }
            }
        }

        void escape_callback()
        {
            auto result = graph.Dijkstra(previous_node, at_node, '@', true);

            if (result.weight == std::numeric_limits<float>::infinity())
            {
                labyrinth_state = LabyrinthState::ERROR;
                error_callback();
            }
            else
            {
                apply_path(result);
                std::swap(prev_labyrinth_state, labyrinth_state);
            }
        }

        void reverse_escape_callback()
        {
            auto result = graph.Dijkstra(previous_node, at_node, '@', true);

            if (result.weight == std::numeric_limits<float>::infinity())
            {
                labyrinth_state = LabyrinthState::ERROR;
                error_callback();
            }
            else { apply_path_reverse(result); }
        }

        void mission_switch_callback()
        {
            // TODO: this is a placeholder
        }

        CompositeState update()
        {
#ifndef SIMULATION
            tick_counter_prev = tick_counter;
            tick_counter      = HAL_GetTick();
            float dt          = (((float)tick_counter) - ((float)(tick_counter_prev))) / 1000.0f;
#else
            auto                   update_timestamp_ = std::chrono::steady_clock::now();
            [[maybe_unused]] float dt =
                std::chrono::duration_cast<std::chrono::milliseconds>(update_timestamp_ - prev_update_timestamp_).count() / 1000.0f;
            prev_update_timestamp_ = update_timestamp_;
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
                        auto distance = graph[previous_node].edges[selected_edge].distance;

                        if (labyrinth_state == LabyrinthState::REVERSE_ESCAPE) { distance = graph[at_node].edges[selected_edge].distance; }

                        if (std::fabs(distance - std::fabs(odometry.distance_local)) < LOCALIZATION_INACCURACY ||
                            labyrinth_state == LabyrinthState::START)
                        {
                            if (labyrinth_state == LabyrinthState::START)
                            {
                                prev_labyrinth_state = labyrinth_state;
                                labyrinth_state      = LabyrinthState::EXPLORING;
                            }

                            at_node = next_node;

                            switch (labyrinth_state)
                            {
                                case LabyrinthState::EXPLORING:
                                {
                                    exploring_callback();
                                    break;
                                }
                                case LabyrinthState::FINISHED:
                                {
                                    finished_callback();
                                    break;
                                }
                                case LabyrinthState::REVERSE_ESCAPE:
                                {
                                    auto result = graph.Dijkstra(previous_node, at_node, '@', true);

                                    if (result.weight == std::numeric_limits<float>::infinity())
                                    {
                                        labyrinth_state = LabyrinthState::ERROR;
                                        error_callback();
                                        break;
                                    }
                                    else { previous_node = result.path[1]; }

                                    odometry.correction(graph[at_node].x, graph[at_node].y);
                                    controller.set_direction(reverse_saved_dir, true);

                                    if (prev_labyrinth_state == LabyrinthState::EXPLORING)
                                    {
                                        prev_labyrinth_state = labyrinth_state;
                                        labyrinth_state      = LabyrinthState::EXPLORING;
                                        exploring_callback();
                                    }
                                    else if (prev_labyrinth_state == LabyrinthState::FINISHED)
                                    {
                                        prev_labyrinth_state = labyrinth_state;
                                        labyrinth_state      = LabyrinthState::FINISHED;
                                        finished_callback();
                                    }

                                    odometry.distance_local += WHEELBASE;

                                    break;
                                }
                                default:
                                {
                                    break;
                                }
                            }

#ifdef SIMULATION
                            std::cout << "[C] at: " << previous_node << " to: " << next_node << " dir: " << controller.direction << std::endl;

#endif
                        }
                    }

                    prev_at_decision_point = at_decision_point;

                    switch (labyrinth_state)
                    {
                        case LabyrinthState::ERROR:
                        {
                            error_callback();
                            break;
                        }
                        case LabyrinthState::STANDBY:
                        {
                            standby_callback();
                            break;
                        }
                        case LabyrinthState::ESCAPE:
                        {
                            escape_callback();
                            break;
                        }
                        case LabyrinthState::MISSION_SWITCH:
                        {
                            mission_switch_callback();
                        }
                        default:
                        {
                            break;
                        }
                    }

                    if ((next_node == Edge::pirate_next_node ||
                         (next_node == Edge::pirate_after_next_node && Edge::pirate_section_percentage > 0.5f)) &&
                        labyrinth_state != LabyrinthState::REVERSE_ESCAPE)
                    {
                        if (labyrinth_state == LabyrinthState::EXPLORING || labyrinth_state == LabyrinthState::FINISHED)
                        {
                            prev_labyrinth_state = labyrinth_state;
                        }
                        labyrinth_state = LabyrinthState::REVERSE_ESCAPE;
                        previous_node   = next_node;
                        odometry.distance_local -= WHEELBASE;
                        reverse_escape_callback();

                        reference_speed   = -LABYRINTH_SPEED_REVERSE;
                        reverse_saved_dir = controller.direction;
                    }
                    else if (labyrinth_state == LabyrinthState::REVERSE_ESCAPE)
                    {
                        if (controller.target_speed < 0.0f && odometry.distance_local < WHEELBASE)
                        {
                            controller.set_direction(graph[at_node].edges[selected_edge].direction);
                        }

                        if (controller.target_speed < 0.0f && odometry.distance_local >= WHEELBASE)
                        {
                            controller.set_direction(reverse_saved_dir, true);
                        }

                        reference_speed = -LABYRINTH_SPEED_REVERSE;
                    }
                    else { reference_speed = LABYRINTH_SPEED; }

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

#ifdef SIMULATION
        std::chrono::time_point<std::chrono::steady_clock> prev_update_timestamp_ = std::chrono::steady_clock::now();
#endif
    };

}  // namespace jlb

#endif  // AS_STATE_HXX

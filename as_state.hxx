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
        Mission        mission         = Mission::STANDBY;
        LabyrinthState labyrinth_state = LabyrinthState::START;
        FastState      fast_state      = FastState::OUT_ACCEL_ZONE;
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
        Mission        mission              = Mission::STANDBY;
        LabyrinthState labyrinth_state      = LabyrinthState::START;
        LabyrinthState prev_labyrinth_state = LabyrinthState::START;
        FastState      fast_state           = FastState::OUT_ACCEL_ZONE;
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

        char          at_node       = START_GATE;
        char          previous_node = START_GATE;
        char          next_node     = START_GATE;
        char          goal_node     = START_GATE;
        unsigned long selected_edge = 0u;

        Direction reverse_saved_dir = Direction::STRAIGHT;

        bool  follow_car                = false;
        bool  flood                     = false;
        char  pirate_previous_node      = 'P';
        char  pirate_next_node          = 'M';
        char  pirate_after_next_node    = 'H';
        float pirate_section_percentage = 0.0f;

        bool pirate_intersecting(const char node_) { return node_ == pirate_next_node || node_ == pirate_after_next_node; }

        ASState(Odometry& odometry_, Controller& controller_, Graph& graph_) : odometry{odometry_}, controller{controller_}, graph{graph_} {}

        void set_states(const CompositeState state_)
        {
            mission              = state_.mission;
            prev_labyrinth_state = labyrinth_state;
            labyrinth_state      = state_.labyrinth_state;
            fast_state           = state_.fast_state;
        }

        void reset(const CompositeState state_)
        {
            set_states(state_);
            under_gate               = false;
            at_cross_section         = false;
            prev_at_decision_point   = false;
            current_number_of_lines  = 0u;
            state_time               = 0.0f;
            state_transition_time    = 0.0f;
            started_state_transition = false;
            tick_counter             = 0u;
            tick_counter_prev        = 0u;

            at_node       = START_GATE;
            previous_node = START_GATE;
            next_node     = START_GATE;
            goal_node     = START_GATE;
            selected_edge = 0u;

            reverse_saved_dir = Direction::STRAIGHT;

            follow_car                = false;
            flood                     = false;
            pirate_previous_node      = 'P';
            pirate_next_node          = 'M';
            pirate_after_next_node    = 'H';
            pirate_section_percentage = 0.0f;
        }

        void pirate_callback(const char prev_node_, const char next_node_, const char after_next_node_, const int section_percentage_)
        {
            pirate_previous_node      = prev_node_;
            pirate_next_node          = next_node_;
            pirate_after_next_node    = after_next_node_;
            pirate_section_percentage = section_percentage_;
            graph.pirate_callback(prev_node_, next_node_, after_next_node_, section_percentage_);
        }

        void apply_path(const DijkstraResult& result)
        {
            goal_node = result.node;
            next_node = result.path[1];

            for (unsigned long i = 0; i < graph[at_node].edges.size(); i++)
                if (graph[at_node].edges[i].to == next_node) selected_edge = i;

            controller.set_direction(graph[at_node].edges[selected_edge].direction);

            previous_node = at_node;

#ifdef SIMULATION
            std::cout << "[C] at: " << previous_node << " to: " << next_node << " dir: " << controller.direction << std::endl;
#endif
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

#ifdef SIMULATION
            std::cout << "[C] at: " << previous_node << " to: " << next_node << " dir: " << controller.direction << std::endl;
#endif
        }

        void exploring_callback()
        {
            std::cout << labyrinth_state << std::endl;
            if (flood)
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::FLOOD_TO_BALANCER;
                flood_to_balancer_callback();
                return;
            }

            if (std::find(std::begin(GATE_NAMES), std::end(GATE_NAMES), at_node) != std::end(GATE_NAMES) &&
                std::find(graph.collected_nodes.begin(), graph.collected_nodes.end(), at_node) == graph.collected_nodes.end() && !flood)
            {
                graph.collected_nodes.push_back(at_node);
            }

            auto result = graph.Dijkstra(previous_node, at_node);

            if (result.weight == std::numeric_limits<float>::infinity())
            {
                if (graph.collected_nodes.size() == NUMBER_OF_GATES)
                {
                    Edge::finished       = true;
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
            else if (result.weight >= WEIGHT_PENALTY * SAFETY_MARGIN && !pirate_intersecting(at_node))
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
            else if (result.weight >= WEIGHT_PENALTY * SAFETY_MARGIN && pirate_intersecting(at_node))
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::ESCAPE;
                escape_callback();
            }
            else { apply_path(result); }
        }

        void flood_to_balancer_callback()
        {
            std::cout << labyrinth_state << std::endl;

            if (at_node == BALANCER_START_NODE)
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::FLOOD_SOLVING;
                flood_solving_callback();
                return;
            }

            auto result = graph.Dijkstra(previous_node, at_node, BALANCER_START_NODE);

            if (result.weight == std::numeric_limits<float>::infinity())
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::ESCAPE;
                escape_callback();
            }
            else if (result.weight >= WEIGHT_PENALTY * SAFETY_MARGIN && !pirate_intersecting(at_node))
            {
                if (result.path.size() > 1)
                {
                    auto new_result = graph.Dijkstra(at_node, result.path[1], BALANCER_START_NODE);
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
            else if (result.weight >= WEIGHT_PENALTY * SAFETY_MARGIN && pirate_intersecting(at_node))
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::ESCAPE;
                escape_callback();
            }
            else { apply_path(result); }
        }

        void flood_solving_callback()
        {
            std::cout << labyrinth_state << std::endl;

            if (at_node == BALANCER_END_NODE)
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::FLOOD_TO_LABYRINTH;
                previous_node        = BALANCER_END_NODE;
                flood_to_labyrinth_callback();
                odometry.distance_local -= WHEELBASE;
                return;
            }

            auto result = graph.Dijkstra(previous_node, at_node, BALANCER_END_NODE);

            if (result.weight == std::numeric_limits<float>::infinity())
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::ERROR;
                error_callback();
            }
            else { apply_path(result); }
        }

        void flood_to_labyrinth_callback()
        {
            std::cout << labyrinth_state << std::endl;

            if (at_node == BALANCER_PREV_NODE)
            {
                auto result = graph.Dijkstra(previous_node, at_node, '@', true);

                if (result.weight == std::numeric_limits<float>::infinity())
                {
                    labyrinth_state = LabyrinthState::ERROR;
                    error_callback();
                    return;
                }
                else { previous_node = result.path[1]; }

                flood = false;
                odometry.distance_local += WHEELBASE;

                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::EXPLORING;
                exploring_callback();

                return;
            }

            auto result = graph.Dijkstra(previous_node, at_node, BALANCER_PREV_NODE);

            if (result.weight == std::numeric_limits<float>::infinity())
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::ERROR;
                error_callback();
            }
            else if (result.weight >= WEIGHT_PENALTY * SAFETY_MARGIN && at_node == BALANCER_START_NODE)
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::STANDBY;
                standby_callback();
            }
            else { apply_path_reverse(result); }
        }

        void finished_callback()
        {
            std::cout << labyrinth_state << std::endl;

            if (at_node == MISSION_SWITCH_NODE &&
                std::find(std::begin(MISSION_SWITCH_PREV_NODES), std::end(MISSION_SWITCH_PREV_NODES), previous_node) !=
                    std::end(MISSION_SWITCH_PREV_NODES))
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::MISSION_SWITCH;
                mission_switch_callback();
                return;
            }
            else if (at_node == MISSION_SWITCH_NODE)
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::ESCAPE;
                escape_callback();
                return;
            }

            auto result = graph.Dijkstra(previous_node, at_node, MISSION_SWITCH_NODE);

            if (result.weight == std::numeric_limits<float>::infinity())
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::ESCAPE;
                escape_callback();
            }
            else if (result.weight >= WEIGHT_PENALTY * SAFETY_MARGIN && !pirate_intersecting(at_node))
            {
                if (result.path.size() > 1)
                {
                    auto new_result = graph.Dijkstra(at_node, result.path[1], MISSION_SWITCH_NODE);
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
            else if (result.weight >= WEIGHT_PENALTY * SAFETY_MARGIN && pirate_intersecting(at_node))
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::ESCAPE;
                escape_callback();
            }
            else { apply_path(result); }
        }

        void error_callback() { std::cout << labyrinth_state << std::endl; }

        void standby_callback()
        {
            std::cout << labyrinth_state << std::endl;

            if (pirate_intersecting(at_node))
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
                    else if (result.path.size() > 1)
                    {
                        auto new_result = graph.Dijkstra(at_node, result.path[1], MISSION_SWITCH_NODE);
                        if (std::fabs(result.weight - new_result.weight) < WEIGHT_PENALTY * SAFETY_MARGIN)
                        {
                            prev_labyrinth_state = labyrinth_state;
                            labyrinth_state      = LabyrinthState::FINISHED;
                            finished_callback();
                        }
                    }
                }
                else if (prev_labyrinth_state == LabyrinthState::FLOOD_TO_BALANCER)
                {
                    auto result = graph.Dijkstra(previous_node, at_node, BALANCER_START_NODE);

                    if (result.weight == std::numeric_limits<float>::infinity())
                    {
                        labyrinth_state = LabyrinthState::ESCAPE;
                        escape_callback();
                        return;
                    }
                    else if (result.weight < WEIGHT_PENALTY * SAFETY_MARGIN)
                    {
                        prev_labyrinth_state = labyrinth_state;
                        labyrinth_state      = LabyrinthState::FLOOD_TO_BALANCER;
                        flood_to_balancer_callback();
                        return;
                    }
                    else if (result.path.size() > 1)
                    {
                        auto new_result = graph.Dijkstra(at_node, result.path[1], BALANCER_START_NODE);
                        if (std::fabs(result.weight - new_result.weight) < WEIGHT_PENALTY * SAFETY_MARGIN)
                        {
                            prev_labyrinth_state = labyrinth_state;
                            labyrinth_state      = LabyrinthState::FLOOD_TO_BALANCER;
                            flood_to_balancer_callback();
                        }
                    }
                }
                else if (prev_labyrinth_state == LabyrinthState::FLOOD_TO_LABYRINTH)
                {
                    auto result = graph.Dijkstra(previous_node, at_node, BALANCER_PREV_NODE);

                    if (result.weight == std::numeric_limits<float>::infinity())
                    {
                        labyrinth_state = LabyrinthState::ERROR;
                        error_callback();
                        return;
                    }
                    else if (result.weight < WEIGHT_PENALTY * SAFETY_MARGIN)
                    {
                        prev_labyrinth_state = labyrinth_state;
                        labyrinth_state      = LabyrinthState::FLOOD_TO_LABYRINTH;
                        flood_to_labyrinth_callback();
                        return;
                    }
                }
            }
        }

        void escape_callback()
        {
            std::cout << labyrinth_state << std::endl;

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
            std::cout << labyrinth_state << std::endl;

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
            std::cout << labyrinth_state << std::endl;

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
                    auto distance = graph[previous_node].edges[selected_edge].distance;

                    if (labyrinth_state == LabyrinthState::REVERSE_ESCAPE || labyrinth_state == LabyrinthState::FLOOD_TO_LABYRINTH)
                    {
                        distance = graph[at_node].edges[selected_edge].distance;
                        // if (-WHEELBASE < odometry.distance_local && odometry.distance_local < distance / 2.0f) { controller.set_passed_half(true);
                        // } else { controller.set_passed_half(false); }
                    }

                    if (odometry.distance_local > distance / 2.0f) { controller.set_passed_half(true); }
                    else { controller.set_passed_half(false); }

                    bool at_decision_point = under_gate || at_cross_section;

                    if ((!prev_at_decision_point && at_decision_point) || (labyrinth_state == LabyrinthState::REVERSE_ESCAPE && at_decision_point) ||
                        (labyrinth_state == LabyrinthState::FLOOD_TO_BALANCER && next_node == BALANCER_START_NODE) ||
                        (labyrinth_state == LabyrinthState::FLOOD_SOLVING && next_node == BALANCER_END_NODE) ||
                        (labyrinth_state == LabyrinthState::FLOOD_TO_LABYRINTH &&
                         (next_node == BALANCER_START_NODE || next_node == BALANCER_PREV_NODE)))
                    {
                        // std::cout << "target: " << distance << " actual: " << odometry.distance_local << std::endl;

                        bool  decide = false;
                        float delta  = distance - std::fabs(odometry.distance_local);
                        switch (labyrinth_state)
                        {
                            case LabyrinthState::START:
                            {
                                decide = true;
                                break;
                            }
                            case LabyrinthState::FLOOD_TO_BALANCER:
                            {
                                if (next_node == BALANCER_START_NODE && delta < 0.01f) { decide = true; }
                                else if (delta < LOCALIZATION_INACCURACY) { decide = true; }
                                break;
                            }
                            case LabyrinthState::FLOOD_SOLVING:
                            {
                                if (next_node == BALANCER_END_NODE && delta < 0.01f) { decide = true; }
                                break;
                            }
                            case LabyrinthState::FLOOD_TO_LABYRINTH:
                            {
                                if ((next_node == BALANCER_START_NODE || next_node == BALANCER_PREV_NODE) && delta < 0.01f) { decide = true; }
                                break;
                            }
                            case LabyrinthState::REVERSE_ESCAPE:
                            {
                                if (delta < LOCALIZATION_INACCURACY / 2.0f) { decide = true; }
                                break;
                            }
                            default:
                            {
                                if (std::fabs(delta) < LOCALIZATION_INACCURACY) { decide = true; }
                                break;
                            }
                        }

                        if (decide)
                        {
                            if (labyrinth_state == LabyrinthState::START)
                            {
                                prev_labyrinth_state    = labyrinth_state;
                                labyrinth_state         = LabyrinthState::EXPLORING;
                                odometry.distance_local = 0.0f;
                            }

                            at_node = next_node;

                            // std::cout << "prev: " << previous_node << " at: " << at_node << " next: " << next_node << " goal: " << goal_node << " "
                            //           << labyrinth_state << " flood: " << flood << std::endl;

                            odometry.correction(graph[at_node].x, graph[at_node].y);
                            odometry.reset_local();

                            // std::cout << "reset: " << labyrinth_state << " prev: " << prev_labyrinth_state << std::endl;

                            switch (labyrinth_state)
                            {
                                case LabyrinthState::EXPLORING:
                                {
                                    exploring_callback();
                                    break;
                                }
                                case LabyrinthState::FLOOD_TO_BALANCER:
                                {
                                    flood_to_balancer_callback();
                                    break;
                                }
                                case LabyrinthState::FLOOD_SOLVING:
                                {
                                    flood_solving_callback();
                                    break;
                                }
                                case LabyrinthState::FLOOD_TO_LABYRINTH:
                                {
                                    flood_to_labyrinth_callback();
                                    break;
                                }
                                case LabyrinthState::FINISHED:
                                {
                                    finished_callback();
                                    break;
                                }
                                case LabyrinthState::REVERSE_ESCAPE:
                                {
                                    if (next_node == pirate_next_node || (next_node == pirate_after_next_node && pirate_section_percentage > 0.5f) ||
                                        (next_node == pirate_previous_node && previous_node == pirate_next_node))
                                    {
                                        labyrinth_state = LabyrinthState::REVERSE_ESCAPE;
                                        reverse_escape_callback();

                                        reference_speed   = -LABYRINTH_SPEED_REVERSE;
                                        reverse_saved_dir = controller.direction;
                                        break;
                                    }

                                    auto result = graph.Dijkstra(previous_node, at_node, '@', true);

                                    if (result.weight == std::numeric_limits<float>::infinity())
                                    {
                                        labyrinth_state = LabyrinthState::ERROR;
                                        error_callback();
                                        break;
                                    }
                                    else { previous_node = result.path[1]; }

                                    odometry.distance_local += WHEELBASE;

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
                                    else if (prev_labyrinth_state == LabyrinthState::FLOOD_TO_BALANCER)
                                    {
                                        prev_labyrinth_state = labyrinth_state;
                                        labyrinth_state      = LabyrinthState::FLOOD_TO_BALANCER;
                                        flood_to_balancer_callback();
                                    }

                                    break;
                                }
                                default:
                                {
                                    break;
                                }
                            }
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

                    if ((next_node == pirate_next_node || (next_node == pirate_after_next_node && pirate_section_percentage > 0.5f) ||
                         (next_node == pirate_previous_node && previous_node == pirate_next_node)) &&
                        labyrinth_state != LabyrinthState::REVERSE_ESCAPE && labyrinth_state != LabyrinthState::FLOOD_TO_LABYRINTH)
                    {
                        if (labyrinth_state == LabyrinthState::EXPLORING || labyrinth_state == LabyrinthState::FINISHED ||
                            labyrinth_state == LabyrinthState::FLOOD_TO_BALANCER)
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
                    else if (labyrinth_state == LabyrinthState::REVERSE_ESCAPE || labyrinth_state == LabyrinthState::FLOOD_TO_LABYRINTH)
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
                    else if (labyrinth_state == LabyrinthState::EXPLORING || labyrinth_state == LabyrinthState::FINISHED ||
                             labyrinth_state == LabyrinthState::ESCAPE || labyrinth_state == LabyrinthState::FLOOD_TO_BALANCER ||
                             labyrinth_state == LabyrinthState::START)
                    {
                        reference_speed = LABYRINTH_SPEED;
                    }
                    else if (labyrinth_state == LabyrinthState::MISSION_SWITCH) { reference_speed = MISSION_SWITCH_SPEED; }
                    else if (labyrinth_state == LabyrinthState::FLOOD_SOLVING) { reference_speed = BALANCER_SPEED; }

                    if ((next_node == pirate_previous_node || next_node == pirate_next_node) && labyrinth_state != LabyrinthState::REVERSE_ESCAPE &&
                        labyrinth_state != LabyrinthState::FLOOD_TO_LABYRINTH)
                    {
                        follow_car = true;
                    }
                    else { follow_car = false; }

                    break;
                }
                case Mission::FAST:
                {
                    follow_car = true;

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
                            // this should never happen== pirate_next_node
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

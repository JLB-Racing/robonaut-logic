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
        Mission            mission              = Mission::STANDBY;
        LabyrinthState     labyrinth_state      = LabyrinthState::START;
        MissionSwitchState mission_switch_state = MissionSwitchState::STANDBY;

#ifdef FAST_V0
        FastState fast_state = FastState::OUT_ACCEL_ZONE;
#else
        FastState fast_state = FastState::FIRST_FAST;
#endif

        float target_speed = 0.0f;

        CompositeState(FastState fast_state_)
            : mission{Mission::FAST},
              labyrinth_state{LabyrinthState::START},
              mission_switch_state{MissionSwitchState::STANDBY},
              fast_state{fast_state_},
              target_speed{0.0f}
        {
        }
        CompositeState(LabyrinthState labyrinth_state_)
            : mission{Mission::LABYRINTH},
              labyrinth_state{labyrinth_state_},
              mission_switch_state{MissionSwitchState::STANDBY},
              fast_state{FastState::FIRST_FAST},
              target_speed{0.0f}
        {
        }
        CompositeState(
            Mission mission_, LabyrinthState labyrinth_state_, MissionSwitchState mission_switch_state_, FastState fast_state_, float target_speed_)
            : mission{mission_},
              labyrinth_state{labyrinth_state_},
              mission_switch_state{mission_switch_state_},
              fast_state{fast_state_},
              target_speed{target_speed_}
        {
        }
    };

    class ASState
    {
    public:
        Mission            prev_mission         = Mission::STANDBY;
        Mission            mission              = Mission::STANDBY;
        LabyrinthState     labyrinth_state      = LabyrinthState::START;
        LabyrinthState     prev_labyrinth_state = LabyrinthState::START;
        MissionSwitchState mission_switch_state = MissionSwitchState::STANDBY;
#ifdef FAST_V0
        FastState fast_state = FastState::OUT_ACCEL_ZONE;
#else
        FastState fast_state = FastState::FIRST_FAST;
#endif
        float target_speed = 0.0f;

        bool     under_gate               = false;
        bool     at_cross_section         = false;
        bool     prev_at_decision_point   = false;
        uint8_t  num_lines                = 0u;
        float    state_time               = 0.0f;
        float    state_transition_time    = 0.0f;
        bool     started_state_transition = false;
        uint32_t tick_counter             = 0u;
        uint32_t tick_counter_prev        = 0u;

        char at_node       = START_GATE;
        char previous_node = START_GATE;
#ifndef SIMULATION
        char next_node = START_NEXT_GATE;
        char goal_node = START_NEXT_GATE;
#else
        char next_node = START_GATE;
        char goal_node = START_GATE;
#endif
        unsigned long selected_edge   = 0u;
        float         target_distance = 0.0f;

        Direction reverse_saved_dir = Direction::STRAIGHT;

        bool  fallback                  = false;
        bool  follow_car                = false;
        bool  flood                     = false;
        char  pirate_previous_node      = '@';
        char  pirate_next_node          = '@';
        char  pirate_after_next_node    = '@';
        float pirate_section_percentage = 0.0f;

        float mission_switch_steering_angle = deg2rad(MAX_WHEEL_ANGLE) * 2.0f;
        float mission_switch_arc_length     = 0.0f;

        uint8_t collected_valid_gates = 0;

        float last_laptime;
        float best_laptime;
        float current_laptime;

        bool    safety_car              = true;
        float   safety_car_time         = 0.0f;
        uint8_t completed_laps          = 0;
        uint8_t completed_overtakes     = 0;
        bool    overtake_started        = false;
        float   overtake_time           = 0.0f;
        float   overtake_steering_angle = 0.0f;

        bool pirate_intersecting(const char node_) { return node_ == pirate_next_node || node_ == pirate_after_next_node; }

        ASState(Odometry& odometry_, Controller& controller_, Graph& graph_) : odometry{odometry_}, controller{controller_}, graph{graph_} {}

        void set_states(const CompositeState state_)
        {
            mission              = state_.mission;
            prev_labyrinth_state = labyrinth_state;
            labyrinth_state      = state_.labyrinth_state;
            mission_switch_state = state_.mission_switch_state;
            fast_state           = state_.fast_state;
        }

        void reset(const CompositeState state_)
        {
            set_states(state_);
            under_gate               = false;
            at_cross_section         = false;
            prev_at_decision_point   = false;
            num_lines                = 0u;
            state_time               = 0.0f;
            state_transition_time    = 0.0f;
            started_state_transition = false;
            tick_counter             = 0u;
            tick_counter_prev        = 0u;

            at_node       = START_GATE;
            previous_node = START_GATE;
#ifndef SIMULATION
            next_node = START_NEXT_GATE;
            goal_node = START_NEXT_GATE;
#else
            next_node = START_GATE;
            goal_node = START_GATE;
#endif
            selected_edge = 0u;

            reverse_saved_dir = Direction::STRAIGHT;

            follow_car                = false;
            flood                     = false;
            pirate_previous_node      = '@';
            pirate_next_node          = '@';
            pirate_after_next_node    = '@';
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
                if (Edge::stolen_gates[static_cast<int>(at_node - 'A')] < 2) { collected_valid_gates++; }
                graph.collected_nodes.push_back(at_node);

#ifdef Q2
                if (collected_valid_gates >= 10 && !flood)
                {
                    Edge::finished       = true;
                    prev_labyrinth_state = labyrinth_state;
                    labyrinth_state      = LabyrinthState::FINISHED;
                    finished_callback();
                    return;
                }
#endif
            }

            auto result = graph.Dijkstra(previous_node, at_node);

            if (result.weight == std::numeric_limits<float>::infinity())
            {
                if (graph.collected_nodes.size() == NUMBER_OF_GATES && !flood)
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
            target_speed = BALANCER_SPEED;

            if (at_node == BALANCER_END_NODE && !flood)
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::FLOOD_TO_LABYRINTH;
                previous_node        = BALANCER_END_NODE;
                flood_to_labyrinth_callback();
                odometry.distance_local -= WHEELBASE;
                return;
            }
            else if (at_node == BALANCER_END_NODE && flood)
            {
                target_speed = 0.0f;
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
            if (flood)
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::FLOOD_TO_BALANCER;
                flood_to_balancer_callback();
                return;
            }

            if (at_node == MISSION_SWITCH_NODE &&
                std::find(std::begin(MISSION_SWITCH_PREV_NODES), std::end(MISSION_SWITCH_PREV_NODES), previous_node) !=
                    std::end(MISSION_SWITCH_PREV_NODES))
            {
                prev_labyrinth_state = labyrinth_state;
                labyrinth_state      = LabyrinthState::MISSION_SWITCH;

                previous_node = MISSION_SWITCH_NODE;
                next_node     = MISSION_SWITCH_NEXT_NODE;

                float h             = MISSION_SWITCH_LATERAL_DIST / 2.0f;
                float R             = WHEELBASE / std::tan(deg2rad(MISSION_SWITCH_STEERING_ANGLE));
                float alpha_per_two = std::acos(1.0f - (h / R));
                float alpha         = 2.0f * alpha_per_two;

                mission_switch_steering_angle = deg2rad(MISSION_SWITCH_STEERING_ANGLE);
                mission_switch_arc_length     = alpha * R;

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

        void error_callback() {}

        void standby_callback()
        {
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
            auto result = graph.Dijkstra(previous_node, at_node, '@', true, true);

            if (result.weight == std::numeric_limits<float>::infinity())
            {
                labyrinth_state = LabyrinthState::ERROR;
                error_callback();
            }
            else { apply_path_reverse(result); }
        }

        void mission_switch_callback()
        {
            switch (mission_switch_state)
            {
                case MissionSwitchState::STANDBY:
                {
                    if (std::fabs(odometry.vx_t) < 0.05f)
                    {
                        odometry.reset_local(true);
                        mission_switch_state = MissionSwitchState::FIRST_FORWARD;
                        break;
                    }

                    follow_car = false;
                    break;
                }
                case MissionSwitchState::FIRST_FORWARD:
                {
                    if (std::fabs(MISSION_SWITCH_FIRST_FORWARD_DIST - odometry.distance_local) < 0.01f)
                    {
                        odometry.reset_local(true);
                        mission_switch_state = MissionSwitchState::FIRST_TURN;
                        break;
                    }

                    follow_car = false;
                    break;
                }
                case MissionSwitchState::FIRST_TURN:
                {
                    if (std::fabs((mission_switch_arc_length / 2.0f) - odometry.distance_local) < 0.01f)
                    {
                        odometry.reset_local(true);
                        mission_switch_state = MissionSwitchState::SECOND_TURN;
                        break;
                    }

                    follow_car = true;
                    break;
                }
                case MissionSwitchState::SECOND_TURN:
                {
                    if (std::fabs((mission_switch_arc_length / 2.0f) - odometry.distance_local) < 0.01f)
                    {
                        odometry.reset_local(true);
                        mission_switch_state = MissionSwitchState::SECOND_FORWARD;
                        break;
                    }

                    follow_car = true;
                    break;
                }
                case MissionSwitchState::SECOND_FORWARD:
                {
                    if (std::fabs(MISSION_SWITCH_SECOND_FORWARD_DIST - odometry.distance_local) < 0.01f)
                    {
                        odometry.reset_local(true);
                        mission_switch_state = MissionSwitchState::STANDBY;
                        prev_mission         = mission;
#ifndef Q2
                        mission = Mission::FAST;
#else
                        mission = Mission::STANDBY;
#endif
                        target_distance = FIRST_FAST_DIST;
                        break;
                    }
                }
                default:
                {
                    break;
                }
            }
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
                    target_distance = graph[previous_node].edges[selected_edge].distance;

                    if (labyrinth_state == LabyrinthState::REVERSE_ESCAPE || labyrinth_state == LabyrinthState::FLOOD_TO_LABYRINTH)
                    {
                        target_distance = graph[at_node].edges[selected_edge].distance;
                    }

                    float delta = target_distance - std::fabs(odometry.distance_local);

                    if (odometry.distance_local > target_distance / 2.0f) { controller.set_passed_half(true); }
                    else { controller.set_passed_half(false); }

                    bool at_decision_point = under_gate || at_cross_section;

#ifndef SIMULATION
                    if ((!prev_at_decision_point && at_decision_point) || delta < LOCALIZATION_FALLBACK ||
                        (labyrinth_state == LabyrinthState::REVERSE_ESCAPE && at_decision_point) ||
                        (labyrinth_state == LabyrinthState::FLOOD_TO_BALANCER && next_node == BALANCER_START_NODE) ||
                        (labyrinth_state == LabyrinthState::FLOOD_SOLVING && next_node == BALANCER_END_NODE) ||
                        (labyrinth_state == LabyrinthState::FLOOD_TO_LABYRINTH &&
                         (next_node == BALANCER_START_NODE || next_node == BALANCER_PREV_NODE)))
#else
                    if ((!prev_at_decision_point && at_decision_point) || (labyrinth_state == LabyrinthState::REVERSE_ESCAPE && at_decision_point) ||
                        (labyrinth_state == LabyrinthState::FLOOD_TO_BALANCER && next_node == BALANCER_START_NODE) ||
                        (labyrinth_state == LabyrinthState::FLOOD_SOLVING && next_node == BALANCER_END_NODE) ||
                        (labyrinth_state == LabyrinthState::FLOOD_TO_LABYRINTH &&
                         (next_node == BALANCER_START_NODE || next_node == BALANCER_PREV_NODE)))
#endif
                    {
                        // std::cout << "target: " << distance << " actual: " << odometry.distance_local << std::endl;

                        bool decide = false;
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

                            if (fallback)
                            {
                                odometry.distance_local += -1.0f * LOCALIZATION_FALLBACK;
                                odometry.x_t_local += -1.0f * LOCALIZATION_FALLBACK;
                                fallback = false;
                            }

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

                                        target_speed      = -LABYRINTH_SPEED_REVERSE;
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
                            break;
                        }
                        case LabyrinthState::FLOOD_SOLVING:
                        {
                            if (at_node == BALANCER_END_NODE && !flood)
                            {
                                prev_labyrinth_state = labyrinth_state;
                                labyrinth_state      = LabyrinthState::FLOOD_TO_LABYRINTH;
                                previous_node        = BALANCER_END_NODE;
                                flood_to_labyrinth_callback();
                                odometry.distance_local -= WHEELBASE;
                            }
                            break;
                        }
                        default:
                        {
                            break;
                        }
                    }

                    if ((next_node == pirate_next_node || (next_node == pirate_after_next_node && pirate_section_percentage > 0.5f) ||
                         (next_node == pirate_previous_node && previous_node == pirate_next_node)) &&
                        labyrinth_state != LabyrinthState::REVERSE_ESCAPE && labyrinth_state != LabyrinthState::FLOOD_TO_LABYRINTH &&
                        mission_switch_state != MissionSwitchState::FIRST_TURN && mission_switch_state != MissionSwitchState::SECOND_TURN &&
                        mission_switch_state != MissionSwitchState::SECOND_FORWARD)
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

                        target_speed      = -LABYRINTH_SPEED_REVERSE;
                        reverse_saved_dir = controller.direction;
                    }

                    if (labyrinth_state == LabyrinthState::REVERSE_ESCAPE)
                    {
                        if (controller.target_speed < 0.0f && odometry.vx_t < 0.0f && odometry.distance_local < WHEELBASE)
                        {
                            controller.set_direction(graph[at_node].edges[selected_edge].direction);
                        }
                        if (controller.target_speed < 0.0f && odometry.vx_t < 0.0f && odometry.distance_local >= WHEELBASE)
                        {
                            controller.set_direction(reverse_saved_dir, true);
                        }
                        target_speed = -LABYRINTH_SPEED_REVERSE;
                    }
                    else if (labyrinth_state == LabyrinthState::FLOOD_TO_LABYRINTH)
                    {
                        if (controller.target_speed < 0.0f && odometry.vx_t < 0.0f && odometry.distance_local < WHEELBASE)
                        {
                            controller.set_direction(graph[at_node].edges[selected_edge].direction);
                        }
                        if (controller.target_speed < 0.0f && odometry.vx_t < 0.0f && odometry.distance_local >= WHEELBASE)
                        {
                            controller.set_direction(reverse_saved_dir, true);
                        }
                        if (next_node == BALANCER_START_NODE) { target_speed = -BALANCER_SPEED; }
                        else { target_speed = -LABYRINTH_SPEED_REVERSE; }
                    }
                    else if (labyrinth_state == LabyrinthState::EXPLORING || labyrinth_state == LabyrinthState::FINISHED ||
                             labyrinth_state == LabyrinthState::ESCAPE || labyrinth_state == LabyrinthState::FLOOD_TO_BALANCER ||
                             labyrinth_state == LabyrinthState::START)
                    {
                        if (graph[at_node].edges[selected_edge].fast && target_speed == LABYRINTH_SPEED_FAST) { target_speed = LABYRINTH_SPEED_FAST; }
                        else if (graph[at_node].edges[selected_edge].fast && target_speed == LABYRINTH_SPEED && odometry.distance_local > 0.2f)
                        {
                            target_speed = LABYRINTH_SPEED_FAST;
                        }
                        else { target_speed = LABYRINTH_SPEED; }
                    }
                    else if (labyrinth_state == LabyrinthState::MISSION_SWITCH) { target_speed = MISSION_SWITCH_SPEED; }
                    else if (labyrinth_state == LabyrinthState::FLOOD_SOLVING)
                    { /*moved to flood_solving callback*/
                    }

                    if ((next_node == pirate_previous_node || next_node == pirate_next_node) && labyrinth_state != LabyrinthState::REVERSE_ESCAPE &&
                        labyrinth_state != LabyrinthState::FLOOD_TO_LABYRINTH)
                    {
                        follow_car = true;
                    }
                    else if (labyrinth_state != LabyrinthState::MISSION_SWITCH) { follow_car = false; }

                    break;
                }
                case Mission::FAST:
                {
#ifndef FAST_V0

                    controller.set_direction(Direction::STRAIGHT);
                    float delta               = target_distance - std::fabs(odometry.distance_local);
                    float safety_car_distance = controller.object_range;

                    if (safety_car_distance < SAFETY_CAR_THRESHOLD)
                    {
                        // IF IN THRESHOLD, RESET TIMEOUT
                        safety_car_time = 0.0;
                    }
                    else
                    {
                        // IF OUT OF THRESHOLD, INCREMENT TIMEOUT
                        safety_car_time += dt;
                    }

                    if (safety_car_time >= SAFETY_CAR_TIMEOUT) { safety_car = false; }
                    else { safety_car = true; }

                    if (completed_laps == 0u && (fast_state == FastState::FIRST_FAST || fast_state == FastState::FIRST_SLOW ||
                                                 fast_state == FastState::SECOND_FAST || fast_state == FastState::SECOND_SLOW))
                    {
                        safety_car = true;
                    }

                    std::cout << "completed_laps: " << static_cast<int>(completed_laps)
                              << " completed_overtakes: " << static_cast<int>(completed_overtakes) << std::endl;

                    switch (fast_state)
                    {
                        case FastState::FIRST_FAST:
                        {
                            target_distance = FIRST_FAST_DIST;
                            if (prev_mission == Mission::LABYRINTH || safety_car) { target_speed = FAST_SPEED_SAFETY_CAR; }
                            else { target_speed = FAST_SPEED[completed_laps]; }

                            if (num_lines >= 3 && (delta < LOCALIZATION_INACCURACY || prev_mission == Mission::LABYRINTH))
                            {
                                if (prev_mission == Mission::LABYRINTH) { prev_mission = Mission::FAST; }
                                fast_state = FastState::FIRST_SLOW;
                                odometry.reset_local(true);
                            }

                            break;
                        }
                        case FastState::FIRST_SLOW:
                        {
                            target_distance = FIRST_SLOW_DIST;
                            if (completed_laps == 6u && completed_overtakes == 2u) { target_speed = 0.0f; }
                            else if (completed_laps == 5u && completed_overtakes < 2u) { target_speed = 0.0f; }
                            else if (safety_car) { target_speed = FAST_SPEED_SAFETY_CAR; }
                            else { target_speed = FAST_SPEED_TURN[completed_laps]; }

                            if (num_lines >= 3 && delta < LOCALIZATION_INACCURACY)
                            {
                                fast_state = FastState::SECOND_FAST;
                                odometry.reset_local(true);
                            }

                            break;
                        }
                        case FastState::SECOND_FAST:
                        {
                            target_distance = SECOND_FAST_DIST;
                            if (safety_car) { target_speed = FAST_SPEED_SAFETY_CAR; }
                            else { target_speed = FAST_SPEED[completed_laps]; }

                            if (num_lines >= 3 && delta < LOCALIZATION_INACCURACY)
                            {
                                fast_state = FastState::SECOND_SLOW;
                                odometry.reset_local(true);
                            }

                            break;
                        }
                        case FastState::SECOND_SLOW:
                        {
                            target_distance = SECOND_SLOW_DIST;
                            if (safety_car) { target_speed = FAST_SPEED_SAFETY_CAR; }
                            else { target_speed = FAST_SPEED_TURN[completed_laps]; }

                            if (num_lines >= 3 && delta < LOCALIZATION_INACCURACY)
                            {
                                fast_state = FastState::THIRD_FAST;
                                odometry.reset_local(true);
                            }

                            break;
                        }
                        case FastState::THIRD_FAST:
                        {
                            target_distance = THIRD_FAST_DIST;
                            if (safety_car) { target_speed = FAST_SPEED_SAFETY_CAR; }
                            else { target_speed = FAST_SPEED[completed_laps]; }

#ifdef OVERTAKE
                            if ((safety_car && (completed_laps == 0u || completed_laps == 2u) &&
                                 safety_car_distance < obj::FOLLOW_DISTANCE * 1.25f) ||
                                overtake_started)
                            {
                                if (!overtake_started)
                                {
                                    overtake_started = true;
                                    overtake_time    = 0.0f;
                                }
                                else
                                {
                                    safety_car      = false;
                                    safety_car_time = SAFETY_CAR_TIMEOUT;
                                }

                                if (overtake_time < OVERTAKE_FIRST_FORWARD_TIME)
                                {
                                    // GO FORWARD
                                    overtake_steering_angle = 0.0f;
                                    target_speed            = FAST_SPEED_SAFETY_CAR;
                                }
                                else if (overtake_time < OVERTAKE_FIRST_FORWARD_TIME + OVERTAKE_FIRST_LEFT_TIME)
                                {
                                    // TURN LEFT
                                    overtake_steering_angle = deg2rad(-OVERTAKE_STEERING_ANGLE);
                                    target_speed            = FAST_SPEED_OVERTAKE_TURN;
                                }
                                else if (overtake_time < OVERTAKE_FIRST_FORWARD_TIME + OVERTAKE_FIRST_LEFT_TIME + OVERTAKE_FIRST_RIGHT_TIME)
                                {
                                    // TURN RIGHT
                                    overtake_steering_angle = deg2rad(OVERTAKE_STEERING_ANGLE);
                                    target_speed            = FAST_SPEED_OVERTAKE_TURN;
                                }
                                else if (overtake_time < OVERTAKE_FIRST_FORWARD_TIME + OVERTAKE_FIRST_LEFT_TIME + OVERTAKE_FIRST_RIGHT_TIME +
                                                             OVERTAKE_SECOND_FORWARD_TIME)
                                {
                                    // GO FORWARD
                                    overtake_steering_angle = 0.0f;
                                    target_speed            = FAST_SPEED_OVERTAKE;
                                }
                                else if (overtake_time < OVERTAKE_FIRST_FORWARD_TIME + OVERTAKE_FIRST_LEFT_TIME + OVERTAKE_FIRST_RIGHT_TIME +
                                                             OVERTAKE_SECOND_FORWARD_TIME + OVERTAKE_SECOND_RIGHT_TIME)
                                {
                                    // TURN RIGHT
                                    overtake_steering_angle = deg2rad(OVERTAKE_STEERING_ANGLE);
                                    target_speed            = FAST_SPEED_OVERTAKE_TURN;
                                }
                                else if (overtake_time < OVERTAKE_FIRST_FORWARD_TIME + OVERTAKE_FIRST_LEFT_TIME + OVERTAKE_FIRST_RIGHT_TIME +
                                                             OVERTAKE_SECOND_FORWARD_TIME + OVERTAKE_SECOND_RIGHT_TIME + OVERTAKE_SECOND_LEFT_TIME)
                                {
                                    // TURN LEFT
                                    overtake_steering_angle = deg2rad(-OVERTAKE_STEERING_ANGLE);
                                    target_speed            = FAST_SPEED_OVERTAKE_TURN;
                                }
                                else
                                {
                                    safety_car       = false;
                                    safety_car_time  = SAFETY_CAR_TIMEOUT;
                                    overtake_started = false;
                                    completed_overtakes++;
                                }
                                overtake_time += dt;
                            }
#endif

                            if (num_lines >= 3 &&
                                (delta < LOCALIZATION_INACCURACY || overtake_time > OVERTAKE_FIRST_FORWARD_TIME + OVERTAKE_FIRST_LEFT_TIME +
                                                                                        OVERTAKE_FIRST_RIGHT_TIME + OVERTAKE_SECOND_FORWARD_TIME +
                                                                                        OVERTAKE_SECOND_RIGHT_TIME + OVERTAKE_SECOND_LEFT_TIME))
                            {
                                fast_state = FastState::THIRD_SLOW;
                                odometry.reset_local(true);
                                overtake_time = 0.0f;
                            }

                            break;
                        }
                        case FastState::THIRD_SLOW:
                        {
                            target_distance = THIRD_SLOW_DIST;
                            if (safety_car) { target_speed = FAST_SPEED_SAFETY_CAR; }
                            else { target_speed = FAST_SPEED_TURN[completed_laps]; }

                            if (num_lines >= 3 && delta < LOCALIZATION_INACCURACY)
                            {
                                fast_state = FastState::FOURTH_FAST;
                                odometry.reset_local(true);
                            }

                            break;
                        }
                        case FastState::FOURTH_FAST:
                        {
                            target_distance = FOURTH_FAST_DIST;
                            if (safety_car) { target_speed = FAST_SPEED_SAFETY_CAR; }
                            else { target_speed = FAST_SPEED[completed_laps]; }

                            if (num_lines >= 3 && delta < LOCALIZATION_INACCURACY)
                            {
                                fast_state = FastState::FOURTH_SLOW;
                                odometry.reset_local(true);
                            }

                            break;
                        }
                        case FastState::FOURTH_SLOW:
                        {
                            target_distance = FOURTH_SLOW_DIST;
                            if (safety_car) { target_speed = FAST_SPEED_SAFETY_CAR; }
                            else { target_speed = FAST_SPEED_TURN[completed_laps]; }

                            if (num_lines >= 3 && delta < LOCALIZATION_INACCURACY)
                            {
                                fast_state = FastState::FIRST_FAST;
                                odometry.reset_local(true);
                                completed_laps++;
                            }

                            break;
                        }
                        default:
                        {
                            break;
                        }
                    }

#else
                    follow_car = true;

                    switch (fast_state)
                    {
                        case FastState::FOLLOW_SAFETY_CAR:
                        {
                            target_speed = FAST_SPEED_SAFETY_CAR;
                            break;
                        }
                        case FastState::OVERTAKE_SAFETY_CAR_START:
                        {
                            target_speed = FAST_SPEED_OVERTAKE;
                            break;
                        }
                        case FastState::OVERTAKE_SAFETY_CAR_END:
                        {
                            target_speed = FAST_SPEED;
                            break;
                        }
                        case FastState::IN_ACCEL_ZONE:
                        {
                            if (num_lines == 1u && !started_state_transition)
                            {
                                started_state_transition = true;
                                state_transition_time    = 0.0f;
                            }
                            else if (num_lines != 1u && started_state_transition) { started_state_transition = false; }

                            if (started_state_transition && state_transition_time > STATE_TRANSITION_TIME_LIMIT && state_time > STATE_MIN_TIME)
                            {
                                fast_state = FastState::OUT_ACCEL_ZONE;
                                state_time = 0.0f;
                                break;
                            }

                            target_speed = FAST_SPEED;
                            break;
                        }
                        case FastState::OUT_ACCEL_ZONE:
                        {
                            if (num_lines == 3u && !started_state_transition)
                            {
                                started_state_transition = true;
                                state_transition_time    = 0.0f;
                            }
                            else if (num_lines != 3u && started_state_transition) { started_state_transition = false; }

                            if (started_state_transition && state_transition_time > STATE_TRANSITION_TIME_LIMIT && state_time > STATE_MIN_TIME)
                            {
                                fast_state = FastState::IN_BRAKE_ZONE;
                                state_time = 0.0f;
                                break;
                            }

                            target_speed = FAST_SPEED;
                            break;
                        }
                        case FastState::IN_BRAKE_ZONE:
                        {
                            if (num_lines == 1u && !started_state_transition)
                            {
                                started_state_transition = true;
                                state_transition_time    = 0.0f;
                            }
                            else if (num_lines != 1u && started_state_transition) { started_state_transition = false; }

                            if (started_state_transition && state_transition_time > STATE_TRANSITION_TIME_LIMIT && state_time > STATE_MIN_TIME)
                            {
                                fast_state = FastState::OUT_BRAKE_ZONE;
                                state_time = 0.0f;
                                break;
                            }

                            target_speed = FAST_SPEED_TURN;
                            break;
                        }
                        case FastState::OUT_BRAKE_ZONE:
                        {
                            if (num_lines == 3u && !started_state_transition)
                            {
                                started_state_transition = true;
                                state_transition_time    = 0.0f;
                            }
                            else if (num_lines != 3u && started_state_transition) { started_state_transition = false; }

                            if (started_state_transition && state_transition_time > STATE_TRANSITION_TIME_LIMIT && state_time > STATE_MIN_TIME)
                            {
                                fast_state = FastState::IN_ACCEL_ZONE;
                                state_time = 0.0f;
                                break;
                            }

                            target_speed = FAST_SPEED_TURN;
                            break;
                        }
                        default:
                        {
                            // this should never happen== pirate_next_node
                            break;
                        }
                    }

                    break;
#endif
                }
                default:
                {
                    // this should never happen
                    break;
                }
            }

            return CompositeState{mission, labyrinth_state, mission_switch_state, fast_state, target_speed};
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

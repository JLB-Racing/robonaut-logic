#ifndef LOGIC_HXX
#define LOGIC_HXX

#include "vehicle_model.hxx"
#include "environment.hxx"

#include <chrono>
#include "SGL/sgl.hxx"

#include "odometry.hxx"
#include "common.hxx"

namespace jlb
{
    enum class Direction
    {
        LEFT,
        RIGHT,
        STRAIGHT,
        REVERSE_LEFT,
        REVERSE_RIGHT,
        REVERSE_STRAIGHT
    };

    enum class Mission
    {
        LABYRINTH,
        FAST,
        FAST_TURN,
        FAST_OVERTAKE
    };

    class Controller
    {
    public:
        unsigned long selected = 0;
        float target_angle = 0.0f;
        float target_speed = 0.0f;

        Direction direction = Direction::STRAIGHT;
        Mission mission = Mission::LABYRINTH;

        Controller() {}
        ~Controller() {}

        float PID(const float error, const float dt)
        {
            float proportional_term = Kp * error;
            integral += Ki * error * dt;
            float derivative_term = Kd * (error - prev_error) / dt;
            return proportional_term + integral + derivative_term;
        }

        float stanley(const float cross_track_error, const float heading_error)
        {
            float kAng = 0.5;
            float kDist = 0.5;
            float kSoft = 1.0;
            float kDamp = 1.0;

            return kAng * heading_error + atan2(kDist * cross_track_error, kSoft + kDamp * current_velocity);
        }

        template <size_t cols>
        void lateral_control(bool (&detection_)[cols])
        {
            if (std::all_of(std::begin(detection_), std::end(detection_), [](bool b)
                            { return b; }))
            {
                return;
            }

#ifdef STM32
            // TODO: add timestamp
#else
            auto control_timestamp_ = std::chrono::steady_clock::now();
            [[maybe_unused]] float dt = std::chrono::duration_cast<std::chrono::milliseconds>(control_timestamp_ - prev_control_timestamp_).count() / 1000.0f;
            prev_control_timestamp_ = control_timestamp_;
#endif

            unsigned long sensor_center = cols / 2;

            unsigned long rightmost = 0;
            for (unsigned long i = 0; i < cols; i++)
                if (!detection_[i] && i > rightmost)
                    rightmost = i;

            unsigned long leftmost = cols;
            for (unsigned long i = 0; i < cols; i++)
                if (!detection_[i] && i < leftmost)
                    leftmost = i;

            unsigned long center = leftmost;
            for (unsigned long i = leftmost; i <= rightmost; i++)
                if (!detection_[i] && std::abs(static_cast<int>(i - (rightmost + leftmost) / 2)) < std::abs(static_cast<int>(center - (rightmost + leftmost) / 2)))
                    center = i;

            if (direction == Direction::LEFT || direction == Direction::REVERSE_LEFT)
                selected = leftmost;
            if (direction == Direction::RIGHT || direction == Direction::REVERSE_RIGHT)
                selected = rightmost;
            if (direction == Direction::STRAIGHT || direction == Direction::REVERSE_STRAIGHT)
                selected = center;

            [[maybe_unused]] float angle_error = (static_cast<int>(selected - sensor_center + 1)) / static_cast<float>(sensor_center) * M_PI / 2.0f;
            float error = (static_cast<int>(selected - sensor_center + 1)) / static_cast<float>(sensor_center);
            target_angle = PID(error, dt);

            if (direction == Direction::REVERSE_LEFT || direction == Direction::REVERSE_RIGHT || direction == Direction::REVERSE_STRAIGHT)
            {
                target_angle = -target_angle;
            }

            prev_error = error;
        }

        void longitudinal_control()
        {
            switch (mission)
            {
            case Mission::LABYRINTH:
            {
                target_speed = LABYRINTH_SPEED;

                if (direction == Direction::REVERSE_LEFT || direction == Direction::REVERSE_RIGHT || direction == Direction::REVERSE_STRAIGHT)
                {
                    target_speed = -target_speed;
                }
            }
            break;

            case Mission::FAST:
                target_speed = FAST_SPEED;
                break;

            case Mission::FAST_TURN:
                target_speed = FAST_SPEED_TURN;
                break;

            case Mission::FAST_OVERTAKE:
                target_speed = FAST_SPEED_OVERTAKE;
                break;

            default:
                break;
            }
        }

        template <size_t cols>
        void update(bool (&detection_)[cols])
        {
            lateral_control(detection_);
            longitudinal_control();
        }

        void set_current_velocity(const float velocity)
        {
            current_velocity = velocity;
        }

    private:
        float integral = 0.0f;
        float prev_error = 0.0f;
        float current_velocity = 0.0f;

#ifdef STM32
        // TODO: add timestamp
#else
        std::chrono::time_point<std::chrono::steady_clock> prev_control_timestamp_ = std::chrono::steady_clock::now();
#endif
    };

    void test_sgl()
    {
        sgl::Graph<std::string, sgl::AdjacencyList> graph_robonaut;

        auto A = graph_robonaut.add_vertex("A");
        auto B = graph_robonaut.add_vertex("B");
        auto C = graph_robonaut.add_vertex("C");
        auto D = graph_robonaut.add_vertex("D");
        auto E = graph_robonaut.add_vertex("E");
        auto F = graph_robonaut.add_vertex("F");
        auto G = graph_robonaut.add_vertex("G");
        auto H = graph_robonaut.add_vertex("H");
        auto I = graph_robonaut.add_vertex("I");
        auto J = graph_robonaut.add_vertex("J");
        auto K = graph_robonaut.add_vertex("K");
        auto L = graph_robonaut.add_vertex("L");
        auto M = graph_robonaut.add_vertex("M");
        auto N = graph_robonaut.add_vertex("N");
        auto O = graph_robonaut.add_vertex("O");
        auto P = graph_robonaut.add_vertex("P");
        auto Q = graph_robonaut.add_vertex("Q");
        auto R = graph_robonaut.add_vertex("R");
        auto S = graph_robonaut.add_vertex("S");
        auto T = graph_robonaut.add_vertex("T");
        auto U = graph_robonaut.add_vertex("U");
        auto V = graph_robonaut.add_vertex("V");
        auto W = graph_robonaut.add_vertex("W");
        // extra node not present on the sheet
        auto X = graph_robonaut.add_vertex("X");
        auto balance = graph_robonaut.add_vertex("/");

        graph_robonaut.add_edge(A, B, 4.0f);
        graph_robonaut.add_edge(A, C, M_PI);
        graph_robonaut.add_edge(A, D, M_PI);
        graph_robonaut.add_edge(B, D, M_PI);
        graph_robonaut.add_edge(B, E, M_PI);
        graph_robonaut.add_edge(C, F, M_PI);
        graph_robonaut.add_edge(D, F, M_PI);
        graph_robonaut.add_edge(D, I, 4.0f);
        graph_robonaut.add_edge(D, G, M_PI);
        graph_robonaut.add_edge(E, G, M_PI);
        graph_robonaut.add_edge(E, J, 4.0f);
        graph_robonaut.add_edge(F, H, M_PI);
        graph_robonaut.add_edge(F, I, M_PI);
        graph_robonaut.add_edge(F, G, 4.0f);
        graph_robonaut.add_edge(G, I, M_PI);
        graph_robonaut.add_edge(G, J, M_PI);
        graph_robonaut.add_edge(H, M, 4.0f);
        graph_robonaut.add_edge(H, K, M_PI);
        graph_robonaut.add_edge(I, K, M_PI);
        graph_robonaut.add_edge(I, N, 4.0f);
        graph_robonaut.add_edge(I, L, M_PI);
        graph_robonaut.add_edge(J, L, M_PI);
        graph_robonaut.add_edge(K, M, M_PI);
        graph_robonaut.add_edge(K, N, M_PI);
        graph_robonaut.add_edge(K, L, 4.0f);
        graph_robonaut.add_edge(L, N, M_PI);
        graph_robonaut.add_edge(L, O, M_PI);
        graph_robonaut.add_edge(M, P, M_PI);
        graph_robonaut.add_edge(M, Q, 2.0f);
        graph_robonaut.add_edge(M, R, M_PI);
        graph_robonaut.add_edge(N, R, M_PI);
        graph_robonaut.add_edge(N, S, 2.0f);
        graph_robonaut.add_edge(N, T, M_PI);
        graph_robonaut.add_edge(O, T, M_PI);
        graph_robonaut.add_edge(O, X, 2.0f);
        graph_robonaut.add_edge(O, U, M_PI);
        graph_robonaut.add_edge(P, Q, 2.0f);
        graph_robonaut.add_edge(Q, balance, 5.0f + M_PI);
        graph_robonaut.add_edge(Q, V, M_PI);
        graph_robonaut.add_edge(Q, R, 2.0f);
        graph_robonaut.add_edge(R, S, 2.0f);
        graph_robonaut.add_edge(S, V, M_PI);
        graph_robonaut.add_edge(S, W, M_PI);
        graph_robonaut.add_edge(S, T, 2.0f);
        graph_robonaut.add_edge(T, X, 2.0f);
        graph_robonaut.add_edge(X, W, M_PI);
        graph_robonaut.add_edge(X, U, 2.0f);
        graph_robonaut.add_edge(V, W, 4.0f);

        auto start_robonaut = std::chrono::high_resolution_clock::now();
        std::map<sgl::uuid, std::pair<float, std::vector<sgl::uuid>>> map_robonaut = dijkstra(graph_robonaut, U);
        auto end_robonaut = std::chrono::high_resolution_clock::now();

        std::cout << "time taken: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_robonaut - start_robonaut).count() << "ms" << std::endl;

        sgl::print_dijkstra(graph_robonaut, map_robonaut);
    }

} // namespace jlb

#endif // LOGIC_HXX
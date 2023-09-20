#ifndef LOGIC_HXX
#define LOGIC_HXX

#include "vehicle_model.hxx"
#include "environment.hxx"

#include <chrono>
#include <numbers>
#include "SGL/sgl.hxx"
#define PI std::numbers::pi_v<float>

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
        static constexpr double DELTA_T = 0.1;
        static constexpr double Kp = 0.6;
        static constexpr double Ki = 0.01;
        static constexpr double Kd = 0.4;

        static constexpr double LABYRINTH_SPEED = 10.0;
        static constexpr double LABYRINTH_SPEED_REVERSE = 5.0;
        static constexpr double FAST_SPEED = 30.0;
        static constexpr double FAST_SPEED_TURN = 10.0;
        static constexpr double FAST_SPEED_OVERTAKE = 20.0;

        unsigned long selected = 0;
        double target_angle = 0.0;
        double target_speed = 0.0;

        Direction direction = Direction::STRAIGHT;
        Mission mission = Mission::LABYRINTH;

        Controller() {}
        ~Controller() {}

        double PID(const double error)
        {
            double proportional_term = Kp * error;
            integral += Ki * error * DELTA_T;
            double derivative_term = Kd * (error - prev_error) / DELTA_T;
            return proportional_term + integral + derivative_term;
        }

        template <size_t cols>
        void lateral_control(bool (&detection_)[cols])
        {
            if (std::all_of(std::begin(detection_), std::end(detection_), [](bool b)
                            { return b; }))
            {
                return;
            }

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

            double error = (static_cast<int>(selected - sensor_center)) / static_cast<double>(sensor_center);
            target_angle = PID(error);

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

    private:
        double integral = 0.0;
        double prev_error = 0.0;
    };

    void test_sgl()
    {
        //////////////////////////////////////////////////////////////////////////////
        //
        //       ROBONAUT
        //

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
        graph_robonaut.add_edge(A, C, PI);
        graph_robonaut.add_edge(A, D, PI);
        graph_robonaut.add_edge(B, D, PI);
        graph_robonaut.add_edge(B, E, PI);
        graph_robonaut.add_edge(C, F, PI);
        graph_robonaut.add_edge(D, F, PI);
        graph_robonaut.add_edge(D, I, 4.0f);
        graph_robonaut.add_edge(D, G, PI);
        graph_robonaut.add_edge(E, G, PI);
        graph_robonaut.add_edge(E, J, 4.0f);
        graph_robonaut.add_edge(F, H, PI);
        graph_robonaut.add_edge(F, I, PI);
        graph_robonaut.add_edge(F, G, 4.0f);
        graph_robonaut.add_edge(G, I, PI);
        graph_robonaut.add_edge(G, J, PI);
        graph_robonaut.add_edge(H, M, 4.0f);
        graph_robonaut.add_edge(H, K, PI);
        graph_robonaut.add_edge(I, K, PI);
        graph_robonaut.add_edge(I, N, 4.0f);
        graph_robonaut.add_edge(I, L, PI);
        graph_robonaut.add_edge(J, L, PI);
        graph_robonaut.add_edge(K, M, PI);
        graph_robonaut.add_edge(K, N, PI);
        graph_robonaut.add_edge(K, L, 4.0f);
        graph_robonaut.add_edge(L, N, PI);
        graph_robonaut.add_edge(L, O, PI);
        graph_robonaut.add_edge(M, P, PI);
        graph_robonaut.add_edge(M, Q, 2.0f);
        graph_robonaut.add_edge(M, R, PI);
        graph_robonaut.add_edge(N, R, PI);
        graph_robonaut.add_edge(N, S, 2.0f);
        graph_robonaut.add_edge(N, T, PI);
        graph_robonaut.add_edge(O, T, PI);
        graph_robonaut.add_edge(O, X, 2.0f);
        graph_robonaut.add_edge(O, U, PI);
        graph_robonaut.add_edge(P, Q, 2.0f);
        graph_robonaut.add_edge(Q, balance, 5.0f + PI);
        graph_robonaut.add_edge(Q, V, PI);
        graph_robonaut.add_edge(Q, R, 2.0f);
        graph_robonaut.add_edge(R, S, 2.0f);
        graph_robonaut.add_edge(S, V, PI);
        graph_robonaut.add_edge(S, W, PI);
        graph_robonaut.add_edge(S, T, 2.0f);
        graph_robonaut.add_edge(T, X, 2.0f);
        graph_robonaut.add_edge(X, W, PI);
        graph_robonaut.add_edge(X, U, 2.0f);
        graph_robonaut.add_edge(V, W, 4.0f);

        auto start_robonaut = std::chrono::high_resolution_clock::now();
        std::map<sgl::uuid, std::pair<float, std::vector<sgl::uuid>>> map_robonaut = dijkstra(graph_robonaut, U);
        auto end_robonaut = std::chrono::high_resolution_clock::now();

        std::cout << "time taken: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_robonaut - start_robonaut).count() << "ms" << std::endl;

        sgl::print_dijkstra(graph_robonaut, map_robonaut);
    }

    //
    //       END OF ROBONAUT
    //
    //////////////////////////////////////////////////////////////////////////////

} // namespace

#endif // LOGIC_HXX
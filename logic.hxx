#ifndef LOGIC_HXX
#define LOGIC_HXX

#include "vehicle_model.hxx"

#include <chrono>
#include <numbers>
#include "SGL/sgl.hxx"
#define PI std::numbers::pi_v<float>

namespace jlb
{
    class Controller
    {
    public:
        Controller() {}
        ~Controller() {}

        double lateral_control([[maybe_unused]] const rsim::vmodel::State &state, [[maybe_unused]] bool line_sensor[rsim::smodel::SENSOR_WIDTH])
        {
            // Constants for the control law
            constexpr double Kp = 0.8;  // Proportional gain
            constexpr double Ki = 0.01; // Integral gain
            constexpr double Kd = 0.6;  // Derivative gain

            // closest false to the center
            int closest = 0;
            for (int i = 0; i < rsim::smodel::SENSOR_WIDTH; i++)
            {
                if (!line_sensor[i] && std::abs(i - 8) < std::abs(closest - 8))
                {
                    closest = i;
                }
            }

            // rightmost false
            int rightmost = 0;
            for (int i = 0; i < rsim::smodel::SENSOR_WIDTH; i++)
            {
                if (!line_sensor[i] && i > rightmost)
                {
                    rightmost = i;
                }
            }

            int leftmost = 15;
            for (int i = 0; i < rsim::smodel::SENSOR_WIDTH; i++)
            {
                if (!line_sensor[i] && i < leftmost)
                {
                    leftmost = i;
                }
            }

            bool no_line = true;
            for (int i = 0; i < rsim::smodel::SENSOR_WIDTH; i++)
            {
                if (!line_sensor[i])
                {
                    no_line = false;
                    break;
                }
            }

            double selected = no_line ? rsim::smodel::SENSOR_WIDTH / 2 : rightmost;

            double error = (selected - rsim::smodel::SENSOR_WIDTH / 2) / static_cast<double>(rsim::smodel::SENSOR_WIDTH / 2.0);

            // Calculate PID terms
            double proportional_term = Kp * error;
            integral += Ki * error * rsim::vmodel::DELTA_T;
            double derivative_term = Kd * (error - prev_error) / rsim::vmodel::DELTA_T;
            double target_angle = proportional_term + integral + derivative_term;

            prev_error = error;

            return target_angle;
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
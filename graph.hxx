#ifndef GRAPH_HXX
#define GRAPH_HXX

#include <cstdio>
#include <cstring>
#include <map>
#include <queue>
#include <ranges>
#include <vector>

namespace jlb
{

    struct DijkstraResult
    {
        char              node;
        std::vector<char> path;
        float             weight;
    };

    struct Edge
    {
        char              from       = '@';
        char              to         = '@';
        Direction         direction  = Direction::STRAIGHT;
        std::vector<char> prev_nodes = {'@'};
        float             distance   = std::numeric_limits<float>::infinity();

        static char  pirate_previous_node;
        static char  pirate_next_node;
        static char  pirate_after_next_node;
        static float pirate_section_percentage;
        static int   stolen_gates[NUMBER_OF_GATES];
        static bool  flood;
        static bool  finished;

        Edge(const char from_, const char to_, const Direction direction_, const std::vector<char> &prev_nodes_, const float distance_)
            : from{from_}, to{to_}, direction{direction_}, prev_nodes{prev_nodes_}, distance{distance_}
        {
        }

        Edge() {}

        float get_weight() const
        {
            float weight = distance;

            // PIRATE
            if (from == pirate_next_node || to == pirate_next_node) { weight += 2.0f * WEIGHT_PENALTY; }
            else if (from == pirate_previous_node || to == pirate_previous_node)
            {
                weight += 1.5f * WEIGHT_PENALTY * (1 - pirate_section_percentage);
            }
            else if (to == pirate_after_next_node) { weight += WEIGHT_PENALTY; }
            else if (from == pirate_after_next_node) { weight += WEIGHT_PENALTY * pirate_section_percentage; }

            // FLOOD
            if (!flood && from != BALANCER_END_NODE && to == BALANCER_START_NODE) { weight = std::numeric_limits<float>::infinity(); }

            if (flood)
            {
                // iterate over BALANCER_PROHIBITED_EDGES
                for (int i = 0; i < NUMBER_OF_BALANCER_PROHIBITED_EDGES; ++i)
                {
                    if (from == BALANCER_PROHIBITED_EDGES[i].first && to == BALANCER_PROHIBITED_EDGES[i].second)
                    {
                        weight = WEIGHT_PENALTY / 2.0f;
                        break;
                    }
                }
            }

            // FINISHED
            if (finished)
            {
                // iterate over MISSION_SWITCH_PROHIBITED_EDGES
                for (int i = 0; i < NUMBER_OF_MISSION_SWITCH_PROHIBITED_EDGES; ++i)
                {
                    if (from == MISSION_SWITCH_PROHIBITED_EDGES[i].first && to == MISSION_SWITCH_PROHIBITED_EDGES[i].second)
                    {
                        weight = WEIGHT_PENALTY / 2.0f;
                        break;
                    }
                }
            }

            // CROSS SECTION
            for (int i = 0; i < NUMBER_OF_CROSS_SECTIONS; ++i)
            {
                const auto &cross = CROSS_SECTIONS[i];
                const auto &v     = cross.first;
                const auto &h     = cross.second;

                if ((((from == v.first && to == v.second) || (to == v.first && from == v.second)) &&
                     ((pirate_previous_node == h.first && pirate_next_node == h.second) ||
                      (pirate_next_node == h.first && pirate_previous_node == h.second))) ||
                    (((from == h.first && to == h.second) || (to == h.first && from == h.second)) &&
                     ((pirate_previous_node == v.first && pirate_next_node == v.second) ||
                      (pirate_next_node == v.first && pirate_previous_node == v.second))))
                {
                    weight += 2.0f * WEIGHT_PENALTY;
                    break;
                }
                else if ((((from == v.first && to == v.second) || (to == v.first && from == v.second)) &&
                          ((pirate_next_node == h.first && pirate_after_next_node == h.second) ||
                           (pirate_after_next_node == h.first && pirate_next_node == h.second))) ||
                         (((from == h.first && to == h.second) || (to == h.first && from == h.second)) &&
                          ((pirate_next_node == v.first && pirate_after_next_node == v.second) ||
                           (pirate_after_next_node == v.first && pirate_next_node == v.second))))
                {
                    weight += 1.5f * WEIGHT_PENALTY;
                    break;
                }
            }

            return weight;
        }

        static bool pirate_intersecting(const char node_) { return node_ == pirate_next_node || node_ == pirate_after_next_node; }

        static Edge invalid_edge;

        static void reset()
        {
            pirate_previous_node      = '@';
            pirate_next_node          = '@';
            pirate_after_next_node    = '@';
            pirate_section_percentage = 0.0f;
            flood                     = false;
            finished                  = false;
        }
    };

    char  Edge::pirate_previous_node          = '@';
    char  Edge::pirate_next_node              = '@';
    char  Edge::pirate_after_next_node        = '@';
    float Edge::pirate_section_percentage     = 0.0f;
    int   Edge::stolen_gates[NUMBER_OF_GATES] = {0};
    bool  Edge::flood                         = false;
    bool  Edge::finished                      = false;

    class Node
    {
    public:
        char              name;
        float             x;
        float             y;
        std::vector<Edge> edges;

        Node(char name_, float x_, float y_) : name{name_}, x(x_), y(y_) {}
        ~Node() {}

        void add_edge(const char to_, const Direction direction_, const std::vector<char> &prev_nodes_, const float distance_)
        {
            edges.push_back(Edge{name, to_, direction_, prev_nodes_, distance_});
        }

        void remove_edge(char name_)
        {
            for (auto it = edges.begin(); it != edges.end(); ++it)
            {
                if (it->to == name_)
                {
                    edges.erase(it);
                    break;
                }
            }
        }
    };

    class Graph
    {
    public:
        std::vector<Node> nodes;
        std::vector<char> collected_nodes;

        Node invalid_node = Node{'@', 0.0f, 0.0f};

        void reset() { Edge::reset(); }

        Graph()
        {
#ifdef Q2
            nodes.push_back(Node{static_cast<char>('A'), px_to_m(128.0f), px_to_m(256.0f)});
            nodes.push_back(Node{static_cast<char>('B'), px_to_m(192.0f), px_to_m(192.0f)});
            nodes.push_back(Node{static_cast<char>('C'), px_to_m(256.0f), px_to_m(256.0f)});
            nodes.push_back(Node{static_cast<char>('D'), px_to_m(320.0f), px_to_m(128.0f)});
            nodes.push_back(Node{static_cast<char>('E'), px_to_m(320.0f), px_to_m(256.0f)});
            nodes.push_back(Node{static_cast<char>('F'), px_to_m(448.0f), px_to_m(128.0f)});
            nodes.push_back(Node{static_cast<char>('G'), px_to_m(448.0f), px_to_m(256.0f)});
            nodes.push_back(Node{static_cast<char>('H'), px_to_m(576.0f), px_to_m(128.0f)});
            nodes.push_back(Node{static_cast<char>('I'), px_to_m(576.0f), px_to_m(256.0f)});
            nodes.push_back(Node{static_cast<char>('J'), px_to_m(704.0f), px_to_m(192.0f)});
            nodes.push_back(Node{static_cast<char>('K'), px_to_m(832.0f), px_to_m(128.0f)});
            nodes.push_back(Node{static_cast<char>('L'), px_to_m(832.0f), px_to_m(256.0f)});
            nodes.push_back(Node{static_cast<char>('M'), px_to_m(960.0f), px_to_m(128.0f)});
            nodes.push_back(Node{static_cast<char>('N'), px_to_m(960.0f), px_to_m(256.0f)});
            nodes.push_back(Node{static_cast<char>('O'), px_to_m(1088.0f), px_to_m(192.0f)});
            nodes.push_back(Node{static_cast<char>('P'), px_to_m(1216.0f), px_to_m(128.0f)});
            nodes.push_back(Node{static_cast<char>('Q'), px_to_m(1216.0f), px_to_m(256.0f)});
            nodes.push_back(Node{static_cast<char>('R'), px_to_m(1344.0f), px_to_m(128.0f)});
            nodes.push_back(Node{static_cast<char>('S'), px_to_m(1344.0f), px_to_m(256.0f)});
            nodes.push_back(Node{static_cast<char>('T'), px_to_m(1472.0f), px_to_m(192.0f)});
            nodes.push_back(Node{static_cast<char>('U'), px_to_m(1600.0f), px_to_m(128.0f)});
            nodes.push_back(Node{static_cast<char>('V'), px_to_m(1600.0f), px_to_m(256.0f)});
            nodes.push_back(Node{static_cast<char>('W'), px_to_m(1664.0f), px_to_m(128.0f)});
            nodes.push_back(Node{static_cast<char>('X'), px_to_m(1664.0f), px_to_m(256.0f)});
            nodes.push_back(Node{static_cast<char>('Y'), px_to_m(1792.0f), px_to_m(128.0f)});
            nodes.push_back(Node{static_cast<char>('Z'), px_to_m(1792.0f), px_to_m(224.0f)});
            nodes.push_back(Node{static_cast<char>('['), px_to_m(1920.0f), px_to_m(224.0f)});

#ifndef SIMULATION
            this->operator[]('A').add_edge('C', Direction::RIGHT, {'A'}, 3.6455f);
            this->operator[]('B').add_edge('C', Direction::LEFT, {'D'}, 2.1818f);
            this->operator[]('B').add_edge('D', Direction::STRAIGHT, {'C'}, 4.5223f);
            // this->operator[]('C').add_edge('A', Direction::LEFT, {'E'}, 3.6455f);
            this->operator[]('C').add_edge('E', Direction::STRAIGHT, {'A', 'B'}, 1.5981f);
            this->operator[]('C').add_edge('B', Direction::RIGHT, {'E'}, 2.1818f);
            this->operator[]('D').add_edge('B', Direction::STRAIGHT, {'F'}, 4.5223f);
            this->operator[]('D').add_edge('F', Direction::LEFT, {'B'}, 3.1607f);
            this->operator[]('E').add_edge('C', Direction::STRAIGHT, {'F', 'G'}, 1.5981f);
            this->operator[]('E').add_edge('F', Direction::LEFT, {'C'}, 4.2768f);
            this->operator[]('E').add_edge('G', Direction::RIGHT, {'C'}, 3.8542f);
            this->operator[]('F').add_edge('D', Direction::RIGHT, {'H', 'I'}, 3.1607f);
            this->operator[]('F').add_edge('E', Direction::LEFT, {'H', 'I'}, 4.2768f);
            this->operator[]('F').add_edge('H', Direction::LEFT, {'D', 'E'}, 2.8414f);
            this->operator[]('F').add_edge('I', Direction::RIGHT, {'D', 'E'}, 3.3468f);
            this->operator[]('G').add_edge('E', Direction::LEFT, {'H', 'I'}, 3.8542f);
            this->operator[]('G').add_edge('H', Direction::LEFT, {'E'}, 3.3559f);
            this->operator[]('G').add_edge('I', Direction::RIGHT, {'E'}, 2.8369f);
            this->operator[]('H').add_edge('F', Direction::RIGHT, {'J', 'K'}, 2.8414f);
            this->operator[]('H').add_edge('G', Direction::LEFT, {'J', 'K'}, 3.3559f);
            this->operator[]('H').add_edge('J', Direction::RIGHT, {'F', 'G'}, 2.1980f);
            this->operator[]('H').add_edge('K', Direction::LEFT, {'F', 'G'}, 4.0651f);
            this->operator[]('I').add_edge('F', Direction::RIGHT, {'L'}, 3.3468f);
            this->operator[]('I').add_edge('G', Direction::LEFT, {'L'}, 2.8369f);
            this->operator[]('I').add_edge('L', Direction::RIGHT, {'F', 'G'}, 4.1768f);
            this->operator[]('J').add_edge('H', Direction::LEFT, {'K', 'L'}, 2.1980f);
            this->operator[]('J').add_edge('K', Direction::LEFT, {'H'}, 2.1352f);
            this->operator[]('J').add_edge('L', Direction::RIGHT, {'H'}, 2.3904f);
            this->operator[]('K').add_edge('H', Direction::RIGHT, {'M', 'N'}, 4.0651f);
            this->operator[]('K').add_edge('J', Direction::LEFT, {'M', 'N'}, 2.1352f);
            this->operator[]('K').add_edge('M', Direction::LEFT, {'H', 'J'}, 2.8818f);
            this->operator[]('K').add_edge('N', Direction::RIGHT, {'H', 'J'}, 3.1939f);
            this->operator[]('L').add_edge('I', Direction::LEFT, {'M', 'N'}, 4.1768f);
            this->operator[]('L').add_edge('J', Direction::RIGHT, {'M', 'N'}, 2.3904f);
            this->operator[]('L').add_edge('M', Direction::LEFT, {'I', 'J'}, 3.2893f);
            this->operator[]('L').add_edge('N', Direction::RIGHT, {'I', 'J'}, 2.5774f);
            this->operator[]('M').add_edge('K', Direction::RIGHT, {'O', 'P'}, 2.8818f);
            this->operator[]('M').add_edge('L', Direction::LEFT, {'O', 'P'}, 3.2893f);
            this->operator[]('M').add_edge('O', Direction::RIGHT, {'K', 'L'}, 1.9787f);
            this->operator[]('M').add_edge('P', Direction::LEFT, {'K', 'L'}, 4.1611f);
            this->operator[]('N').add_edge('K', Direction::RIGHT, {'O', 'Q'}, 3.1939f);
            this->operator[]('N').add_edge('L', Direction::LEFT, {'O', 'Q'}, 2.5774f);
            this->operator[]('N').add_edge('O', Direction::LEFT, {'K', 'L'}, 2.2808f);
            this->operator[]('N').add_edge('Q', Direction::RIGHT, {'K', 'L'}, 4.4668f);
            this->operator[]('O').add_edge('M', Direction::RIGHT, {'P'}, 1.9787f);
            this->operator[]('O').add_edge('N', Direction::LEFT, {'P'}, 2.2808f);
            this->operator[]('O').add_edge('P', Direction::RIGHT, {'M', 'N'}, 2.4830f);
            this->operator[]('P').add_edge('M', Direction::RIGHT, {'R', 'S'}, 4.1611f);
            this->operator[]('P').add_edge('O', Direction::LEFT, {'R', 'S'}, 2.4830f);
            this->operator[]('P').add_edge('R', Direction::LEFT, {'M', 'O'}, 3.0528f);
            this->operator[]('P').add_edge('S', Direction::RIGHT, {'M', 'O'}, 3.4623f);
            this->operator[]('Q').add_edge('N', Direction::LEFT, {'R', 'S'}, 4.4668f);
            this->operator[]('Q').add_edge('R', Direction::LEFT, {'N'}, 3.4587f);
            this->operator[]('Q').add_edge('S', Direction::RIGHT, {'N'}, 2.8438f);
            this->operator[]('R').add_edge('P', Direction::RIGHT, {'T', 'U'}, 3.0528f);
            this->operator[]('R').add_edge('Q', Direction::LEFT, {'T', 'U'}, 3.4587f);
            this->operator[]('R').add_edge('T', Direction::RIGHT, {'P', 'Q'}, 2.0351f);
            this->operator[]('R').add_edge('U', Direction::LEFT, {'P', 'Q'}, 3.6602f);
            this->operator[]('S').add_edge('P', Direction::RIGHT, {'T', 'V'}, 3.4623f);
            this->operator[]('S').add_edge('Q', Direction::LEFT, {'T', 'V'}, 2.8438f);
            this->operator[]('S').add_edge('T', Direction::LEFT, {'P', 'Q'}, 2.2329f);
            this->operator[]('S').add_edge('V', Direction::RIGHT, {'P', 'Q'}, 4.0612f);
            this->operator[]('T').add_edge('R', Direction::RIGHT, {'U', 'V'}, 2.0351f);
            this->operator[]('T').add_edge('S', Direction::LEFT, {'U', 'V'}, 2.2329f);
            this->operator[]('T').add_edge('U', Direction::LEFT, {'R', 'S'}, 1.9249f);
            this->operator[]('T').add_edge('V', Direction::RIGHT, {'R', 'S'}, 2.3330f);
            this->operator[]('U').add_edge('R', Direction::RIGHT, {'W'}, 3.6602f);
            this->operator[]('U').add_edge('T', Direction::LEFT, {'W'}, 1.9249f);
            this->operator[]('U').add_edge('W', Direction::STRAIGHT, {'R', 'T'}, 1.8282f);
            this->operator[]('V').add_edge('S', Direction::LEFT, {'X'}, 4.0612f);
            this->operator[]('V').add_edge('T', Direction::RIGHT, {'X'}, 2.3330f);
            this->operator[]('V').add_edge('X', Direction::STRAIGHT, {'S', 'T'}, 1.4863f);
            this->operator[]('W').add_edge('U', Direction::STRAIGHT, {'X', 'Y'}, 1.8282f);
            this->operator[]('W').add_edge('X', Direction::RIGHT, {'U'}, 3.7654f);
            // this->operator[]('W').add_edge('Y', Direction::LEFT, {'U'}, 2.6399f);
            this->operator[]('X').add_edge('V', Direction::STRAIGHT, {'W', 'Z'}, 1.4863f);
            this->operator[]('X').add_edge('W', Direction::LEFT, {'V'}, 3.7654f);
            this->operator[]('X').add_edge('Z', Direction::RIGHT, {'V'}, 2.1500f);
            // this->operator[]('Y').add_edge('W', Direction::RIGHT, {'Y'}, 2.6399f);
            this->operator[]('Z').add_edge('X', Direction::LEFT, {'['}, 2.1500f);
            this->operator[]('Z').add_edge('[', Direction::STRAIGHT, {'X'}, 2.0400f);
            this->operator[]('[').add_edge('Z', Direction::STRAIGHT, {'['}, 2.0400f);
#else
            const auto UNIT            = SQUARE_LENGTH * 2.0f;
            const auto QUARTER_CIRCLE  = 2 * UNIT * M_PI / 4.0f;
            const auto CROSS_UNIT      = 3.65f;
            const auto HALF_CROSS_UNIT = 2.7f;

            this->operator[]('A').add_edge('C', Direction::RIGHT, {'A'}, 2.0f * UNIT);
            this->operator[]('B').add_edge('C', Direction::LEFT, {'D'}, QUARTER_CIRCLE);
            this->operator[]('B').add_edge('D', Direction::STRAIGHT, {'C'}, QUARTER_CIRCLE + UNIT);
            // this->operator[]('C').add_edge('A', Direction::LEFT, {'E'}, 2.0f * UNIT);
            this->operator[]('C').add_edge('E', Direction::STRAIGHT, {'A', 'B'}, UNIT);
            this->operator[]('C').add_edge('B', Direction::RIGHT, {'E'}, QUARTER_CIRCLE);
            this->operator[]('D').add_edge('B', Direction::STRAIGHT, {'F'}, QUARTER_CIRCLE + UNIT);
            this->operator[]('D').add_edge('F', Direction::LEFT, {'B'}, 2.0f * UNIT);
            this->operator[]('E').add_edge('C', Direction::STRAIGHT, {'F', 'G'}, UNIT);
            this->operator[]('E').add_edge('F', Direction::LEFT, {'C'}, CROSS_UNIT);
            this->operator[]('E').add_edge('G', Direction::RIGHT, {'C'}, 2.0f * UNIT);
            this->operator[]('F').add_edge('D', Direction::RIGHT, {'H', 'I'}, 2.0f * UNIT);
            this->operator[]('F').add_edge('E', Direction::LEFT, {'H', 'I'}, CROSS_UNIT);
            this->operator[]('F').add_edge('H', Direction::LEFT, {'D', 'E'}, 2.0f * UNIT);
            this->operator[]('F').add_edge('I', Direction::RIGHT, {'D', 'E'}, CROSS_UNIT);
            this->operator[]('G').add_edge('E', Direction::LEFT, {'H', 'I'}, 2.0f * UNIT);
            this->operator[]('G').add_edge('H', Direction::LEFT, {'E'}, CROSS_UNIT);
            this->operator[]('G').add_edge('I', Direction::RIGHT, {'E'}, 2.0f * UNIT);
            this->operator[]('H').add_edge('F', Direction::RIGHT, {'J', 'K'}, 2.0f * UNIT);
            this->operator[]('H').add_edge('G', Direction::LEFT, {'J', 'K'}, CROSS_UNIT);
            this->operator[]('H').add_edge('J', Direction::RIGHT, {'F', 'G'}, HALF_CROSS_UNIT);
            this->operator[]('H').add_edge('K', Direction::LEFT, {'F', 'G'}, 4.0f * UNIT);
            this->operator[]('I').add_edge('F', Direction::RIGHT, {'L'}, CROSS_UNIT);
            this->operator[]('I').add_edge('G', Direction::LEFT, {'L'}, 2.0f * UNIT);
            this->operator[]('I').add_edge('L', Direction::RIGHT, {'F', 'G'}, 4.0f * UNIT);
            this->operator[]('J').add_edge('H', Direction::LEFT, {'K', 'L'}, HALF_CROSS_UNIT);
            this->operator[]('J').add_edge('K', Direction::LEFT, {'H'}, HALF_CROSS_UNIT);
            this->operator[]('J').add_edge('L', Direction::RIGHT, {'H'}, HALF_CROSS_UNIT);
            this->operator[]('K').add_edge('H', Direction::RIGHT, {'M', 'N'}, 4.0f * UNIT);
            this->operator[]('K').add_edge('J', Direction::LEFT, {'M', 'N'}, HALF_CROSS_UNIT);
            this->operator[]('K').add_edge('M', Direction::LEFT, {'H', 'J'}, 2.0f * UNIT);
            this->operator[]('K').add_edge('N', Direction::RIGHT, {'H', 'J'}, CROSS_UNIT);
            this->operator[]('L').add_edge('I', Direction::LEFT, {'M', 'N'}, 4.0f * UNIT);
            this->operator[]('L').add_edge('J', Direction::RIGHT, {'M', 'N'}, HALF_CROSS_UNIT);
            this->operator[]('L').add_edge('M', Direction::LEFT, {'I', 'J'}, CROSS_UNIT);
            this->operator[]('L').add_edge('N', Direction::RIGHT, {'I', 'J'}, 2.0f * UNIT);
            this->operator[]('M').add_edge('K', Direction::RIGHT, {'O', 'P'}, 2.0f * UNIT);
            this->operator[]('M').add_edge('L', Direction::LEFT, {'O', 'P'}, CROSS_UNIT);
            this->operator[]('M').add_edge('O', Direction::RIGHT, {'K', 'L'}, HALF_CROSS_UNIT);
            this->operator[]('M').add_edge('P', Direction::LEFT, {'K', 'L'}, 4.0f * UNIT);
            this->operator[]('N').add_edge('K', Direction::RIGHT, {'O', 'Q'}, CROSS_UNIT);
            this->operator[]('N').add_edge('L', Direction::LEFT, {'O', 'Q'}, 2.0f * UNIT);
            this->operator[]('N').add_edge('O', Direction::LEFT, {'K', 'L'}, HALF_CROSS_UNIT);
            this->operator[]('N').add_edge('Q', Direction::RIGHT, {'K', 'L'}, 4.0f * UNIT);
            this->operator[]('O').add_edge('M', Direction::RIGHT, {'P'}, HALF_CROSS_UNIT);
            this->operator[]('O').add_edge('N', Direction::LEFT, {'P'}, HALF_CROSS_UNIT);
            this->operator[]('O').add_edge('P', Direction::RIGHT, {'M', 'N'}, HALF_CROSS_UNIT);
            this->operator[]('P').add_edge('M', Direction::RIGHT, {'R', 'S'}, 4.0f * UNIT);
            this->operator[]('P').add_edge('O', Direction::LEFT, {'R', 'S'}, HALF_CROSS_UNIT);
            this->operator[]('P').add_edge('R', Direction::LEFT, {'M', 'O'}, 2.0f * UNIT);
            this->operator[]('P').add_edge('S', Direction::RIGHT, {'M', 'O'}, CROSS_UNIT);
            this->operator[]('Q').add_edge('N', Direction::LEFT, {'R', 'S'}, 4.0f * UNIT);
            this->operator[]('Q').add_edge('R', Direction::LEFT, {'N'}, CROSS_UNIT);
            this->operator[]('Q').add_edge('S', Direction::RIGHT, {'N'}, 2.0f * UNIT);
            this->operator[]('R').add_edge('P', Direction::RIGHT, {'T', 'U'}, 2.0f * UNIT);
            this->operator[]('R').add_edge('Q', Direction::LEFT, {'T', 'U'}, CROSS_UNIT);
            this->operator[]('R').add_edge('T', Direction::RIGHT, {'P', 'Q'}, HALF_CROSS_UNIT);
            this->operator[]('R').add_edge('U', Direction::LEFT, {'P', 'Q'}, 4.0f * UNIT);
            this->operator[]('S').add_edge('P', Direction::RIGHT, {'T', 'V'}, CROSS_UNIT);
            this->operator[]('S').add_edge('Q', Direction::LEFT, {'T', 'V'}, 2.0f * UNIT);
            this->operator[]('S').add_edge('T', Direction::LEFT, {'P', 'Q'}, HALF_CROSS_UNIT);
            this->operator[]('S').add_edge('V', Direction::RIGHT, {'P', 'Q'}, 4.0f * UNIT);
            this->operator[]('T').add_edge('R', Direction::RIGHT, {'U', 'V'}, HALF_CROSS_UNIT);
            this->operator[]('T').add_edge('S', Direction::LEFT, {'U', 'V'}, HALF_CROSS_UNIT);
            this->operator[]('T').add_edge('U', Direction::LEFT, {'R', 'S'}, HALF_CROSS_UNIT);
            this->operator[]('T').add_edge('V', Direction::RIGHT, {'R', 'S'}, HALF_CROSS_UNIT);
            this->operator[]('U').add_edge('R', Direction::RIGHT, {'W'}, 4.0f * UNIT);
            this->operator[]('U').add_edge('T', Direction::LEFT, {'W'}, HALF_CROSS_UNIT);
            this->operator[]('U').add_edge('W', Direction::STRAIGHT, {'R', 'T'}, UNIT);
            this->operator[]('V').add_edge('S', Direction::LEFT, {'X'}, 4.0f * UNIT);
            this->operator[]('V').add_edge('T', Direction::RIGHT, {'X'}, HALF_CROSS_UNIT);
            this->operator[]('V').add_edge('X', Direction::STRAIGHT, {'S', 'T'}, UNIT);
            this->operator[]('W').add_edge('U', Direction::STRAIGHT, {'X', 'Y'}, UNIT);
            this->operator[]('W').add_edge('X', Direction::RIGHT, {'U'}, 2.0f * QUARTER_CIRCLE);
            // this->operator[]('W').add_edge('Y', Direction::LEFT, {'U'}, 2.0f * UNIT);
            this->operator[]('X').add_edge('V', Direction::STRAIGHT, {'W', 'Z'}, UNIT);
            this->operator[]('X').add_edge('W', Direction::LEFT, {'V'}, 2.0f * QUARTER_CIRCLE);
            this->operator[]('X').add_edge('Z', Direction::RIGHT, {'V'}, 2.8f);
            // this->operator[]('Y').add_edge('W', Direction::RIGHT, {'Y'}, 2.0f * UNIT);
            this->operator[]('Z').add_edge('X', Direction::LEFT, {'['}, 2.8f);
            this->operator[]('Z').add_edge('[', Direction::STRAIGHT, {'X'}, 2.0f * UNIT);
            this->operator[]('[').add_edge('Z', Direction::STRAIGHT, {'['}, 2.0f * UNIT);
#endif

#else
            nodes.push_back(Node{static_cast<char>('A'), px_to_m(704), px_to_m(448)});
            nodes.push_back(Node{static_cast<char>('B'), px_to_m(704), px_to_m(576)});
            nodes.push_back(Node{static_cast<char>('C'), px_to_m(640), px_to_m(384)});
            nodes.push_back(Node{static_cast<char>('D'), px_to_m(640), px_to_m(512)});
            nodes.push_back(Node{static_cast<char>('E'), px_to_m(640), px_to_m(640)});
            nodes.push_back(Node{static_cast<char>('F'), px_to_m(576), px_to_m(448)});
            nodes.push_back(Node{static_cast<char>('G'), px_to_m(576), px_to_m(576)});
            nodes.push_back(Node{static_cast<char>('H'), px_to_m(512), px_to_m(384)});
            nodes.push_back(Node{static_cast<char>('I'), px_to_m(512), px_to_m(512)});
            nodes.push_back(Node{static_cast<char>('J'), px_to_m(512), px_to_m(640)});
            nodes.push_back(Node{static_cast<char>('K'), px_to_m(448), px_to_m(448)});
            nodes.push_back(Node{static_cast<char>('L'), px_to_m(448), px_to_m(576)});
            nodes.push_back(Node{static_cast<char>('M'), px_to_m(384), px_to_m(384)});
            nodes.push_back(Node{static_cast<char>('N'), px_to_m(384), px_to_m(512)});
            nodes.push_back(Node{static_cast<char>('O'), px_to_m(384), px_to_m(640)});
            nodes.push_back(Node{static_cast<char>('P'), px_to_m(320), px_to_m(320)});
            nodes.push_back(Node{static_cast<char>('Q'), px_to_m(320), px_to_m(384)});
            nodes.push_back(Node{static_cast<char>('R'), px_to_m(320), px_to_m(448)});
            nodes.push_back(Node{static_cast<char>('S'), px_to_m(320), px_to_m(512)});
            nodes.push_back(Node{static_cast<char>('T'), px_to_m(320), px_to_m(576)});
            nodes.push_back(Node{static_cast<char>('U'), px_to_m(320), px_to_m(704)});
            nodes.push_back(Node{static_cast<char>('V'), px_to_m(256), px_to_m(448)});
            nodes.push_back(Node{static_cast<char>('W'), px_to_m(256), px_to_m(576)});
            nodes.push_back(Node{static_cast<char>('X'), px_to_m(96), px_to_m(448)});
            nodes.push_back(Node{static_cast<char>('Y'), px_to_m(96), px_to_m(576)});

#ifndef SIMULATION
            this->operator[]('A').add_edge('C', Direction::LEFT, {'B', 'D'}, 1.99f);
            this->operator[]('A').add_edge('B', Direction::LEFT, {'C'}, 2.41f);
            this->operator[]('A').add_edge('D', Direction::RIGHT, {'C'}, 1.98f);
            this->operator[]('B').add_edge('A', Direction::RIGHT, {'E'}, 2.41f);
            this->operator[]('B').add_edge('E', Direction::RIGHT, {'A', 'D'}, 1.98f);
            this->operator[]('B').add_edge('D', Direction::LEFT, {'E'}, 1.99f);
            this->operator[]('C').add_edge('A', Direction::RIGHT, {'F'}, 1.99f);
            this->operator[]('C').add_edge('F', Direction::LEFT, {'A'}, 1.98f);
            this->operator[]('D').add_edge('A', Direction::LEFT, {'F', 'G', 'I'}, 1.98f);
            this->operator[]('D').add_edge('B', Direction::RIGHT, {'F', 'G', 'I'}, 1.99f);
            this->operator[]('D').add_edge('G', Direction::LEFT, {'A', 'B'}, 1.98f);
            this->operator[]('D').add_edge('I', Direction::STRAIGHT, {'A', 'B'}, 2.41f);
            this->operator[]('D').add_edge('F', Direction::RIGHT, {'A', 'B'}, 1.97f);
            this->operator[]('E').add_edge('B', Direction::LEFT, {'G', 'J'}, 1.98f);
            this->operator[]('E').add_edge('J', Direction::LEFT, {'B'}, 2.41f);
            this->operator[]('E').add_edge('G', Direction::RIGHT, {'B'}, 1.97f);
            this->operator[]('F').add_edge('C', Direction::RIGHT, {'D', 'G', 'I'}, 1.98f);
            this->operator[]('F').add_edge('D', Direction::LEFT, {'C', 'H'}, 1.97f);
            this->operator[]('F').add_edge('G', Direction::STRAIGHT, {'C', 'H'}, 2.41f);
            this->operator[]('F').add_edge('I', Direction::RIGHT, {'C', 'H'}, 1.98f);
            this->operator[]('F').add_edge('H', Direction::LEFT, {'D', 'G', 'I'}, 2.0f);
            this->operator[]('G').add_edge('F', Direction::STRAIGHT, {'E', 'J'}, 2.41f);
            this->operator[]('G').add_edge('D', Direction::RIGHT, {'E', 'J'}, 1.98f);
            this->operator[]('G').add_edge('E', Direction::LEFT, {'D', 'F', 'I'}, 1.97f);
            this->operator[]('G').add_edge('J', Direction::RIGHT, {'D', 'F', 'I'}, 1.98f);
            this->operator[]('G').add_edge('I', Direction::LEFT, {'E', 'J'}, 1.99f);
            this->operator[]('H').add_edge('F', Direction::RIGHT, {'K', 'M'}, 2.0f);
            this->operator[]('H').add_edge('K', Direction::LEFT, {'F'}, 1.98f);
            this->operator[]('H').add_edge('M', Direction::RIGHT, {'F'}, 2.41f);
            this->operator[]('I').add_edge('F', Direction::LEFT, {'K', 'L', 'N'}, 1.98f);
            this->operator[]('I').add_edge('D', Direction::STRAIGHT, {'K', 'L', 'N'}, 2.41f);
            this->operator[]('I').add_edge('G', Direction::RIGHT, {'K', 'L', 'N'}, 1.99f);
            this->operator[]('I').add_edge('L', Direction::LEFT, {'D', 'F', 'G'}, 1.98f);
            this->operator[]('I').add_edge('N', Direction::STRAIGHT, {'D', 'F', 'G'}, 2.41f);
            this->operator[]('I').add_edge('K', Direction::RIGHT, {'D', 'F', 'G'}, 1.97f);
            this->operator[]('J').add_edge('G', Direction::LEFT, {'L'}, 1.98f);
            this->operator[]('J').add_edge('E', Direction::RIGHT, {'L'}, 2.41f);
            this->operator[]('J').add_edge('L', Direction::RIGHT, {'E', 'G'}, 1.97f);
            this->operator[]('K').add_edge('H', Direction::RIGHT, {'I', 'L', 'N'}, 1.98f);
            this->operator[]('K').add_edge('I', Direction::LEFT, {'H', 'M'}, 1.97f);
            this->operator[]('K').add_edge('L', Direction::STRAIGHT, {'H', 'M'}, 2.41f);
            this->operator[]('K').add_edge('N', Direction::RIGHT, {'H', 'M'}, 1.98f);
            this->operator[]('K').add_edge('M', Direction::LEFT, {'I', 'L', 'N'}, 2.0f);
            this->operator[]('L').add_edge('K', Direction::STRAIGHT, {'J', 'O'}, 2.41f);
            this->operator[]('L').add_edge('I', Direction::RIGHT, {'J', 'O'}, 1.98f);
            this->operator[]('L').add_edge('J', Direction::LEFT, {'I', 'K', 'N'}, 1.97f);
            this->operator[]('L').add_edge('O', Direction::RIGHT, {'I', 'K', 'N'}, 1.98f);
            this->operator[]('L').add_edge('N', Direction::LEFT, {'J', 'O'}, 1.99f);
            this->operator[]('M').add_edge('H', Direction::LEFT, {'P', 'Q', 'R'}, 2.41f);
            this->operator[]('M').add_edge('K', Direction::RIGHT, {'P', 'Q', 'R'}, 2.0f);
            this->operator[]('M').add_edge('R', Direction::LEFT, {'H', 'K'}, 1.98f);
            this->operator[]('M').add_edge('Q', Direction::STRAIGHT, {'H', 'K'}, 1.2f);
            // this->operator[]('M').add_edge('P', Direction::RIGHT, {'H', 'K'}, 1.97f);
            this->operator[]('N').add_edge('K', Direction::LEFT, {'R', 'S', 'T'}, 1.98f);
            this->operator[]('N').add_edge('I', Direction::STRAIGHT, {'R', 'S', 'T'}, 2.41f);
            this->operator[]('N').add_edge('L', Direction::RIGHT, {'R', 'S', 'T'}, 1.99f);
            this->operator[]('N').add_edge('T', Direction::LEFT, {'K', 'I', 'L'}, 1.98f);
            this->operator[]('N').add_edge('S', Direction::STRAIGHT, {'K', 'I', 'L'}, 1.2f);
            this->operator[]('N').add_edge('R', Direction::RIGHT, {'K', 'I', 'L'}, 1.97f);
            this->operator[]('O').add_edge('L', Direction::LEFT, {'T', 'U', 'W'}, 1.98f);
            // this->operator[]('O').add_edge('U', Direction::LEFT, {'L'}, 1.98f);
            this->operator[]('O').add_edge('W', Direction::STRAIGHT, {'L'}, 3.18f);
            this->operator[]('O').add_edge('T', Direction::RIGHT, {'L'}, 1.97f);
            // this->operator[]('P').add_edge('M', Direction::LEFT, {'P'}, 1.97f);
            // this->operator[]('P').add_edge('R', Direction::RIGHT, {'P'}, 2.41f);
            this->operator[]('Q').add_edge('M', Direction::STRAIGHT, {'V', 'X'}, 1.2f);
            this->operator[]('Q').add_edge('V', Direction::LEFT, {'M'}, 1.99f);
            this->operator[]('Q').add_edge('X', Direction::RIGHT, {'M'}, 4.885f);
            this->operator[]('R').add_edge('M', Direction::RIGHT, {'N', 'T'}, 1.98f);
            this->operator[]('R').add_edge('N', Direction::LEFT, {'M', 'P'}, 1.97f);
            this->operator[]('R').add_edge('T', Direction::RIGHT, {'M', 'P'}, 2.41f);
            // this->operator[]('R').add_edge('P', Direction::LEFT, {'N', 'T'}, 2.41f);
            this->operator[]('S').add_edge('N', Direction::STRAIGHT, {'V', 'W'}, 1.2f);
            this->operator[]('S').add_edge('W', Direction::LEFT, {'N'}, 1.99f);
            this->operator[]('S').add_edge('V', Direction::RIGHT, {'N'}, 1.98f);
            this->operator[]('T').add_edge('R', Direction::LEFT, {'O', 'U'}, 2.41f);
            this->operator[]('T').add_edge('N', Direction::RIGHT, {'O', 'U'}, 1.98f);
            this->operator[]('T').add_edge('O', Direction::LEFT, {'N', 'R'}, 1.97f);
            // this->operator[]('T').add_edge('U', Direction::RIGHT, {'N', 'R'}, 2.41f);
            this->operator[]('U').add_edge('T', Direction::LEFT, {'U'}, 2.41f);
            this->operator[]('U').add_edge('O', Direction::RIGHT, {'U'}, 1.98f);
            this->operator[]('V').add_edge('Q', Direction::RIGHT, {'S', 'W'}, 1.99f);
            this->operator[]('V').add_edge('S', Direction::LEFT, {'Q'}, 1.98f);
            this->operator[]('V').add_edge('W', Direction::RIGHT, {'Q'}, 2.41f);
            this->operator[]('W').add_edge('V', Direction::LEFT, {'O'}, 2.41f);
            this->operator[]('W').add_edge('S', Direction::RIGHT, {'O'}, 1.99f);
            this->operator[]('W').add_edge('O', Direction::STRAIGHT, {'S', 'V'}, 3.18f);
            this->operator[]('X').add_edge('Q', Direction::LEFT, {'Y'}, 4.885f);
            this->operator[]('X').add_edge('Y', Direction::STRAIGHT, {'Q'}, 2.04f);
            this->operator[]('Y').add_edge('X', Direction::STRAIGHT, {'Y'}, 2.04f);
#else
            const auto UNIT           = SQUARE_LENGTH * 2.0f;
            const auto QUARTER_CIRCLE = 2 * UNIT * M_PI / 4.0f;

            this->operator[]('A').add_edge('C', Direction::LEFT, {'B', 'D'}, QUARTER_CIRCLE);
            this->operator[]('A').add_edge('B', Direction::LEFT, {'C'}, 2.0f * UNIT);
            this->operator[]('A').add_edge('D', Direction::RIGHT, {'C'}, QUARTER_CIRCLE);
            this->operator[]('B').add_edge('A', Direction::RIGHT, {'E'}, 2.0f * UNIT);
            this->operator[]('B').add_edge('E', Direction::RIGHT, {'A', 'D'}, QUARTER_CIRCLE);
            this->operator[]('B').add_edge('D', Direction::LEFT, {'E'}, QUARTER_CIRCLE);
            this->operator[]('C').add_edge('A', Direction::RIGHT, {'F'}, QUARTER_CIRCLE);
            this->operator[]('C').add_edge('F', Direction::LEFT, {'A'}, QUARTER_CIRCLE);
            this->operator[]('D').add_edge('A', Direction::LEFT, {'F', 'G', 'I'}, QUARTER_CIRCLE);
            this->operator[]('D').add_edge('B', Direction::RIGHT, {'F', 'G', 'I'}, QUARTER_CIRCLE);
            this->operator[]('D').add_edge('G', Direction::LEFT, {'A', 'B'}, QUARTER_CIRCLE);
            this->operator[]('D').add_edge('I', Direction::STRAIGHT, {'A', 'B'}, 2.0f * UNIT);
            this->operator[]('D').add_edge('F', Direction::RIGHT, {'A', 'B'}, QUARTER_CIRCLE);
            this->operator[]('E').add_edge('B', Direction::LEFT, {'G', 'J'}, QUARTER_CIRCLE);
            this->operator[]('E').add_edge('J', Direction::LEFT, {'B'}, 2.0f * UNIT);
            this->operator[]('E').add_edge('G', Direction::RIGHT, {'B'}, QUARTER_CIRCLE);
            this->operator[]('F').add_edge('C', Direction::RIGHT, {'D', 'G', 'I'}, QUARTER_CIRCLE);
            this->operator[]('F').add_edge('D', Direction::LEFT, {'C', 'H'}, QUARTER_CIRCLE);
            this->operator[]('F').add_edge('G', Direction::STRAIGHT, {'C', 'H'}, 2.0f * UNIT);
            this->operator[]('F').add_edge('I', Direction::RIGHT, {'C', 'H'}, QUARTER_CIRCLE);
            this->operator[]('F').add_edge('H', Direction::LEFT, {'D', 'G', 'I'}, QUARTER_CIRCLE);
            this->operator[]('G').add_edge('F', Direction::STRAIGHT, {'E', 'J'}, 2.0f * UNIT);
            this->operator[]('G').add_edge('D', Direction::RIGHT, {'E', 'J'}, QUARTER_CIRCLE);
            this->operator[]('G').add_edge('E', Direction::LEFT, {'D', 'F', 'I'}, QUARTER_CIRCLE);
            this->operator[]('G').add_edge('J', Direction::RIGHT, {'D', 'F', 'I'}, QUARTER_CIRCLE);
            this->operator[]('G').add_edge('I', Direction::LEFT, {'E', 'J'}, QUARTER_CIRCLE);
            this->operator[]('H').add_edge('F', Direction::RIGHT, {'K', 'M'}, QUARTER_CIRCLE);
            this->operator[]('H').add_edge('K', Direction::LEFT, {'F'}, QUARTER_CIRCLE);
            this->operator[]('H').add_edge('M', Direction::RIGHT, {'F'}, 2.0f * UNIT);
            this->operator[]('I').add_edge('F', Direction::LEFT, {'K', 'L', 'N'}, QUARTER_CIRCLE);
            this->operator[]('I').add_edge('D', Direction::STRAIGHT, {'K', 'L', 'N'}, 2.0f * UNIT);
            this->operator[]('I').add_edge('G', Direction::RIGHT, {'K', 'L', 'N'}, QUARTER_CIRCLE);
            this->operator[]('I').add_edge('L', Direction::LEFT, {'D', 'F', 'G'}, QUARTER_CIRCLE);
            this->operator[]('I').add_edge('N', Direction::STRAIGHT, {'D', 'F', 'G'}, 2.0f * UNIT);
            this->operator[]('I').add_edge('K', Direction::RIGHT, {'D', 'F', 'G'}, QUARTER_CIRCLE);
            this->operator[]('J').add_edge('G', Direction::LEFT, {'L'}, QUARTER_CIRCLE);
            this->operator[]('J').add_edge('E', Direction::RIGHT, {'L'}, 2.0f * UNIT);
            this->operator[]('J').add_edge('L', Direction::RIGHT, {'E', 'G'}, QUARTER_CIRCLE);
            this->operator[]('K').add_edge('H', Direction::RIGHT, {'I', 'L', 'N'}, QUARTER_CIRCLE);
            this->operator[]('K').add_edge('I', Direction::LEFT, {'H', 'M'}, QUARTER_CIRCLE);
            this->operator[]('K').add_edge('L', Direction::STRAIGHT, {'H', 'M'}, 2.0f * UNIT);
            this->operator[]('K').add_edge('N', Direction::RIGHT, {'H', 'M'}, QUARTER_CIRCLE);
            this->operator[]('K').add_edge('M', Direction::LEFT, {'I', 'L', 'N'}, QUARTER_CIRCLE);
            this->operator[]('L').add_edge('K', Direction::STRAIGHT, {'J', 'O'}, 2.0f * UNIT);
            this->operator[]('L').add_edge('I', Direction::RIGHT, {'J', 'O'}, QUARTER_CIRCLE);
            this->operator[]('L').add_edge('J', Direction::LEFT, {'I', 'K', 'N'}, QUARTER_CIRCLE);
            this->operator[]('L').add_edge('O', Direction::RIGHT, {'I', 'K', 'N'}, QUARTER_CIRCLE);
            this->operator[]('L').add_edge('N', Direction::LEFT, {'J', 'O'}, QUARTER_CIRCLE);
            this->operator[]('M').add_edge('H', Direction::LEFT, {'P', 'Q', 'R'}, 2.0f * UNIT);
            this->operator[]('M').add_edge('K', Direction::RIGHT, {'P', 'Q', 'R'}, QUARTER_CIRCLE);
            this->operator[]('M').add_edge('R', Direction::LEFT, {'H', 'K'}, QUARTER_CIRCLE);
            this->operator[]('M').add_edge('Q', Direction::STRAIGHT, {'H', 'K'}, UNIT);
            // this->operator[]('M').add_edge('P', Direction::RIGHT, {'H', 'K'}, QUARTER_CIRCLE);
            this->operator[]('N').add_edge('K', Direction::LEFT, {'R', 'S', 'T'}, QUARTER_CIRCLE);
            this->operator[]('N').add_edge('I', Direction::STRAIGHT, {'R', 'S', 'T'}, 2.0f * UNIT);
            this->operator[]('N').add_edge('L', Direction::RIGHT, {'R', 'S', 'T'}, QUARTER_CIRCLE);
            this->operator[]('N').add_edge('T', Direction::LEFT, {'K', 'I', 'L'}, QUARTER_CIRCLE);
            this->operator[]('N').add_edge('S', Direction::STRAIGHT, {'K', 'I', 'L'}, UNIT);
            this->operator[]('N').add_edge('R', Direction::RIGHT, {'K', 'I', 'L'}, QUARTER_CIRCLE);
            this->operator[]('O').add_edge('L', Direction::LEFT, {'T', 'U', 'W'}, QUARTER_CIRCLE);
            // this->operator[]('O').add_edge('U', Direction::LEFT, {'L'}, QUARTER_CIRCLE);
            this->operator[]('O').add_edge('W', Direction::STRAIGHT, {'L'}, UNIT + QUARTER_CIRCLE);
            this->operator[]('O').add_edge('T', Direction::RIGHT, {'L'}, QUARTER_CIRCLE);
            // this->operator[]('P').add_edge('M', Direction::LEFT, {'P'}, QUARTER_CIRCLE);
            // this->operator[]('P').add_edge('R', Direction::RIGHT, {'P'}, 2.0f * UNIT);
            this->operator[]('Q').add_edge('M', Direction::STRAIGHT, {'V', 'X'}, UNIT);
            this->operator[]('Q').add_edge('V', Direction::LEFT, {'M'}, QUARTER_CIRCLE);
            this->operator[]('Q').add_edge('X', Direction::RIGHT, {'M'}, 3.0f * UNIT + 0.85f * QUARTER_CIRCLE);
            this->operator[]('R').add_edge('M', Direction::RIGHT, {'N', 'T'}, QUARTER_CIRCLE);
            this->operator[]('R').add_edge('N', Direction::LEFT, {'M', 'P'}, QUARTER_CIRCLE);
            this->operator[]('R').add_edge('T', Direction::RIGHT, {'M', 'P'}, 2.0f * UNIT);
            // this->operator[]('R').add_edge('P', Direction::LEFT, {'N', 'T'}, 2.0f * UNIT);
            this->operator[]('S').add_edge('N', Direction::STRAIGHT, {'V', 'W'}, UNIT);
            this->operator[]('S').add_edge('W', Direction::LEFT, {'N'}, QUARTER_CIRCLE);
            this->operator[]('S').add_edge('V', Direction::RIGHT, {'N'}, QUARTER_CIRCLE);
            this->operator[]('T').add_edge('R', Direction::LEFT, {'O', 'U'}, 2.0f * UNIT);
            this->operator[]('T').add_edge('N', Direction::RIGHT, {'O', 'U'}, QUARTER_CIRCLE);
            this->operator[]('T').add_edge('O', Direction::LEFT, {'N', 'R'}, QUARTER_CIRCLE);
            // this->operator[]('T').add_edge('U', Direction::RIGHT, {'N', 'R'}, 2.0f * UNIT);
            this->operator[]('U').add_edge('T', Direction::LEFT, {'U'}, 2.0f * UNIT);
            this->operator[]('U').add_edge('O', Direction::RIGHT, {'U'}, QUARTER_CIRCLE);
            this->operator[]('V').add_edge('Q', Direction::RIGHT, {'S', 'W'}, QUARTER_CIRCLE);
            this->operator[]('V').add_edge('S', Direction::LEFT, {'Q'}, QUARTER_CIRCLE);
            this->operator[]('V').add_edge('W', Direction::RIGHT, {'Q'}, 2.0f * UNIT);
            this->operator[]('W').add_edge('V', Direction::LEFT, {'O'}, 2.0f * UNIT);
            this->operator[]('W').add_edge('S', Direction::RIGHT, {'O'}, QUARTER_CIRCLE);
            this->operator[]('W').add_edge('O', Direction::STRAIGHT, {'S', 'V'}, UNIT + QUARTER_CIRCLE);
            this->operator[]('X').add_edge('Q', Direction::LEFT, {'Y'}, 3.0f * UNIT + 0.85f * QUARTER_CIRCLE);
            this->operator[]('X').add_edge('Y', Direction::STRAIGHT, {'Q'}, 2.0f * UNIT);
            this->operator[]('Y').add_edge('X', Direction::STRAIGHT, {'Y'}, 2.0f * UNIT);
#endif
#endif
        }

        ~Graph() {}

        Node &operator[](char name)
        {
#ifdef Q2
            if (nodes.empty() || name < 'A' || name > '[') { return invalid_node; }
            return nodes[static_cast<int>(name - 'A')];
#else
            if (nodes.empty() || name < 'A' || name > 'Y') { return invalid_node; }
            return nodes[static_cast<int>(name - 'A')];
#endif
        }

        const Node &operator[](char name) const
        {
#ifdef Q2
            if (nodes.empty() || name < 'A' || name > '[') { return invalid_node; }
            return nodes[static_cast<int>(name - 'A')];
#else
            if (nodes.empty() || name < 'A' || name > 'Y') { return invalid_node; }
            return nodes[static_cast<int>(name - 'A')];
#endif
        }

        void pirate_callback(const char prev_node_, const char next_node_, const char after_next_node_, const int section_percentage_)
        {
            if (Edge::pirate_next_node == prev_node_) { Edge::stolen_gates[static_cast<int>(prev_node_ - 'A')] += 1; }

            Edge::pirate_previous_node   = prev_node_;
            Edge::pirate_next_node       = next_node_;
            Edge::pirate_after_next_node = after_next_node_;
            if (section_percentage_ > 100) { Edge::pirate_section_percentage = 1.0f; }
            else if (section_percentage_ < 0) { Edge::pirate_section_percentage = 0.0f; }
            else { Edge::pirate_section_percentage = section_percentage_ / 100.0f; }
        }

        void print_dijkstra(const std::map<char, std::pair<float, std::vector<char>>> &map)
        {
#ifdef SIMULATION
            for (auto &v : map)
            {
                std::cout << v.first << "\t" << v.second.first << "\t\t";
                for (auto &u : v.second.second) std::cout << u << " ";
                std::cout << std::endl;
            }
#endif
        }

        DijkstraResult Dijkstra(char previous_node, char current_node, char end_node = '@', bool escape = false)
        {
            std::map<char, std::pair<float, std::vector<char>>>                                                                    result;
            std::priority_queue<std::pair<float, char>, std::vector<std::pair<float, char>>, std::greater<std::pair<float, char>>> queue;
            const auto &start_vertex = this->operator[](current_node);

            for (auto &vertex : nodes)
            {
                if (vertex.name == start_vertex.name) { result[vertex.name] = std::make_pair(0, std::vector<char>{}); }
                else { result[vertex.name] = std::make_pair(std::numeric_limits<float>::infinity(), std::vector<char>{}); }
            }

            queue.push(std::make_pair(0, start_vertex.name));

            while (!queue.empty())
            {
                auto [distance, vertex_id] = queue.top();
                queue.pop();

                for (unsigned long index = 0; index < this->operator[](vertex_id).edges.size(); ++index)
                {
                    auto &edge = this->operator[](vertex_id).edges[index];

                    // if the checked vertex is the current node check if the previous node is in the list of previous nodes
                    if (vertex_id == current_node &&
                        std::find(edge.prev_nodes.begin(), edge.prev_nodes.end(), previous_node) == edge.prev_nodes.end())
                    {
                        continue;
                    }
                    // else if the checked vertex is not the current node check if it's predecessor along the path is in the list of previous
                    // nodes
                    else if (vertex_id != current_node &&
                             std::find(edge.prev_nodes.begin(), edge.prev_nodes.end(), result[vertex_id].second.back()) == edge.prev_nodes.end())
                    {
                        continue;
                    }

                    char  neighbor = edge.to;
                    float weight   = edge.get_weight();

                    if (result[neighbor].first > distance + weight)
                    {
                        result[neighbor].first  = distance + weight;
                        result[neighbor].second = result[vertex_id].second;
                        result[neighbor].second.push_back(vertex_id);
                        queue.push(std::make_pair(result[neighbor].first, neighbor));
                    }
                }
            }

            // insert starting to the back of vector the uuid of the specified vertex
            for (auto &[vertex_id, pair] : result)
            {
                if (end_node == '@' && !escape && !Edge::flood && !Edge::finished)
                {
                    if (Edge::stolen_gates[static_cast<int>(vertex_id - 'A')] > 1 && pair.first != 0) { pair.first += WEIGHT_PENALTY / 5.0f; }
                    else if (Edge::stolen_gates[static_cast<int>(vertex_id - 'A')] > 0 && pair.first != 0) { pair.first += WEIGHT_PENALTY / 10.0f; }

                    if (vertex_id == 'Q') { pair.first += 3.0f * WEIGHT_PENALTY / 4.0f; }
                }
                pair.second.push_back(vertex_id);
            }

            if (end_node == '@')
            {
                // in the result find the node with the smallest distance which is not in collected_nodes
                float min_distance = std::numeric_limits<float>::infinity();
                char  min_node     = '@';
                for (auto &[vertex_id, pair] : result)
                {
                    if (pair.first != 0 && pair.first < min_distance &&
                        (escape || (std::find(collected_nodes.begin(), collected_nodes.end(), vertex_id) == collected_nodes.end() &&
                                    std::find(std::begin(GATE_NAMES), std::end(GATE_NAMES), vertex_id) != std::end(GATE_NAMES))))
                    {
                        min_distance = pair.first;
                        min_node     = vertex_id;
                    }
                }
                return DijkstraResult{min_node, result[min_node].second, min_distance};
            }
            else
            {
                // print_dijkstra(result);
                // std::cout << previous_node << " " << current_node << " " << end_node << std::endl;
                return DijkstraResult{end_node, result[end_node].second, result[end_node].first};
            }
        }
    };

}  // namespace jlb

#endif  // GRAPH_HXX
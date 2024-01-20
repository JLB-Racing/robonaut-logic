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
            if (!flood && to == 'X') { weight = std::numeric_limits<float>::infinity(); }

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
                    weight += WEIGHT_PENALTY;
                    break;
                }
            }

            return weight;
        }

        static bool pirate_intersecting(const char node_) { return node_ == pirate_next_node || node_ == pirate_after_next_node; }

        static Edge invalid_edge;
    };

    char  Edge::pirate_previous_node          = 'P';
    char  Edge::pirate_next_node              = 'M';
    char  Edge::pirate_after_next_node        = 'H';
    float Edge::pirate_section_percentage     = 0.0f;
    int   Edge::stolen_gates[NUMBER_OF_GATES] = {0};
    bool  Edge::flood                         = false;

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

        Graph()
        {
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

            const auto UNIT           = SQUARE_LENGTH;
            const auto QUARTER_CIRCLE = 2 * UNIT * M_PI / 4.0f;

            this->operator[]('A').add_edge('C', Direction::LEFT, {'B', 'D'}, QUARTER_CIRCLE);
            this->operator[]('A').add_edge('B', Direction::STRAIGHT, {'C'}, 2.0f * UNIT);
            this->operator[]('A').add_edge('D', Direction::RIGHT, {'C'}, QUARTER_CIRCLE);
            this->operator[]('B').add_edge('A', Direction::STRAIGHT, {'E'}, 2.0f * UNIT);
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
            this->operator[]('E').add_edge('J', Direction::STRAIGHT, {'B'}, 2.0f * UNIT);
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
            this->operator[]('H').add_edge('M', Direction::STRAIGHT, {'F'}, 2.0f * UNIT);
            this->operator[]('I').add_edge('F', Direction::LEFT, {'K', 'L', 'N'}, QUARTER_CIRCLE);
            this->operator[]('I').add_edge('D', Direction::STRAIGHT, {'K', 'L', 'N'}, 2.0f * UNIT);
            this->operator[]('I').add_edge('G', Direction::RIGHT, {'K', 'L', 'N'}, QUARTER_CIRCLE);
            this->operator[]('I').add_edge('L', Direction::LEFT, {'D', 'F', 'G'}, QUARTER_CIRCLE);
            this->operator[]('I').add_edge('N', Direction::STRAIGHT, {'D', 'F', 'G'}, 2.0f * UNIT);
            this->operator[]('I').add_edge('K', Direction::RIGHT, {'D', 'F', 'G'}, QUARTER_CIRCLE);
            this->operator[]('J').add_edge('G', Direction::LEFT, {'L'}, QUARTER_CIRCLE);
            this->operator[]('J').add_edge('E', Direction::STRAIGHT, {'L'}, 2.0f * UNIT);
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
            this->operator[]('M').add_edge('H', Direction::STRAIGHT, {'P', 'Q', 'R'}, 2.0f * UNIT);
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
            // this->operator[]('P').add_edge('Q', Direction::STRAIGHT, {'P'}, UNIT);
            // this->operator[]('Q').add_edge('P', Direction::STRAIGHT, {'R'}, UNIT);
            this->operator[]('Q').add_edge('M', Direction::STRAIGHT, {'V', 'X'}, UNIT);
            // this->operator[]('Q').add_edge('R', Direction::STRAIGHT, {'P'}, UNIT);
            this->operator[]('Q').add_edge('V', Direction::LEFT, {'M'}, QUARTER_CIRCLE);
            this->operator[]('Q').add_edge('X', Direction::STRAIGHT, {'M'}, 2.5f * UNIT + QUARTER_CIRCLE);
            // this->operator[]('R').add_edge('Q', Direction::STRAIGHT, {'N', 'S'}, UNIT);
            this->operator[]('R').add_edge('M', Direction::RIGHT, {'N', 'S'}, QUARTER_CIRCLE);
            this->operator[]('R').add_edge('N', Direction::LEFT, {'M', 'Q'}, QUARTER_CIRCLE);
            this->operator[]('R').add_edge('S', Direction::STRAIGHT, {'M', 'Q'}, UNIT);
            this->operator[]('S').add_edge('R', Direction::STRAIGHT, {'T'}, UNIT);
            this->operator[]('S').add_edge('N', Direction::STRAIGHT, {'V', 'W'}, UNIT);
            this->operator[]('S').add_edge('T', Direction::STRAIGHT, {'R'}, UNIT);
            this->operator[]('S').add_edge('W', Direction::LEFT, {'N'}, QUARTER_CIRCLE);
            this->operator[]('S').add_edge('V', Direction::RIGHT, {'N'}, QUARTER_CIRCLE);
            this->operator[]('T').add_edge('S', Direction::STRAIGHT, {'O', 'U'}, UNIT);
            this->operator[]('T').add_edge('N', Direction::RIGHT, {'O', 'U'}, QUARTER_CIRCLE);
            this->operator[]('T').add_edge('O', Direction::LEFT, {'N', 'S'}, QUARTER_CIRCLE);
            // this->operator[]('T').add_edge('U', Direction::STRAIGHT, {'N', 'S'}, 2.0f * UNIT);
            this->operator[]('U').add_edge('T', Direction::STRAIGHT, {'U'}, 2.0f * UNIT);
            this->operator[]('U').add_edge('O', Direction::RIGHT, {'U'}, QUARTER_CIRCLE);
            this->operator[]('V').add_edge('Q', Direction::RIGHT, {'S', 'W'}, QUARTER_CIRCLE);
            this->operator[]('V').add_edge('S', Direction::LEFT, {'Q'}, QUARTER_CIRCLE);
            this->operator[]('V').add_edge('W', Direction::STRAIGHT, {'Q'}, 2.0f * UNIT);
            this->operator[]('W').add_edge('V', Direction::STRAIGHT, {'O'}, 2.0f * UNIT);
            this->operator[]('W').add_edge('S', Direction::RIGHT, {'O'}, QUARTER_CIRCLE);
            this->operator[]('W').add_edge('O', Direction::STRAIGHT, {'S', 'V'}, UNIT + QUARTER_CIRCLE);
            this->operator[]('X').add_edge('Q', Direction::STRAIGHT, {'X'}, 2.5f * UNIT + QUARTER_CIRCLE);
        }

        ~Graph() {}

        Node &operator[](char name)
        {
            if (nodes.empty() || name < 'A' || name > 'X') { return invalid_node; }
            return nodes[static_cast<int>(name - 'A')];
        }

        const Node &operator[](char name) const
        {
            if (nodes.empty() || name < 'A' || name > 'X') { return invalid_node; }
            return nodes[static_cast<int>(name - 'A')];
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
                if (end_node == '@' && !escape)
                {
                    if (Edge::stolen_gates[static_cast<int>(vertex_id - 'A')] > 1 && pair.first != 0) { pair.first += WEIGHT_PENALTY / 5.0f; }
                    else if (Edge::stolen_gates[static_cast<int>(vertex_id - 'A')] > 0 && pair.first != 0) { pair.first += WEIGHT_PENALTY / 10.0f; }
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
            else { return DijkstraResult{end_node, result[end_node].second, result[end_node].first}; }
        }
    };

}  // namespace jlb

#endif  // GRAPH_HXX

#ifndef GRAPH_HXX
#define GRAPH_HXX

namespace jlb
{
    struct Edge
    {
        char              node;
        Direction         direction;
        std::vector<char> prev_nodes;
        float             weight;
    };

    class Node
    {
    public:
        char              name;
        float             x;
        float             y;
        std::vector<Edge> edges;

        Node(char name_, float x_, float y_) : name{name_}, x(x_), y(y_) {}
        ~Node() {}

        void add_edge(char name_, Direction direction_, std::vector<char> prev_nodes_, float weight_ = 0.0f) { edges.push_back(Edge{name_, direction_, prev_nodes_, weight_}); }
    };

    class Graph
    {
    public:
        std::vector<Node> nodes;

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
            this->operator[]('M').add_edge('P', Direction::RIGHT, {'H', 'K'}, QUARTER_CIRCLE);
            this->operator[]('N').add_edge('K', Direction::LEFT, {'R', 'S', 'T'}, QUARTER_CIRCLE);
            this->operator[]('N').add_edge('I', Direction::STRAIGHT, {'R', 'S', 'T'}, 2.0f * UNIT);
            this->operator[]('N').add_edge('L', Direction::RIGHT, {'R', 'S', 'T'}, QUARTER_CIRCLE);
            this->operator[]('N').add_edge('T', Direction::LEFT, {'K', 'I', 'L'}, QUARTER_CIRCLE);
            this->operator[]('N').add_edge('S', Direction::STRAIGHT, {'K', 'I', 'L'}, UNIT);
            this->operator[]('N').add_edge('R', Direction::RIGHT, {'K', 'I', 'L'}, QUARTER_CIRCLE);
            this->operator[]('O').add_edge('L', Direction::LEFT, {'T', 'U', 'W'}, QUARTER_CIRCLE);
            this->operator[]('O').add_edge('U', Direction::LEFT, {'L'}, QUARTER_CIRCLE);
            this->operator[]('O').add_edge('W', Direction::STRAIGHT, {'L'}, UNIT + QUARTER_CIRCLE);
            this->operator[]('O').add_edge('T', Direction::RIGHT, {'L'}, QUARTER_CIRCLE);
            this->operator[]('P').add_edge('M', Direction::LEFT, {'P'}, QUARTER_CIRCLE);
            this->operator[]('P').add_edge('Q', Direction::STRAIGHT, {'P'}, UNIT);
            this->operator[]('Q').add_edge('P', Direction::STRAIGHT, {'R'}, UNIT);
            this->operator[]('Q').add_edge('M', Direction::STRAIGHT, {'V', 'X'}, UNIT);
            this->operator[]('Q').add_edge('R', Direction::STRAIGHT, {'P'}, UNIT);
            this->operator[]('Q').add_edge('V', Direction::LEFT, {'M'}, QUARTER_CIRCLE);
            this->operator[]('Q').add_edge('X', Direction::STRAIGHT, {'M'}, 2.5f * UNIT + QUARTER_CIRCLE);
            this->operator[]('R').add_edge('Q', Direction::STRAIGHT, {'N', 'S'}, UNIT);
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
            this->operator[]('T').add_edge('U', Direction::STRAIGHT, {'N', 'S'}, 2.0f * UNIT);
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
#ifdef SIMULATION
            if (nodes.empty()) throw std::runtime_error("Graph is empty");
            if (name < 'A' || name > 'X') throw std::runtime_error("Invalid node name");
#endif
            return nodes[static_cast<int>(name - 'A')];
        }
    };
}  // namespace jlb

#endif  // GRAPH_HXX

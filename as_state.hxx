#ifndef AS_STATE_HXX
#define AS_STATE_HXX

namespace jlb
{

    class ASState
    {
    public:
        bool under_gate = false;
        bool at_cross_section = false;
        bool prev_at_decision_point = false;

        [[maybe_unused]] char previous_node = 'U';
        [[maybe_unused]] char next_node = 'U';

    private:
    };

} // namespace jlb

#endif // AS_STATE_HXX
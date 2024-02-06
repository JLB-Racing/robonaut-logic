#ifndef TYPES_HXX
#define TYPES_HXX

#include <cmath>

namespace jlb
{

    ///////////////////////////////////////////////////////////////////////////
    //
    //      ENUMS
    //

    enum class Direction
    {
        LEFT,
        RIGHT,
        STRAIGHT,
    };

    enum class Mission
    {
        STANDBY,
        LABYRINTH,
        FAST,
    };

    enum class LabyrinthState
    {
        // TODO: this is just placeholder
        START,
        EXPLORING,
        STANDBY,
        ESCAPE,
        REVERSE_ESCAPE,
        FLOOD_TO_BALANCER,
        FLOOD_SOLVING,
        FLOOD_TO_LABYRINTH,
        FINISHED,
        MISSION_SWITCH,
        ERROR,
    };

    enum class MissionSwitchState
    {
        STANDBY,
        FIRST_FORWARD,
        FIRST_TURN,
        SECOND_TURN,
        SECOND_FORWARD,
    };

    enum class FastState
    {
        FIRST_FAST,
        FIRST_SLOW,
        SECOND_FAST,
        SECOND_SLOW,
        THIRD_FAST,
        THIRD_SLOW,
        FOURTH_FAST,
        FOURTH_SLOW,
        IN_ACCEL_ZONE,
        OUT_ACCEL_ZONE,
        IN_BRAKE_ZONE,
        OUT_BRAKE_ZONE,
    };

    //
    //      END ENUMS
    //
    ///////////////////////////////////////////////////////////////////////////

    typedef std::pair<char, char>         pcc;
    typedef std::pair<pcc, pcc>           cross;

}  // namespace jlb

#endif  // TYPES_HXX

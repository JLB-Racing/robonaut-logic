#ifndef TYPES_HXX
#define TYPES_HXX

#include <cmath>

#define PARAM static constexpr

#ifndef px_to_m
#define px_to_m(px) (px * (jlb::SQUARE_LENGTH * 2.0f) / jlb::BITMAP_SIZE)
#endif
#ifndef m_to_px
#define m_to_px(m) (m * jlb::BITMAP_SIZE / (jlb::SQUARE_LENGTH * 2.0f))
#endif

#ifndef rad2deg
#define rad2deg(rad) (rad * 180.0f / static_cast<float>(M_PI))
#endif
#ifndef deg2rad
#define deg2rad(deg) (deg * static_cast<float>(M_PI) / 180.0f)
#endif

typedef std::pair<char, char> pcc;
typedef std::pair<pcc, pcc>   cross;

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
		FOLLOW_SAFETY_CAR,
		OVERTAKE_SAFETY_CAR_START,
		OVERTAKE_SAFETY_CAR_END,
		IN_ACCEL_ZONE,
		OUT_ACCEL_ZONE,
		IN_BRAKE_ZONE,
		OUT_BRAKE_ZONE,
	};

#ifdef SIMULATION
    std::ostream &operator<<(std::ostream &os, const Direction &direction)
    {
        switch (direction)
        {
            case Direction::LEFT:
                os << "LEFT";
                break;
            case Direction::RIGHT:
                os << "RIGHT";
                break;
            case Direction::STRAIGHT:
                os << "STRAIGHT";
                break;
            default:
                os << "UNKNOWN";
                break;
        }
        return os;
    }

    std::ostream &operator<<(std::ostream &os, const Mission &mission)
    {
        switch (mission)
        {
            case Mission::STANDBY:
                os << "STANDBY";
                break;
            case Mission::LABYRINTH:
                os << "LABYRINTH";
                break;
            case Mission::FAST:
                os << "FAST";
                break;
            default:
                os << "UNKNOWN";
                break;
        }
        return os;
    }

    std::ostream &operator<<(std::ostream &os, const LabyrinthState &state)
	{
		switch (state)
		{
			case LabyrinthState::START:
				os << "START";
				break;
			case LabyrinthState::EXPLORING:
				os << "EXPLORING";
				break;
			case LabyrinthState::STANDBY:
				os << "STANDBY";
				break;
			case LabyrinthState::ESCAPE:
				os << "ESCAPE";
				break;
			case LabyrinthState::REVERSE_ESCAPE:
				os << "REVERSE_ESCAPE";
				break;
			case LabyrinthState::FLOOD_TO_BALANCER:
				os << "FLOOD_TO_BALANCER";
				break;
			case LabyrinthState::FLOOD_SOLVING:
				os << "FLOOD_SOLVING";
				break;
			case LabyrinthState::FLOOD_TO_LABYRINTH:
				os << "FLOOD_TO_LABYRINTH";
				break;
			case LabyrinthState::FINISHED:
				os << "FINISHED";
				break;
			case LabyrinthState::MISSION_SWITCH:
				os << "MISSION_SWITCH";
				break;
			case LabyrinthState::ERROR:
				os << "ERROR";
				break;
			default:
				os << "UNKNOWN";
				break;
		}
		return os;
	}

    std::ostream &operator<<(std::ostream &os, const MissionSwitchState &state)
	{
		switch (state)
		{
			case MissionSwitchState::STANDBY:
				os << "STANDBY";
				break;
			case MissionSwitchState::FIRST_FORWARD:
				os << "FIRST_FORWARD";
				break;
			case MissionSwitchState::FIRST_TURN:
				os << "FIRST_TURN";
				break;
			case MissionSwitchState::SECOND_TURN:
				os << "SECOND_TURN";
				break;
			case MissionSwitchState::SECOND_FORWARD:
				os << "SECOND_FORWARD";
				break;
			default:
				os << "UNKNOWN";
				break;
		}
		return os;
	}

    std::ostream &operator<<(std::ostream &os, const FastState &state)
        {
            switch (state)
            {
                case FastState::FOLLOW_SAFETY_CAR:
                    os << "FOLLOW_SAFETY_CAR";
                    break;
                case FastState::OVERTAKE_SAFETY_CAR_START:
                    os << "OVERTAKE_SAFETY_CAR_START";
                    break;
                case FastState::OVERTAKE_SAFETY_CAR_END:
                    os << "OVERTAKE_SAFETY_CAR_END";
                    break;
                case FastState::IN_ACCEL_ZONE:
                    os << "IN_ACCEL_ZONE";
                    break;
                case FastState::OUT_ACCEL_ZONE:
                    os << "OUT_ACCEL_ZONE";
                    break;
                case FastState::IN_BRAKE_ZONE:
                    os << "IN_BRAKE_ZONE";
                    break;
                case FastState::OUT_BRAKE_ZONE:
                    os << "OUT_BRAKE_ZONE";
                    break;
                default:
                    os << "UNKNOWN";
                    break;
            }
            return os;
        }
#endif

    //
    //      END ENUMS
    //
    ///////////////////////////////////////////////////////////////////////////

}  // namespace jlb

#endif  // TYPES_HXX

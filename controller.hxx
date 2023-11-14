#ifndef CONTROLLER_HXX
#define CONTROLLER_HXX

#include "common.hxx"

#include <chrono>

namespace jlb
{

    struct ControlSignal
    {
        float target_angle;
        float target_speed;
    };

    class Controller
    {
    public:
        unsigned long selected_front = SENSOR_COUNT / 2;
        unsigned long selected_rear = SENSOR_COUNT / 2;
        [[maybe_unused]] float line_position_front = 0.0f;
        [[maybe_unused]] float line_position_rear = 0.0f;
        [[maybe_unused]] float prev_line_position_front = 0.0f;
        [[maybe_unused]] float prev_line_position_rear = 0.0f;

        float target_angle = 0.0f;
        float target_speed = 0.0f;

        bool detection_front[SENSOR_COUNT];
        bool detection_rear[SENSOR_COUNT];
        std::vector<float> line_positions_front;
        std::vector<float> line_positions_rear;

        Direction direction = Direction::STRAIGHT;
        Mission mission = Mission::LABYRINTH;

        Controller(Direction direction_ = Direction::STRAIGHT) : direction{direction_} {}
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
            return kAng * heading_error + atan2(kDist * cross_track_error, kSoft + kDamp * current_velocity);
        }

        unsigned long select_control_point(bool *detection)
        {
            unsigned long selected;

            unsigned long rightmost = 0;
            for (unsigned long i = 0; i < SENSOR_COUNT; i++)
                if (!detection[i] && i > rightmost)
                    rightmost = i;

            unsigned long leftmost = SENSOR_COUNT;
            for (unsigned long i = 0; i < SENSOR_COUNT; i++)
                if (!detection[i] && i < leftmost)
                    leftmost = i;

            unsigned long center = leftmost;
            for (unsigned long i = leftmost; i <= rightmost; i++)
                if (!detection[i] && std::abs(static_cast<int>(i - (rightmost + leftmost) / 2)) < std::abs(static_cast<int>(center - (rightmost + leftmost) / 2)))
                    center = i;

            if (direction == Direction::LEFT || direction == Direction::REVERSE_LEFT)
                selected = leftmost;
            if (direction == Direction::RIGHT || direction == Direction::REVERSE_RIGHT)
                selected = rightmost;
            if (direction == Direction::STRAIGHT || direction == Direction::REVERSE_STRAIGHT)
                selected = center;

            return selected;
        }

        // float select_control_point(std::vector<float> line_positions, float prev_line_position)
        // {
        //     // sort by ascending order
        //     std::sort(line_positions.begin(), line_positions.end());

        //     if (line_positions.size() == 1)
        //     {
        //         return line_positions[0];
        //     }
        //     else if (line_positions.size() == 2)
        //     {
        //         switch (direction)
        //         {
        //         case Direction::STRAIGHT:
        //         {
        //             return std::fabs(line_positions[0] - prev_line_position) < std::fabs(line_positions[1] - prev_line_position) ? line_positions[0] : line_positions[1];
        //         }
        //         case Direction::LEFT:
        //         {
        //             return line_positions[0];
        //         }
        //         case Direction::RIGHT:
        //         {
        //             return line_positions[1];
        //         }
        //         default:
        //             return 0.0f;
        //         }
        //     }
        //     else if (line_positions.size() == 3)
        //     {
        //         switch (direction)
        //         {
        //         case Direction::STRAIGHT:
        //         {
        //             // return the middle one
        //             return line_positions[1];
        //         }
        //         case Direction::LEFT:
        //         {
        //             return line_positions[0];
        //         }
        //         case Direction::RIGHT:
        //         {
        //             return line_positions[2];
        //         }
        //         default:
        //             return 0.0f;
        //         }
        //     }
        //     else if (line_positions.size() == 4)
        //     {
        //         switch (direction)
        //         {
        //         case Direction::STRAIGHT:
        //         {
        //             // return the middle one
        //             return line_positions[1] + line_positions[2] / 2.0f;
        //         }
        //         case Direction::LEFT:
        //         {
        //             return line_positions[0];
        //         }
        //         case Direction::RIGHT:
        //         {
        //             return line_positions[3];
        //         }
        //         default:
        //             return 0.0f;
        //         }
        //     }
        //     else
        //     {
        //         // this should never happen

        //         return 0.0f;
        //     }
        // }

        void lateral_control()
        {
            if (std::all_of(std::begin(detection_front), std::end(detection_front), [](bool b)
                            { return b; }))
            {
                return;
            }

            if (std::all_of(std::begin(detection_rear), std::end(detection_rear), [](bool b)
                            { return b; }))
            {
                return;
            }

            // if (line_positions_front.size() == 0 || line_positions_rear.size() == 0 || line_positions_front.size() > 4 || line_positions_rear.size() > 4)
            // {
            //     return;
            // }

#ifdef STM32
            // TODO: add timestamp
#else
            auto control_timestamp_ = std::chrono::steady_clock::now();
            [[maybe_unused]] float dt = std::chrono::duration_cast<std::chrono::milliseconds>(control_timestamp_ - prev_control_timestamp_).count() / 1000.0f;
            prev_control_timestamp_ = control_timestamp_;
#endif
            float sensor_rate = SENSOR_WIDTH / SENSOR_COUNT;

            unsigned long sensor_center = SENSOR_COUNT / 2.0f;
            selected_front = select_control_point(detection_front);
            selected_rear = select_control_point(detection_rear);
            line_position_front = (static_cast<float>(selected_front) - static_cast<float>(sensor_center) + 1) * sensor_rate;
            line_position_rear = (static_cast<float>(selected_rear) - static_cast<float>(sensor_center) + 1) * sensor_rate;

            // line_position_front = select_control_point(line_positions_front, prev_line_position_front);
            // line_position_rear = select_control_point(line_positions_rear, prev_line_position_rear);
            // prev_line_position_front = line_position_front;
            // prev_line_position_rear = line_position_rear;

            // selected_front = static_cast<unsigned long>(std::round(line_position_front / sensor_rate) + SENSOR_COUNT / 2.0f - 1);
            // selected_rear = static_cast<unsigned long>(std::round(line_position_rear / sensor_rate) + SENSOR_COUNT / 2.0f - 1);

            float heading_error = std::atan2(line_position_front - line_position_rear, SENSOR_BASE);
            float cross_track_error = line_position_front;

            if (USE_STANLEY)
            {
                target_angle = stanley(cross_track_error, heading_error);
            }
            else
            {
                target_angle = PID(cross_track_error, dt);
            }

            if (target_angle > MAX_WHEEL_ANGLE)
                target_angle = MAX_WHEEL_ANGLE;
            if (target_angle < -MAX_WHEEL_ANGLE)
                target_angle = -MAX_WHEEL_ANGLE;

            if (direction == Direction::REVERSE_LEFT || direction == Direction::REVERSE_RIGHT || direction == Direction::REVERSE_STRAIGHT)
            {
                target_angle = -target_angle;
            }

            prev_error = cross_track_error;
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

        ControlSignal update()
        {
            lateral_control();
            longitudinal_control();

            return {target_angle, target_speed};
        }

        void set_detection_front(bool *detection_front_, std::vector<float> line_positions_front_)
        {
            for (unsigned long i = 0; i < SENSOR_COUNT; i++)
                detection_front[i] = detection_front_[i];
            line_positions_front = line_positions_front_;
        }

        void set_detection_rear(bool *detection_rear_, std::vector<float> line_positions_rear_)
        {
            for (unsigned long i = 0; i < SENSOR_COUNT; i++)
                detection_rear[i] = detection_rear_[i];
            line_positions_rear = line_positions_rear_;
        }

        void set_current_velocity(const float current_velocity_)
        {
            current_velocity = current_velocity_;
        }

        void set_direction(const Direction direction_)
        {
            direction = direction_;
        }

        void set_mission(const Mission mission_)
        {
            mission = mission_;
        }

    private:
        float integral = 0.0f;
        float prev_error = 0.0f;
        float current_velocity = 0.0f;

#ifdef STM32
        // TODO: add timestamp
#else
        std::chrono::time_point<std::chrono::steady_clock>
            prev_control_timestamp_ = std::chrono::steady_clock::now();
#endif
    };

} // namespace jlb

#endif // CONTROLLER_HXX
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
        unsigned long selected = 0;
        float target_angle = 0.0f;
        float target_speed = 0.0f;
        bool detection[SENSOR_WIDTH];

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
            float kAng = 0.5f;
            float kDist = 0.5f;
            float kSoft = 1.0f;
            float kDamp = 1.0f;

            return kAng * heading_error + atan2(kDist * cross_track_error, kSoft + kDamp * current_velocity);
        }

        void lateral_control()
        {
            if (std::all_of(std::begin(detection), std::end(detection), [](bool b)
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

            unsigned long sensor_center = SENSOR_WIDTH / 2;

            unsigned long rightmost = 0;
            for (unsigned long i = 0; i < SENSOR_WIDTH; i++)
                if (!detection[i] && i > rightmost)
                    rightmost = i;

            unsigned long leftmost = SENSOR_WIDTH;
            for (unsigned long i = 0; i < SENSOR_WIDTH; i++)
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

            [[maybe_unused]] float angle_error = (static_cast<int>(selected - sensor_center + 1)) / static_cast<float>(sensor_center) * M_PI / 2.0f;
            float error = (static_cast<int>(selected - sensor_center + 1)) / static_cast<float>(sensor_center);
            target_angle = PID(error, dt);

            if (target_angle > MAX_WHEEL_ANGLE)
                target_angle = MAX_WHEEL_ANGLE;
            if (target_angle < -MAX_WHEEL_ANGLE)
                target_angle = -MAX_WHEEL_ANGLE;

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

        ControlSignal update()
        {
            lateral_control();
            longitudinal_control();

            return {target_angle, target_speed};
        }

        void set_current_velocity(const float current_velocity_)
        {
            current_velocity = current_velocity_;
        }

        void set_detection(bool *detection_)
        {
            for (unsigned long i = 0; i < SENSOR_WIDTH; i++)
                detection[i] = detection_[i];
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
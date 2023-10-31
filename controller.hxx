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
        float target_angle = 0.0f;
        float target_speed = 0.0f;
        bool detection_front[SENSOR_COUNT];
        bool detection_rear[SENSOR_COUNT];

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

            unsigned long selected;

            if (direction == Direction::LEFT || direction == Direction::REVERSE_LEFT)
                selected = leftmost;
            if (direction == Direction::RIGHT || direction == Direction::REVERSE_RIGHT)
                selected = rightmost;
            if (direction == Direction::STRAIGHT || direction == Direction::REVERSE_STRAIGHT)
                selected = center;

            return selected;
        }

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

#ifdef STM32
            // TODO: add timestamp
#else
            auto control_timestamp_ = std::chrono::steady_clock::now();
            [[maybe_unused]] float dt = std::chrono::duration_cast<std::chrono::milliseconds>(control_timestamp_ - prev_control_timestamp_).count() / 1000.0f;
            prev_control_timestamp_ = control_timestamp_;
#endif

            unsigned long sensor_center = SENSOR_COUNT / 2.0f;
            selected_front = select_control_point(detection_front);
            selected_rear = select_control_point(detection_rear);

            float sensor_rate = SENSOR_WIDTH / SENSOR_COUNT;
            float sensor_center_m = static_cast<float>(sensor_center) * sensor_rate;
            float selected_front_m = static_cast<float>(selected_front) * sensor_rate;
            float selected_rear_m = static_cast<float>(selected_rear) * sensor_rate;

            [[maybe_unused]] float heading_error = std::atan2(selected_front_m - selected_rear_m, SENSOR_BASE);

            float cross_track_error = (selected_front_m - sensor_center_m + sensor_rate) / sensor_center_m;
            // target_angle = PID(cross_track_error, dt);
            target_angle = stanley(cross_track_error, heading_error);

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

        void set_current_velocity(const float current_velocity_)
        {
            current_velocity = current_velocity_;
        }

        void set_detection_front(bool *detection_front_)
        {
            for (unsigned long i = 0; i < SENSOR_COUNT; i++)
                detection_front[i] = detection_front_[i];
        }

        void set_detection_rear(bool *detection_rear_)
        {
            for (unsigned long i = 0; i < SENSOR_COUNT; i++)
                detection_rear[i] = detection_rear_[i];
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
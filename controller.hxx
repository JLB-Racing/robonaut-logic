#ifndef CONTROLLER_HXX
#define CONTROLLER_HXX

#include <chrono>
#include <complex>
#include <vector>

#include "common.hxx"
#include "pid.hxx"

#ifndef SIMULATION
#include "Servo.h"

extern uint32_t usWidth_throttle;
#endif
namespace jlb
{

    struct ControlSignal
    {
        float target_angle;
        float target_speed;
    };

    struct ControlParams
    {
        float kP;
        float kDelta;
    };

    class Controller
    {
    public:
        unsigned long          selected_front           = SENSOR_COUNT / 2;
        unsigned long          selected_rear            = SENSOR_COUNT / 2;
        [[maybe_unused]] float line_position_front      = 0.0f;
        [[maybe_unused]] float line_position_rear       = 0.0f;
        [[maybe_unused]] float prev_line_position_front = 0.0f;
        [[maybe_unused]] float prev_line_position_rear  = 0.0f;

        float reference_speed   = 0.0f;
        float target_angle      = 0.0f;
        float target_speed      = 0.0f;
        float cross_track_error = 0.0f;
        float heading_error     = 0.0f;
        float dist_error_norm   = 0.0f;
        float ang_error_norm    = 0.0f;

        float              object_range = 100.0f;
        bool               detection_front[SENSOR_COUNT];
        bool               detection_rear[SENSOR_COUNT];
        std::vector<float> line_positions_front;
        std::vector<float> line_positions_rear;

        Direction direction      = Direction::STRAIGHT;
        Direction prev_direction = Direction::STRAIGHT;

        uint32_t tick_counter      = 0u;
        uint32_t tick_counter_prev = 0u;

        bool passed_half = false;

        Controller() : direction{} {}

        ~Controller() {}

        float select_control_point(std::vector<float> line_positions, float prev_line_position)
        {
            std::sort(line_positions.begin(), line_positions.end());

            if (line_positions.size() == 1) { return line_positions[0]; }
            else if (line_positions.size() == 2)
            {
                float tmp;
                switch (direction)
                {
                    case Direction::LEFT:
                    {
                        tmp = line_positions[0];
                        break;
                    }
                    case Direction::STRAIGHT:
                    {
                        if (direction == prev_direction)
                        {
                            tmp = std::fabs(line_positions[0] - prev_line_position) > std::fabs(line_positions[1] - prev_line_position)
                                      ? line_positions[0]
                                      : line_positions[1];
                        }
                        else
                        {
                            tmp = std::fabs(line_positions[0] - prev_line_position) < std::fabs(line_positions[1] - prev_line_position)
                                      ? line_positions[0]
                                      : line_positions[1];
                        }
                        break;
                    }
                    case Direction::RIGHT:
                    {
                        tmp = line_positions[1];
                        break;
                    }
                    default:
                    {
                        tmp = 0.0f;
                        break;
                    }
                }

                if (passed_half) { tmp = std::fabs(line_positions[0]) < std::fabs(line_positions[1]) ? line_positions[0] : line_positions[1]; }
                return tmp;
            }
            else if (line_positions.size() == 3)
            {
                switch (direction)
                {
                    case Direction::LEFT:
                    {
                        return line_positions[0];
                    }
                    case Direction::STRAIGHT:
                    {
                        return line_positions[1];
                    }
                    case Direction::RIGHT:
                    {
                        return line_positions[2];
                    }
                    default:
                        return 0.0f;
                }
            }
            else if (line_positions.size() == 4)
            {
                // switch (direction)
                // {
                //     case Direction::LEFT:
                //     {
                //         return line_positions[0];
                //     }
                //     case Direction::STRAIGHT:
                //     {
                //         return line_positions[1] + line_positions[2] / 2.0f;
                //     }
                //     case Direction::RIGHT:
                //     {
                //         return line_positions[3];
                //     }
                //     default:
                //         return 0.0f;
                // }

                return line_positions[1] + line_positions[2] / 2.0f;
            }
            else
            {
                // this should never happen

                return 0.0f;
            }
        }

        ControlParams get_control_params()
        {
            current_velocity += std::numeric_limits<float>::epsilon();

#ifndef SIMULATION
            float d5 = OFFSET_EXP1 + std::log2(current_velocity + OFFSET_EXP2);
#else
            float d5 = OFFSET + SLOPE * std::fabs(current_velocity);
#endif
            if (d5 < D5_MIN) d5 = D5_MIN;
            float               t5  = d5 / std::fabs(current_velocity);
            float               T   = t5 / 3.0f * DAMPING;
            float               wp  = (1.0f / T) * sqrt(1.0f - DAMPING * DAMPING);
            float               phi = acosf(DAMPING);
            float               x   = wp / tan(phi);
            std::complex<float> s1  = std::complex<float>(x, wp);
            std::complex<float> s2  = std::complex<float>(x, -wp);

            std::complex<float> kP     = -SENSOR_BASE / (std::fabs(current_velocity) * std::fabs(current_velocity)) * s1 * s2;
            std::complex<float> kDelta = -SENSOR_BASE / std::fabs(current_velocity) * ((s1 + s2) - std::fabs(current_velocity) * kP);

            return {kP.real(), kDelta.real()};
        }

        void lateral_control([[maybe_unused]] const float dt)
        {
            if (std::all_of(std::begin(detection_front), std::end(detection_front), [](bool b) { return b; }) ||
                std::all_of(std::begin(detection_rear), std::end(detection_rear), [](bool b) { return b; }) || line_positions_front.size() == 0 ||
                line_positions_rear.size() == 0)
            {
                // if (target_speed <= FAST_SPEED_TURN || target_speed <= LABYRINTH_SPEED || target_speed <= LABYRINTH_SPEED_REVERSE)
                // {
                //     if (target_angle < 0) { target_angle = -deg2rad(MAX_WHEEL_ANGLE); }
                //     else if (target_angle == 0) { target_angle = 0; }
                //     else if (target_angle > 0) { target_angle = deg2rad(MAX_WHEEL_ANGLE); }
                // }
                return;
            }

            if (line_positions_front.size() > 4 || line_positions_rear.size() > 4) { return; }

            line_position_front      = select_control_point(line_positions_front, prev_line_position_front);
            line_position_rear       = select_control_point(line_positions_rear, prev_line_position_rear);
            prev_line_position_front = line_position_front;
            prev_line_position_rear  = line_position_rear;

            float sensor_rate   = SENSOR_WIDTH / SENSOR_COUNT;
            float sensor_center = SENSOR_COUNT / 2.0f;
            selected_front      = static_cast<unsigned long>(line_position_front / sensor_rate + sensor_center);
            selected_rear       = static_cast<unsigned long>(line_position_rear / sensor_rate + sensor_center);

            cross_track_error = line_position_front;
            heading_error     = std::atan2(line_position_front - line_position_rear, SENSOR_BASE);

            auto [kP, kDelta] = get_control_params();
            target_angle      = -kP * cross_track_error - kDelta * heading_error;

            if (target_angle > deg2rad(MAX_WHEEL_ANGLE)) target_angle = deg2rad(MAX_WHEEL_ANGLE);
            if (target_angle < -deg2rad(MAX_WHEEL_ANGLE)) target_angle = -deg2rad(MAX_WHEEL_ANGLE);

            if (target_speed < 0.0f) { target_angle = -target_angle; }
        }

        void longitudinal_control([[maybe_unused]] const float dt, [[maybe_unused]] bool follow_car)
        {
            /*
            float dist_error = std::min(std::abs(cross_track_error), DIST_ERROR_MAX);
            float ang_error  = std::min(std::abs(heading_error), deg2rad(ANG_ERROR_MAX));

            dist_error_norm = dist_error / DIST_ERROR_MAX;
            ang_error_norm  = ang_error / deg2rad(ANG_ERROR_MAX);

            float x      = std::max(dist_error_norm, ang_error_norm);
            target_speed = std::min(reference_speed, reference_speed * (1.0f - (0.1666667f * x) - (0.8333333f * x * x)));
            */

#ifndef SIMULATION
            if (!((usWidth_throttle > 1800) && (usWidth_throttle < 2800))) { reference_speed = 0.0f; }
#endif

            if (reference_speed > target_speed + MAX_ACCELERATION * dt) { target_speed += MAX_ACCELERATION * dt; }
            else if (reference_speed < target_speed - MAX_DECELERATION * dt) { target_speed -= MAX_DECELERATION * dt; }
            else { target_speed = reference_speed; }

            if (follow_car)
            {
                float object_rate = object_pid.update(obj::FOLLOW_DISTANCE, object_range, dt);
                target_speed *= std::pow((1 - object_rate), 2);
            }
        }

        ControlSignal update(bool follow_car = false)
        {
#ifndef SIMULATION
            tick_counter_prev = tick_counter;
            tick_counter      = HAL_GetTick();
            float dt          = (((float)tick_counter) - ((float)(tick_counter_prev))) / 1000.0f;
#else
            auto                   control_timestamp_ = std::chrono::steady_clock::now();
            [[maybe_unused]] float dt =
                std::chrono::duration_cast<std::chrono::milliseconds>(control_timestamp_ - prev_control_timestamp_).count() / 1000.0f;
            prev_control_timestamp_ = control_timestamp_;
#endif

            lateral_control(dt);
            longitudinal_control(dt, follow_car);

            return {target_angle, target_speed};
        }

        ControlSignal update_mission_switch(bool follow_car = false)
        {
#ifndef SIMULATION
            tick_counter_prev = tick_counter;
            tick_counter      = HAL_GetTick();
            float dt          = (((float)tick_counter) - ((float)(tick_counter_prev))) / 1000.0f;
#else
            auto                   control_timestamp_ = std::chrono::steady_clock::now();
            [[maybe_unused]] float dt =
                std::chrono::duration_cast<std::chrono::milliseconds>(control_timestamp_ - prev_control_timestamp_).count() / 1000.0f;
            prev_control_timestamp_ = control_timestamp_;
#endif

            // TODO: mission switch
            lateral_control(dt);
            longitudinal_control(dt, follow_car);

            return {0, 0};
        }

        ControlSignal update_balancer()
        {
#ifndef SIMULATION
            tick_counter_prev = tick_counter;
            tick_counter      = HAL_GetTick();
            float dt          = (((float)tick_counter) - ((float)(tick_counter_prev))) / 1000.0f;
#else
            auto                   control_timestamp_ = std::chrono::steady_clock::now();
            [[maybe_unused]] float dt =
                std::chrono::duration_cast<std::chrono::milliseconds>(control_timestamp_ - prev_control_timestamp_).count() / 1000.0f;
            prev_control_timestamp_ = control_timestamp_;
#endif

            // TODO: balancer control

            lateral_control(dt);
            longitudinal_control(dt, false);

            return {target_angle, target_speed};
        }

        void set_object_range(const float object_range_) { object_range = object_range_; }

        void set_detection_front(bool *detection_front_, std::vector<float> line_positions_front_)
        {
            for (unsigned long i = 0; i < SENSOR_COUNT; i++) detection_front[i] = detection_front_[i];
            line_positions_front = line_positions_front_;
        }

        void set_detection_rear(bool *detection_rear_, std::vector<float> line_positions_rear_)
        {
            for (unsigned long i = 0; i < SENSOR_COUNT; i++) detection_rear[i] = detection_rear_[i];
            line_positions_rear = line_positions_rear_;
        }

        void swap_front_rear()
        {
            std::swap(detection_front, detection_rear);
            std::swap(line_positions_front, line_positions_rear);
            std::reverse(std::begin(detection_front), std::end(detection_front));
            std::reverse(std::begin(detection_rear), std::end(detection_rear));
            std::reverse(std::begin(line_positions_front), std::end(line_positions_front));
            std::reverse(std::begin(line_positions_rear), std::end(line_positions_rear));

            // use multiplication lambda with -1 on line positions
            std::transform(
                std::begin(line_positions_front), std::end(line_positions_front), std::begin(line_positions_front), [](float f) { return -f; });
            std::transform(
                std::begin(line_positions_rear), std::end(line_positions_rear), std::begin(line_positions_rear), [](float f) { return -f; });
        }

        void set_current_velocity(const float current_velocity_) { current_velocity = current_velocity_; }

        void set_direction(const Direction direction_, const bool reverse = false)
        {
            if (!reverse)
            {
                prev_direction = direction;
                direction      = direction_;
            }
            else
            {
                if (direction == Direction::LEFT) { prev_direction = Direction::RIGHT; }
                else if (direction == Direction::RIGHT) { prev_direction = Direction::LEFT; }
                else { prev_direction = Direction::STRAIGHT; }

                if (direction_ == Direction::LEFT) { direction = Direction::RIGHT; }
                else if (direction_ == Direction::RIGHT) { direction = Direction::LEFT; }
                else { direction = Direction::STRAIGHT; }
            }
        }

        void set_reference_speed(const float reference_speed_) { reference_speed = reference_speed_; }

        void set_passed_half(const bool passed_half_) { passed_half = passed_half_; }

    private:
        float current_velocity = 0.0f;

        PID object_pid{obj::kP, obj::kI, obj::kD, obj::TAU, obj::T, obj::LIM_MIN, obj::LIM_MAX, obj::DEADBAND, obj::DERIVATIVE_FILTER_ALPHA};

#ifdef SIMULATION
        std::chrono::time_point<std::chrono::steady_clock> prev_control_timestamp_ = std::chrono::steady_clock::now();
#endif
    };

}  // namespace jlb

#endif  // CONTROLLER_HXX

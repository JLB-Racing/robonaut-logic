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

        float target_speed      = 0.0f;
        float target_angle      = 0.0f;
        float cross_track_error = 0.0f;
        float heading_error     = 0.0f;
        float dist_error_norm   = 0.0f;
        float ang_error_norm    = 0.0f;

        float              object_range = 100.0f;
        bool               detection_front[SENSOR_COUNT];
        bool               detection_rear[SENSOR_COUNT];
        std::vector<float> line_positions_front;
        std::vector<float> line_positions_rear;
        float              distance_local = 0.0f;
        uint8_t current_lap = 0u;

        Direction direction      = START_DIRECTION;
        Direction prev_direction = START_DIRECTION;

        uint32_t tick_counter      = 0u;
        uint32_t tick_counter_prev = 0u;

        bool passed_half    = false;
        bool deadman_switch = false;

        Controller() {}

        ~Controller() {}

        void set_current_lap(uint8_t current_lap_)
        {
        	current_lap = current_lap_ < 6 ? current_lap_ : 6;
        }

        float select_control_point(std::vector<float> line_positions, float prev_line_position)
        {
            std::sort(line_positions.begin(), line_positions.end());

            if(line_positions.size() == 0) { return 0.0f; }
            else if (line_positions.size() == 1) { return line_positions[0]; }
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
                            tmp = std::fabs(line_positions[0] - prev_line_position) < std::fabs(line_positions[1] - prev_line_position)
                                      ? line_positions[0]
                                      : line_positions[1];
                        }
                        else
                        {
                            tmp = std::fabs(line_positions[0] - prev_line_position) > std::fabs(line_positions[1] - prev_line_position)
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

                //if (passed_half) { tmp = std::fabs(line_positions[0]) < std::fabs(line_positions[1]) ? line_positions[0] : line_positions[1]; }
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
                /*switch (direction)
                {
                    case Direction::LEFT:
                    {
                        return line_positions[0];
                    }
                    case Direction::STRAIGHT:
                    {
                        return line_positions[1] + line_positions[2] / 2.0f;
                    }
                    case Direction::RIGHT:
                    {
                        return line_positions[3];
                    }
                    default:
                        return 0.0f;
                }*/

                return line_positions[1] + line_positions[2] / 2.0f;
            }
            else if(line_positions.size() > 5)
            {
            	return 0.0f;
            }
            else
            {
                float sum = 0.0f;
                for (auto &i : line_positions) { sum += i; }
                return sum / line_positions.size();
            }
        }

        ControlParams get_control_params()
        {
            current_velocity += std::numeric_limits<float>::epsilon();

            float damping = DAMPING;

#ifndef SIMULATION
            float d5 = OFFSET_EXP1 + std::log2(std::fabs(current_velocity) + OFFSET_EXP2);
            if (target_speed < 0.0f)
            {
                d5      = D5_REVERSE;
                damping = DAMPING_REVERSE;
            }
#ifndef FAST_V0
            if ((current_velocity < (FAST_SPEED_TURN[current_lap] + LOW_SPEED_EPSILON)) || (current_velocity < (LABYRINTH_SPEED + LOW_SPEED_EPSILON)))
			{
				damping = DAMPING_TURN;
				d5      = D5_MIN;
			}
#else
            if ((current_velocity < (FAST_SPEED_TURN + LOW_SPEED_EPSILON)) || (current_velocity < (LABYRINTH_SPEED + LOW_SPEED_EPSILON)))
			{
				damping = DAMPING_TURN;
				d5      = D5_MIN;
			}
#endif
#else
            float d5 = OFFSET + SLOPE * std::fabs(current_velocity);
#endif
            if ((d5 < D5_MIN) || std::isnan(d5)) d5 = D5_MIN;
            float               t5  = d5 / std::fabs(current_velocity);
            float               T   = t5 / 3.0f * damping;
            float               wp  = (1.0f / T) * sqrt(1.0f - damping * damping);
            float               phi = acosf(damping);
            float               x   = wp / tan(phi);
            std::complex<float> s1  = std::complex<float>(x, wp);
            std::complex<float> s2  = std::complex<float>(x, -wp);

            std::complex<float> kP     = -SENSOR_BASE / (std::fabs(current_velocity) * std::fabs(current_velocity)) * s1 * s2;
            std::complex<float> kDelta = -SENSOR_BASE / std::fabs(current_velocity) * ((s1 + s2) - std::fabs(current_velocity) * kP);

            if (target_speed < 0.0f) { return {kP.real() * MULTIPLIER_REVERSE, kDelta.real()}; }
            return {kP.real(), kDelta.real()};
        }

        void lateral_control([[maybe_unused]] const float dt, [[maybe_unused]] bool state_space = false)
        {
            if (line_positions_front.size() != 0)
            {
                line_position_front      = select_control_point(line_positions_front, prev_line_position_front);
                prev_line_position_front = line_position_front;
            }
            else { line_position_front = prev_line_position_front; }

            if (line_positions_rear.size() != 0)
            {
                line_position_rear      = select_control_point(line_positions_rear, prev_line_position_rear);
                prev_line_position_rear = line_position_rear;
            }
            else { line_position_rear = prev_line_position_rear; }

            float sensor_rate   = SENSOR_WIDTH / SENSOR_COUNT;
            float sensor_center = SENSOR_COUNT / 2.0f;
            selected_front      = static_cast<unsigned long>(line_position_front / sensor_rate + sensor_center);
            selected_rear       = static_cast<unsigned long>(line_position_rear / sensor_rate + sensor_center);

            cross_track_error = line_position_front;

            heading_error = std::atan2(line_position_front - line_position_rear, SENSOR_BASE);

#ifndef SIMULATION
            if ((target_speed < 0.0f && current_velocity < 0.0f) || state_space)
            {
                auto [kP, kDelta] = get_control_params();
                target_angle      = -kP * cross_track_error - kDelta * heading_error;
            }
            else { target_angle = -lateral_pid.update(0, cross_track_error, dt); }
#else
            auto [kP, kDelta] = get_control_params();
            target_angle      = -kP * cross_track_error - kDelta * heading_error;
#endif

            if (target_angle > deg2rad(MAX_WHEEL_ANGLE)) target_angle = deg2rad(MAX_WHEEL_ANGLE);
            if (target_angle < -deg2rad(MAX_WHEEL_ANGLE)) target_angle = -deg2rad(MAX_WHEEL_ANGLE);

            if (target_speed < 0.0f && current_velocity < 0.0f) { target_angle = -target_angle; }
        }

        void longitudinal_control([[maybe_unused]] const float dt, [[maybe_unused]] bool follow_car)
        {
            if (follow_car && target_speed >= 0 && object_range < SAFETY_CAR_THRESHOLD && target_speed > FAST_SPEED_SAFETY_CAR)
            {
                target_speed = FAST_SPEED_SAFETY_CAR;
            }

#ifndef SIMULATION
            if (((usWidth_throttle > 1600) && (usWidth_throttle < 2800))) { deadman_switch = true; }
            else { deadman_switch = false; }
#endif

            if (follow_car && target_speed >= 0.0f)
            {
                // target_speed_prev = target_speed;
                object_pid.update_limits(0.0f, target_speed);
                float safety_car_speed = object_pid.update(object_range, obj::FOLLOW_DISTANCE, dt);
                target_speed           = safety_car_speed;
            }
        }

        ControlSignal update(bool follow_car = false, bool state_space = false)
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

            lateral_control(dt, state_space);
            longitudinal_control(dt, follow_car);

            return {target_angle, target_speed};
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

        void set_distance_local(const float distance_local_) { distance_local = distance_local_; }

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

        void set_target_speed(const float target_speed_) { target_speed = target_speed_; }

        void set_passed_half(const bool passed_half_) { passed_half = passed_half_; }

    private:
        float current_velocity = 0.0f;
        PID   object_pid{obj::kP, obj::kI, obj::kD, obj::TAU, obj::T, obj::LIM_MIN, obj::LIM_MAX, obj::DEADBAND, obj::DERIVATIVE_FILTER_ALPHA};
        PID   lateral_pid{lat::kP, lat::kI, lat::kD, lat::TAU, lat::T, lat::LIM_MIN, lat::LIM_MAX, lat::DEADBAND, lat::DERIVATIVE_FILTER_ALPHA};

#ifdef SIMULATION
        std::chrono::time_point<std::chrono::steady_clock> prev_control_timestamp_ = std::chrono::steady_clock::now();
#endif
    };

}  // namespace jlb

#endif  // CONTROLLER_HXX

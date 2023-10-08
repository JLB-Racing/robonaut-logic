#ifndef ODOMETRY_HXX
#define ODOMETRY_HXX

#include "common.hxx"

#include <deque>
#include <chrono>
#include <cmath>

#ifdef STM32
// TODO: check if this is necessary
#include "FreeRTOS.h"
#include "task.h"
#else
#include <thread>
#include <mutex>
#endif

namespace jlb
{
    struct Odom
    {
        float vx = 0.0f;
        float x = 0.0f;
        float y = 0.0f;
        float theta = 0.0f;
    };

    class Odometry
    {
    public:
        float vx_t = 0.0f;    // linear velocity
        float w_t = 0.0f;     // angular velocity
        float x_t = 0.0f;     // x position
        float y_t = 0.0f;     // y position
        float theta_t = 0.0f; // orientation

        float meas_motor_rpm = 0.0f;
        float meas_ang_vel_z = 0.0f;

        Odometry(const float x_t_ = 0.0f, const float y_t_ = 0.0f, const float theta_t_ = 0.0f)
            : x_t(x_t_), y_t(y_t_), theta_t(normalize_angle(theta_t_)) {}

        ~Odometry() {}

        void rpm_callback(const float motor_rpm)
        {
            meas_motor_rpm = motor_rpm;

            float wheel_rpm = motor_rpm * jlb::GEAR_RATIO_MOTOR_TO_WHEEL;
            float velocity = M_PI * jlb::WHEEL_DIAMETER * wheel_rpm / 60.0f;

            if (std::fabs(velocity) > jlb::MAX_VELOCITY)
            {
                return;
            }

            v_buffer_.push_back(velocity);
            if (v_buffer_.size() > jlb::VELOCITY_BUFFER_SIZE)
            {
                v_buffer_.pop_front();
            }

            vx_t = std::accumulate(v_buffer_.begin(), v_buffer_.end(), 0.0f) / v_buffer_.size();
        }

        void imu_callback(const float ang_vel_z)
        {
            meas_ang_vel_z = ang_vel_z;

            if (std::fabs(ang_vel_z) > jlb::MAX_YAW_RATE)
            {
                return;
            }

            w_buffer_.push_back(ang_vel_z);
            if (w_buffer_.size() > jlb::IMU_BUFFER_SIZE)
            {
                w_buffer_.pop_front();
            }

            w_t = std::accumulate(w_buffer_.begin(), w_buffer_.end(), 0.0f) / w_buffer_.size();
        }

        Odom update_odom()
        {
#ifdef STM32
            // TODO: add timestamp and dt
#else
            if (first_update_)
            {
                odom_timestamp_ = std::chrono::steady_clock::now();
                first_update_ = false;
            }

            auto update_timestamp = std::chrono::steady_clock::now();
            float dt = std::chrono::duration_cast<std::chrono::milliseconds>(update_timestamp - odom_timestamp_).count() / 1000.0f;
#endif

            if (dt > 0.0f)
            {
                vx_t = std::fabs(vx_t) < 0.03 ? 0.0 : vx_t;
                w_t = std::fabs(w_t) < 0.015 ? 0.0 : w_t;
                // float vy_t = WHEELBASE * w_t / 2.0f;
                float vy_t = 0.0f;

                x_t += (vx_t * std::cos(theta_t) - vy_t * std::sin(theta_t)) * dt;
                y_t += (vx_t * std::sin(theta_t) + vy_t * std::cos(theta_t)) * dt;
                theta_t = normalize_angle(theta_t + w_t * dt);
            }

            odom_timestamp_ = update_timestamp > odom_timestamp_ ? update_timestamp : odom_timestamp_;

            return {vx_t, x_t, y_t, theta_t};
        }

        float normalize_angle(float angle)
        {
            angle = std::fmod(angle, 2.0f * M_PI);
            if (angle < 0.0f)
            {
                angle += 2.0f * M_PI;
            }
            return angle;
        }

    private:
        std::deque<float> v_buffer_;
        std::deque<float> w_buffer_;
        bool first_update_ = true;

#ifdef STM32
        // TODO: add timestamp
#else
        std::chrono::time_point<std::chrono::steady_clock> odom_timestamp_;
#endif
    };
} // namespace jlb

#endif // ODOMETRY_HXX
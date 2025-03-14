#ifndef ODOMETRY_HXX
#define ODOMETRY_HXX

#include <chrono>
#include <cmath>
#include <deque>
#include <numeric>

#include "common.hxx"

namespace jlb
{
    struct Odom
    {
        float vx             = 0.0f;
        float x              = 0.0f;
        float y              = 0.0f;
        float theta          = 0.0f;
        float distance_local = 0.0f;
    };

    class Odometry
    {
    public:
        float vx_t           = 0.0f;  // linear velocity
        float w_t            = 0.0f;  // angular velocity
        float x_t            = 0.0f;  // x position
        float y_t            = 0.0f;  // y position
        float theta_t        = 0.0f;  // orientation
        float x_t_local      = 0.0f;  // local x position
        float y_t_local      = 0.0f;  // local y position
        float theta_t_local  = 0.0f;  // local orientation
        float distance_local = 0.0f;  // distance traveled since last checkpoint

        float meas_wheel_rpm = 0.0f;
        float meas_ang_vel_x = 0.0f;
        float meas_ang_vel_y = 0.0f;
        float meas_ang_vel_z = 0.0f;
        float meas_lin_acc_x = 0.0f;
        float meas_lin_acc_y = 0.0f;
        float meas_lin_acc_z = 0.0f;

        uint32_t tick_counter      = 0u;
        uint32_t tick_counter_prev = 0u;
        Odometry(const float x_t_ = 0.0f, const float y_t_ = 0.0f, const float theta_t_ = 0.0f)
            : x_t(x_t_), y_t(y_t_), theta_t(normalize_angle(theta_t_))
        {
        }

        ~Odometry() {}

        void rpm_callback(const float wheel_rpm)
        {
            meas_wheel_rpm = wheel_rpm;
            float velocity = M_PI * jlb::WHEEL_DIAMETER * wheel_rpm / 60.0f;

            if (std::fabs(velocity) > jlb::MAX_VELOCITY) { return; }

            v_buffer_.push_back(velocity);
            if (v_buffer_.size() > jlb::VELOCITY_BUFFER_SIZE) { v_buffer_.pop_front(); }

            vx_t = std::accumulate(v_buffer_.begin(), v_buffer_.end(), 0.0f) / v_buffer_.size();
        }

        void imu_callback(
            const float ang_vel_x, const float ang_vel_y, const float ang_vel_z, const float lin_acc_x, const float lin_acc_y, const float lin_acc_z)
        {
            meas_ang_vel_z = ang_vel_x;
            meas_ang_vel_z = ang_vel_y;
            meas_ang_vel_z = ang_vel_z;
            meas_lin_acc_x = lin_acc_x;
            meas_lin_acc_x = lin_acc_y;
            meas_lin_acc_x = lin_acc_z;

            if (std::fabs(ang_vel_z) > jlb::MAX_YAW_RATE) { return; }

            w_buffer_.push_back(ang_vel_z);
            if (w_buffer_.size() > jlb::IMU_BUFFER_SIZE) { w_buffer_.pop_front(); }

            w_t = std::accumulate(w_buffer_.begin(), w_buffer_.end(), 0.0f) / w_buffer_.size();
        }

        Odom update_odom()
        {
#ifndef SIMULATION
            tick_counter_prev = tick_counter;
            tick_counter      = HAL_GetTick();
            float dt          = (((float)tick_counter) - ((float)(tick_counter_prev))) / 1000.0f;

#else
            if (first_update_)
            {
                odom_timestamp_ = std::chrono::steady_clock::now();
                first_update_   = false;
            }

            auto  update_timestamp = std::chrono::steady_clock::now();
            float dt               = std::chrono::duration_cast<std::chrono::milliseconds>(update_timestamp - odom_timestamp_).count() / 1000.0f;
#endif

            if (dt > 0.0f)
            {
                vx_t = std::fabs(vx_t) < 0.03 ? 0.0 : vx_t;
                w_t  = std::fabs(w_t) < 0.015 ? 0.0 : w_t;
                // float vy_t = WHEELBASE * w_t / 2.0f;
                float vy_t = 0.0f;

                x_t += (vx_t * std::cos(theta_t) - vy_t * std::sin(theta_t)) * dt;
                y_t += (vx_t * std::sin(theta_t) + vy_t * std::cos(theta_t)) * dt;
                theta_t = normalize_angle(theta_t + w_t * dt);

                x_t_local += vx_t * std::cos(theta_t_local) * dt;
                y_t_local += vx_t * std::sin(theta_t_local) * dt;
                theta_t_local = normalize_angle(theta_t_local + w_t * dt);
                distance_local += vx_t * dt;
            }
#ifdef SIMULATION
            odom_timestamp_ = update_timestamp > odom_timestamp_ ? update_timestamp : odom_timestamp_;
#endif
            return {vx_t, x_t, y_t, theta_t, distance_local};
        }

        float normalize_angle(float angle)
        {
            angle = std::fmod(angle, 2.0f * M_PI);
            if (angle < 0.0f) { angle += 2.0f * M_PI; }
            return angle;
        }

        void correction([[maybe_unused]] float x_t_, [[maybe_unused]] float y_t_)
        {
            x_t = x_t_ - (SENSOR_BASE / 2.0f * std::cos(theta_t));
            y_t = y_t_ - (SENSOR_BASE / 2.0f * std::sin(theta_t));

            // clamp theta to certain values whichever is closer
            // the values are 0, 90, 180, 270, 360

            float theta_0   = std::fabs(theta_t);
            float theta_90  = std::fabs(theta_t - M_PI / 2.0f);
            float theta_180 = std::fabs(theta_t - M_PI);
            float theta_270 = std::fabs(theta_t - 3.0f * M_PI / 2.0f);
            float theta_360 = std::fabs(theta_t - 2.0f * M_PI);

            float min_theta = std::min({theta_0, theta_90, theta_180, theta_270, theta_360});

            if (min_theta == theta_0) { theta_t = (theta_t + 0.0f) / 2.0f; }
            else if (min_theta == theta_90) { theta_t = (theta_t + M_PI / 2.0f) / 2.0f; }
            else if (min_theta == theta_180) { theta_t = (theta_t + M_PI) / 2.0f; }
            else if (min_theta == theta_270) { theta_t = (theta_t + 3.0f * M_PI / 2.0f) / 2.0f; }
            else if (min_theta == theta_360) { theta_t = (theta_t + 2.0f * M_PI) / 2.0f; }

            theta_t = normalize_angle(theta_t);
        }

        void reset_local(bool force = false)
        {
            if (std::fabs(distance_local) > WHEELBASE || force)
            {
                x_t_local      = 0.0f;
                y_t_local      = 0.0f;
                theta_t_local  = 0.0f;
                distance_local = 0.0f;
            }
        }

    private:
        std::deque<float>     v_buffer_;
        std::deque<float>     w_buffer_;
        [[maybe_unused]] bool first_update_ = true;

#ifdef SIMULATION
        std::chrono::time_point<std::chrono::steady_clock> odom_timestamp_;
#endif
    };
}  // namespace jlb

#endif  // ODOMETRY_HXX

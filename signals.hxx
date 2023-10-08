#ifndef SIGNALS_HXX
#define SIGNALS_HXX

#include <string>
#include <vector>

#include "common.hxx"
#include "udp.hxx"

namespace jlb
{
    struct Signal
    {
        uint8_t id;
        std::string name;
        std::string unit;
        float min;
        float max;
        float offset;
        float multiplier;
    };

    class SignalLibrary
    {
    public:
        std::vector<Signal> signals = std::vector<Signal>(SIGNALS_SIZE);

        ~SignalLibrary() {}

        static SignalLibrary &get_instance()
        {
            static SignalLibrary instance;
            return instance;
        }

        const Signal &operator[](const uint8_t id) const
        {
            return signals.at(id);
        }

    private:
        SignalLibrary()
        {
#ifndef SIMULATION

#else
            signals.at(TARGET_ANGLE_ID) = Signal({TARGET_ANGLE_ID, "target_angle", "rad", -1.0f, 1.0f, 1.0f, 1000.0f});
            signals.at(TARGET_SPEED_ID) = Signal({TARGET_SPEED_ID, "target_speed", "px/s", -MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY, 1000.0f});
            signals.at(POSITION_X_ID) = Signal({POSITION_X_ID, "position_x", "px", 0.0f, 1088.0f, 0.0f, 1.0f});
            signals.at(POSITION_Y_ID) = Signal({POSITION_Y_ID, "position_y", "px", 0.0f, 1024.0f, 0.0f, 1.0f});
            signals.at(POSITION_THETA_ID) = Signal({POSITION_THETA_ID, "position_theta", "rad", 0, 2 * M_PI, 0.0f, 1000.0f});
            signals.at(ODOM_LINEAR_VELOCITY_X_ID) = Signal({ODOM_LINEAR_VELOCITY_X_ID, "odom_linear_velocity_x", "px/s", -MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY, 1000.0f});
            signals.at(ODOM_ANGULAR_VELOCITY_Z_ID) = Signal({ODOM_ANGULAR_VELOCITY_Z_ID, "odom_angular_velocity_z", "rad/s", -MAX_YAW_RATE, MAX_YAW_RATE, MAX_YAW_RATE, 1000.0f});
            signals.at(MEAS_MOTOR_RPM_ID) = Signal({MEAS_MOTOR_RPM_ID, "meas_motor_rpm", "rpm", -MAX_MOTOR_RPM, MAX_MOTOR_RPM, MAX_MOTOR_RPM, 1000.0f});
            signals.at(MEAS_ANGULAR_VELOCITY_Z_ID) = Signal({MEAS_ANGULAR_VELOCITY_Z_ID, "meas_angular_velocity_z", "rad/s", -MAX_YAW_RATE, MAX_YAW_RATE, MAX_YAW_RATE, 1000.0f});
#endif
        }
    };

    class Value
    {
    public:
        uint8_t id;
        float value;
        Signal signal;
        static void packet_from_value(char *msg, size_t max_size, const Signal &signal, float value)
        {
            if (max_size < 5)
            {
                return;
            }

            msg[0] = signal.id;

            uint32_t unsigned_value = static_cast<uint32_t>((value + signal.offset) * signal.multiplier);

            // Assuming little-endian byte order, populate bytes 1-8 with the unsigned_value.
            for (int i = 1; i <= 4; ++i)
            {
                msg[i] = static_cast<uint8_t>(unsigned_value & 0xFF); // Extract the least significant byte
                unsigned_value >>= 8;                                 // Shift right to get the next byte
            }
        }

        Value(char *msg, size_t max_size)
        {
            if (max_size < 5)
            {
                return;
            }

            id = msg[0];
            signal = signal_library[id];

            uint32_t reconstructed_value = 0;

            // Assuming little-endian byte order, reconstruct the unsigned_value from bytes 1-8.
            for (int i = 4; i >= 1; --i)
            {
                reconstructed_value <<= 8;                           // Shift left to make room for the next byte
                reconstructed_value |= static_cast<uint8_t>(msg[i]); // Extract the next byte and add it to the reconstructed value
            }

            value = static_cast<float>(reconstructed_value) / signal.multiplier - signal.offset;
        }

        ~Value()
        {
        }

    private:
        SignalLibrary &signal_library = SignalLibrary::get_instance();
    };

    class SignalSender
    {
    public:
        ~SignalSender()
        {
        }

        static SignalSender &get_instance()
        {
            static SignalSender instance;
            return instance;
        }

        int send(char *msg, size_t max_size)
        {
#ifdef STM32
            // TODO: send UDP packet for STM32
#else
            return client.send(msg, max_size);
#endif
        }

    private:
#ifdef STM32
        // TODO: add UDPClient for STM32
#else
        UDPClient client;
#endif

#ifdef STM32
        // TODO: initialize UDPClient for STM32
#else
        SignalSender() : client(SERVER_ADDRESS, SERVER_PORT)
#endif
        {
        }
    };

} // namespace jlb

#endif // SIGNALS_HXX
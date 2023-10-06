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
            signals.at(TARGET_SPEED_ID) = Signal({TARGET_SPEED_ID, "target_speed", "px/s", -500.0f, 500.0f, 500.0f, 1000.0f});
#endif
        }
    };

    class Value
    {
    public:
        uint8_t id;
        float value;
        Signal signal;

        Value(char *msg, size_t max_size)
        {
            if (max_size < 5)
            {
                return;
            }

            // first byte is the id
            id = msg[0];
            signal = signal_library.signals[id];

            // the following 4 bytes are the value in MSB order
            uint32_t second_byte = msg[1];
            uint32_t third_byte = msg[2];
            uint32_t fourth_byte = msg[3];
            uint32_t fifth_byte = msg[4];
            uint32_t unsigned_value = (second_byte << 24) | (third_byte << 16) | (fourth_byte << 8) | fifth_byte;

            // convert to float
            value = *reinterpret_cast<float *>(&unsigned_value);

            // apply offset and multiplier
            value = value / signal.multiplier - signal.offset;
        }

        ~Value() {}

        static void packet_from_value(uint8_t *msg, size_t max_size, const Signal &signal, float value)
        {
            if (max_size < 5)
            {
                return;
            }

            // first byte is the id
            msg[0] = signal.id;

            // apply offset and multiplier
            value = (value + signal.offset) * signal.multiplier;

            // the following 4 bytes are the value in MSB order
            uint32_t unsigned_value = *reinterpret_cast<uint32_t *>(&value);
            msg[1] = (unsigned_value >> 24) & 0xFF;
            msg[2] = (unsigned_value >> 16) & 0xFF;
            msg[3] = (unsigned_value >> 8) & 0xFF;
            msg[4] = unsigned_value & 0xFF;
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

        int send(uint8_t *msg, size_t max_size)
        {
            return client.send(msg, max_size);
        }

    private:
        UDPClient client;

        SignalSender() : client(SERVER_ADDRESS, SERVER_PORT)
        {
        }
    };

} // namespace jlb

#endif // SIGNALS_HXX
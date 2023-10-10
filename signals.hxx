#ifndef SIGNALS_HXX
#define SIGNALS_HXX

#include <string>
#include <vector>

#include "common.hxx"
#include "udp.hxx"

#include "jlb-binutil.h"

namespace jlb
{
    class SignalSender
    {
    public:
        jlb_rx_t jlb_rx_t;

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
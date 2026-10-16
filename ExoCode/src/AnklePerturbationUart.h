#ifndef ANKLE_PERTURBATION_UART_H
#define ANKLE_PERTURBATION_UART_H
#include "AnklePerturbationSupport.h"
#include "AnklePerturbationProtocol.h"

namespace ankle_perturbation
{
    template<class Handler> void flush_events(Handler* handler, ExoData* data)
    {
        Status event;
        while (data->perturbation.pop(event)) handler->UART_msg(wire::pack_status(event));
    }

    // Called on the control MCU ONLY. Unknown/legacy commands continue through
    // their existing handlers unchanged, but cannot leave a pulse armed.
    template<class Handler> bool handle_uart(Handler* handler, ExoData* data, const UART_msg_t& msg)
    {
        if (msg.command == wire::query)
        {
            if (msg.len != 0) return true;
            data->perturbation.begin_session(millis());
            handler->UART_msg(wire::pack_status(Status{}));
            return true;
        }
        if (msg.command == wire::request)
        {
            if (msg.len != 4 || !wire::integer(msg.data[0], 255)) return true;
            const uint16_t id = uint16_t(msg.joint_id) * 256 + (uint16_t)msg.data[0];
            if (!valid_request(msg.data[1], msg.data[2], msg.data[3], id)) return true;
            const uint8_t side = (uint8_t)msg.data[1];
            data->perturbation.request(side, msg.data[2], (uint16_t)msg.data[3],
                                       id, millis(), allowed(data, side), rejection_reason(data, side));
            flush_events(handler, data);
            return true;
        }
        if (msg.command == wire::link)
        {
            if (msg.len == 1 && wire::integer(msg.data[0], 1))
                data->perturbation.heartbeat(msg.data[0] != 0, millis());
            flush_events(handler, data);
            return true;
        }
        // Existing protocol values are intentionally not renumbered.
        switch (msg.command)
        {
            case 0x02: case 0x04: case 0x06: // controller/status/config
            case 0x08: case 0x0A: case 0x0C: // calibration/refinement
            case 0x0E: case 0x10: case 0x14: // motors/zero/parameter
            case 0x18: case 0x19: // FSR thresholds/reset
                data->perturbation.cancel();
                flush_events(handler, data);
                break;
        }
        return false;
    }
}
#endif

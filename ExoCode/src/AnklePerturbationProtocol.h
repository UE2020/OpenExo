#ifndef ANKLE_PERTURBATION_PROTOCOL_H
#define ANKLE_PERTURBATION_PROTOCOL_H
#include "AnklePerturbation.h"
#include "UART_msg_t.h"
#include <stdio.h>

namespace ankle_perturbation { namespace wire
{
    constexpr uint8_t query = 0x1C;
    constexpr uint8_t status = 0x1D;
    constexpr uint8_t request = 0x1E;
    constexpr uint8_t link = 0x20;

    inline bool integer(float value, float upper)
    {
        return isfinite(value) && value >= 0 && value <= upper && value == floorf(value);
    }

    inline bool from_ble(const float* values, int count, UART_msg_t& msg)
    {
        if (!values || count != 4 || !valid_request(values[0], values[1], values[2], values[3])) return false;
        const uint16_t id = (uint16_t)values[3];
        msg = UART_msg_t{};
        msg.command = request;
        msg.joint_id = id >> 8;
        msg.len = 4;
        msg.data[0] = id & 255;
        msg.data[1] = values[0];
        msg.data[2] = values[1];
        msg.data[3] = values[2];
        return true;
    }

    inline UART_msg_t pack_status(const Status& event)
    {
        // Teensy->Nano UART scales every float by 100 into int16_t. Splitting
        // IDs and integer centi-Nm into bytes avoids overflow AND truncation.
        UART_msg_t msg{};
        msg.command = status;
        msg.joint_id = event.request_id >> 8;
        msg.len = 7;
        const uint16_t magnitude = (uint16_t)(lroundf(event.magnitude * 100) + 1000);
        msg.data[0] = event.request_id & 255;
        msg.data[1] = event.side;
        msg.data[2] = event.state;
        msg.data[3] = magnitude & 255;
        msg.data[4] = magnitude >> 8;
        msg.data[5] = event.duration_ms;
        msg.data[6] = event.reason;
        return msg;
    }

    inline bool unpack_status(const UART_msg_t& msg, Status& event)
    {
        if (msg.command != status || (msg.len != 6 && msg.len != 7)) return false;
        for (uint8_t i=0; i<msg.len; ++i) if (!integer(msg.data[i], 255)) return false;
        if (msg.len == 7 && msg.data[6] > unsafe_state) return false;
        const uint16_t id = uint16_t(msg.joint_id) * 256 + (uint16_t)msg.data[0];
        const uint16_t magnitude = uint16_t(msg.data[4]) * 256 + (uint16_t)msg.data[3];
        if (id > 9999 || msg.data[1] > 2 || msg.data[2] > 6 || magnitude > 2000 || msg.data[5] > 200) return false;
        event = Status{id, (uint8_t)msg.data[1], (uint8_t)msg.data[2],
                       (int(magnitude) - 1000) / 100.0f, (uint16_t)msg.data[5],
                       msg.len == 7 ? (uint8_t)msg.data[6] : uint8_t(no_rejection)};
        return true;
    }

    inline int pack_readback(const Status& event, uint8_t* buffer, size_t size)
    {
        if (!buffer || size < 12) return 0;
        const int16_t magnitude = (int16_t)lroundf(event.magnitude * 100);
        const uint16_t fields[3] = {event.request_id, (uint16_t)magnitude, event.duration_ms};
        buffer[0] = 'A'; buffer[1] = 'P'; buffer[2] = 1;
        buffer[3] = fields[0] & 255; buffer[4] = fields[0] >> 8;
        buffer[5] = event.side; buffer[6] = event.state; buffer[7] = event.reason;
        buffer[8] = fields[1] & 255; buffer[9] = fields[1] >> 8;
        buffer[10] = fields[2] & 255; buffer[11] = fields[2] >> 8;
        return 12;
    }

    inline int format_status(const Status& event, char* buffer, size_t size)
    {
        const int length = snprintf(buffer, size, "Sp5c%ldn%ldn%ldn%ldn%ldn",
            long(event.request_id) * 100, long(event.side) * 100, long(event.state) * 100,
            lroundf(event.magnitude * 100), long(event.duration_ms) * 100);
        return length > 0 && size_t(length) < size ? length : 0;
    }
}}
#endif

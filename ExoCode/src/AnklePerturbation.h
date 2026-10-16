#ifndef ANKLE_PERTURBATION_H
#define ANKLE_PERTURBATION_H

#include <stdint.h>
#include <math.h>

// Hardware-independent state machine, executed by the Teensy's control loop.
namespace ankle_perturbation
{
    enum State : uint8_t { idle=0, armed=1, active=2, completed=3,
                           cancelled=4, rejected=5, expired=6 };
    enum Reason : uint8_t { no_rejection=0, trial_not_running=1, estop_active=2,
        user_is_paused=3, device_error=4, calibration_in_progress=5, unsupported_controller=6,
        ankle_unused=7, motor_disabled=8, motor_timeout=9, torque_uncalibrated=10,
        invalid_sensor=11, no_session=12, stale_link=13, already_busy=14,
        reused_request=15, malformed_request=16, unsafe_state=17 };
    struct Status
    {
        uint16_t request_id = 0;
        uint8_t side = 0;
        uint8_t state = idle;
        float magnitude = 0;
        uint16_t duration_ms = 0;
        uint8_t reason = no_rejection;
    };

    inline bool valid_request(float side, float magnitude, float duration, float request_id)
    {
        if (!isfinite(side) || !isfinite(magnitude) || !isfinite(duration) || !isfinite(request_id)) return false;
        if (side != floorf(side) || duration != floorf(duration) || request_id != floorf(request_id)) return false;
        if (request_id < 1 || request_id > 9999) return false;
        if (side == 0) return magnitude == 0 && duration == 1;
        return (side == 1 || side == 2) && fabsf(magnitude) > 0 && fabsf(magnitude) <= 10 && duration >= 10 && duration <= 200;
    }

    class Engine
    {
    public:
        void begin_session(uint32_t now)
        {
            // Rechecking capability on the same live link is read-only for the
            // pulse and replay protection. Only a new/expired link resets IDs.
            if (!_session || uint32_t(now - _last_link) > 500)
            {
                _high_water_id = 0;
                _status = Status{};
                _head = _count = 0;
            }
            _session = true;
            _last_link = now;
        }

        void heartbeat(bool connected, uint32_t now)
        {
            if (!connected) { cancel(); _session = false; }
            else if (_session) _last_link = now;
        }

        void cancel()
        {
            if (_status.state == armed || _status.state == active)
            {
                _status.state = cancelled;
                push(_status);
            }
        }

        uint8_t side() const { return _status.side; }

        void request(uint8_t side, float magnitude, uint16_t duration,
                     uint16_t request_id, uint32_t now, bool allowed, uint8_t denial = unsafe_state)
        {
            if (side == 0) { cancel(); return; }
            if (request_id == _status.request_id && side == _status.side &&
                magnitude == _status.magnitude && duration == _status.duration_ms)
            {
                push(_status); // idempotent retry, including after completion
                return;
            }
            Status reply{request_id, side, rejected, magnitude, duration};
            const bool busy = _status.state == armed || _status.state == active;
            if (!_session || uint32_t(now - _last_link) > 500 || !allowed || busy ||
                request_id <= _high_water_id || !valid_request(side, magnitude, duration, request_id))
            {
                reply.reason = !_session ? no_session :
                    uint32_t(now - _last_link) > 500 ? stale_link :
                    !allowed ? denial : busy ? already_busy :
                    request_id <= _high_water_id ? reused_request : malformed_request;
                if (request_id > _high_water_id) _high_water_id = request_id;
                push(reply);
                return;
            }
            _high_water_id = request_id;
            _status = Status{request_id, side, armed, magnitude, duration};
            _armed_at = now;
            push(_status);
        }

        void tick(uint32_t now, uint8_t side, bool ground_strike, bool allowed)
        {
            if (_status.state != armed && _status.state != active) return;
            if (!allowed || !_session || uint32_t(now - _last_link) > 500)
            {
                cancel();
                return;
            }
            if (_status.state == armed && uint32_t(now - _armed_at) >= 10000)
            {
                _status.state = expired;
                push(_status);
            }
            else if (_status.state == armed && side == _status.side && ground_strike)
            {
                _started_at = now;
                _status.state = active;
                push(_status);
            }
            else if (_status.state == active && uint32_t(now - _started_at) >= _status.duration_ms)
            {
                _status.state = completed;
                push(_status);
            }
        }

        float offset(uint8_t side) const
        {
            return _status.state == active && side == _status.side ? _status.magnitude : 0.0f;
        }

        bool pop(Status& result)
        {
            if (!_count) return false;
            result = _events[_head];
            _head = (_head + 1) % 8;
            --_count;
            return true;
        }

    private:
        void push(const Status& status)
        {
            // Bounded storage; keep the most recent authoritative terminal state.
            if (_count == 8) { _head = (_head + 1) % 8; --_count; }
            _events[(_head + _count) % 8] = status;
            ++_count;
        }
        Status _status;
        Status _events[8];
        uint8_t _head = 0, _count = 0;
        bool _session = false;
        uint16_t _high_water_id = 0;
        uint32_t _last_link = 0, _armed_at = 0, _started_at = 0;
    };
}
#endif

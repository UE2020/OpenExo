#ifndef BLE_TX_QUEUE_H
#define BLE_TX_QUEUE_H

#include <stddef.h>
#include <stdint.h>

// Complete frames stay in FIFO order while the MCU services UART/I2C. Never
// interleave another frame inside a partially transmitted controller snapshot.
class BleTxQueue
{
public:
    static const size_t CAPACITY = 64;
    static const size_t MAX_FRAME_BYTES = 512;

    uint8_t* writable_data()
    {
        return _count < CAPACITY ? _frames[(_head + _count) % CAPACITY].data : nullptr;
    }

    bool commit(size_t length)
    {
        if (_count == CAPACITY || length == 0 || length > MAX_FRAME_BYTES)
        {
            return false;
        }
        _frames[(_head + _count) % CAPACITY].length = length;
        ++_count;
        return true;
    }

    const uint8_t* front_data() const
    {
        return _count ? _frames[_head].data + _offset : nullptr;
    }

    size_t front_remaining() const
    {
        return _count ? _frames[_head].length - _offset : 0;
    }

    void consume(size_t length)
    {
        if (length == 0 || length > front_remaining())
        {
            return;
        }
        _offset += length;
        if (_offset == _frames[_head].length)
        {
            _head = (_head + 1) % CAPACITY;
            --_count;
            _offset = 0;
        }
    }

    template<typename Notify>
    bool flush_one(size_t payload, bool credits_ready, Notify notify)
    {
        const size_t remaining = front_remaining();
        if (!credits_ready || !payload || !remaining)
        {
            return false;
        }
        const size_t length = remaining < payload ? remaining : payload;
        if (!notify(front_data(), length))
        {
            return false;
        }
        consume(length);
        return true;
    }

    void clear()
    {
        _head = 0;
        _count = 0;
        _offset = 0;
    }

private:
    struct Frame
    {
        uint8_t data[MAX_FRAME_BYTES + 1]; // spare byte for snprintf's terminator
        size_t length;
    };
    Frame _frames[CAPACITY];
    size_t _head = 0;
    size_t _count = 0;
    size_t _offset = 0;
};

#endif

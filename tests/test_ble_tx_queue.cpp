// Native smoke: g++ -std=c++11 tests/test_ble_tx_queue.cpp -o test_ble_tx_queue
#include "../ExoCode/src/BleTxQueue.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

static void enqueue(BleTxQueue& queue, const std::string& frame)
{
    uint8_t* slot = queue.writable_data();
    assert(slot != nullptr);
    assert(frame.size() <= BleTxQueue::MAX_FRAME_BYTES);
    std::memcpy(slot, frame.data(), frame.size());
    assert(queue.commit(frame.size()));
}

static std::string snapshot(unsigned joint, unsigned controller, float base)
{
    std::string frame = "Sp24c" + std::to_string(joint) + "n" + std::to_string(controller) + "n";
    for (unsigned i = 0; i < 22; ++i)
    {
        char token[32];
        std::snprintf(token, sizeof(token), "%.9gn", static_cast<double>(base + i * .001f));
        frame += token;
    }
    return frame;
}

static void continuous_samples(size_t mtu_payload)
{
    BleTxQueue queue;
    std::string expected;
    std::string delivered;
    std::vector<std::string> notifications;
    unsigned service_calls = 0;
    unsigned deferred = 0;
    unsigned sample_count = 0;
    auto notify = [&](const uint8_t* bytes, size_t length)
    {
        assert(length <= mtu_payload);
        notifications.emplace_back(reinterpret_cast<const char*>(bytes), length);
        delivered += notifications.back();
        return true;
    };
    auto push = [&](const std::string& frame)
    {
        enqueue(queue, frame);
        expected += frame;
    };
    const std::string first_snapshot = snapshot(65, 2, .00000001f);
    push(first_snapshot);
    for (unsigned tick = 0; tick < 600; ++tick)
    {
        // The real scheduler returns on exhausted credits; the next MCU loop
        // remains free to service a newly generated study sample every 9ms.
        ++service_calls;
        if (tick % 9 == 0)
        {
            push("S?3c" + std::to_string(sample_count * 100) + "n200n300n");
            ++sample_count;
        }
        if (tick == 50)
        {
            push("Sa5c6500n200n0n100n0n");
        }
        if (tick == 300)
        {
            push(snapshot(33, 3, -.125f));
        }
        const bool credits_ready = tick >= 200;
        const size_t notifications_before = notifications.size();
        queue.flush_one(mtu_payload, credits_ready, notify);
        if (!credits_ready)
        {
            ++deferred;
            assert(notifications.size() == notifications_before);
        }
    }
    while (queue.front_remaining())
    {
        assert(queue.flush_one(mtu_payload, true, notify));
    }
    assert(delivered == expected); // exact snapshots, ACK, and every sample in order
    assert(service_calls == 600 && deferred == 200 && sample_count == 67);
    if (first_snapshot.size() <= mtu_payload)
    {
        assert(notifications.front() == first_snapshot); // negotiated-MTU throughput
    }
    std::cout << "payload=" << mtu_payload << ": 67 ordered samples, 2 snapshots, ACK; "
              << "200 credit-starved service cycles returned without notify\n";
}

static void failed_delivery_capacity_wrap_and_disconnect()
{
    BleTxQueue queue;
    std::string expected;
    for (unsigned i = 0; i < BleTxQueue::CAPACITY; ++i)
    {
        const std::string frame = "S?2c" + std::to_string(i * 100) + "n200n";
        enqueue(queue, frame);
        expected += frame;
    }
    assert(queue.writable_data() == nullptr);
    assert(!queue.commit(1));
    const size_t before = queue.front_remaining();
    assert(!queue.flush_one(20, true, [](const uint8_t*, size_t) { return false; }));
    assert(queue.front_remaining() == before);
    std::string delivered;
    auto notify = [&](const uint8_t* bytes, size_t length)
    {
        delivered.append(reinterpret_cast<const char*>(bytes), length);
        return true;
    };
    while (queue.front_remaining())
    {
        assert(queue.flush_one(20, true, notify));
    }
    assert(delivered == expected);
    enqueue(queue, snapshot(65, 2, 1));
    assert(queue.flush_one(20, true, notify));
    assert(queue.front_remaining());
    queue.clear(); // a partially sent snapshot cannot leak into the next link
    unsigned notifications = 0;
    assert(!queue.flush_one(20, true, [&](const uint8_t*, size_t) { ++notifications; return true; }));
    assert(notifications == 0);
    delivered.clear();
    enqueue(queue, "S?2c9900n100n");
    assert(queue.flush_one(244, true, notify));
    assert(delivered == "S?2c9900n100n");
    std::cout << "full FIFO preserved all 64 samples; failed sends retried; reconnect starts clean\n";
}

int main()
{
    continuous_samples(20);
    continuous_samples(244);
    failed_delivery_capacity_wrap_and_disconnect();
}

#include "Arduino.h"
#include "BleMessageQueue.h"
#include "Logger.h"

//For mutex lock
#if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
#include "mbed.h"
#include "rtos.h"
static rtos::Mutex queue_mutex;
#endif

#define BLE_QUEUE_DEBUG 0

static const int k_max_size = 32;
static BleMessage queue[k_max_size];
static int m_head = 0;
static int m_tail = 0;
static int m_size = 0;
static const BleMessage empty_message = BleMessage();

BleMessage ble_queue::pop()
{
    #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
        queue_mutex.lock();
    #endif

    if (m_size > 0)
    {
        BleMessage msg = queue[m_head];
        m_head = (m_head + 1) % k_max_size;
        m_size--;
        #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
            queue_mutex.unlock();
        #endif
        return msg;
    }
    else
    {
        logger::println("BleMessageQueue::pop_queue->No messages in Queue!", LogLevel::Warn);
        #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
            queue_mutex.unlock();
        #endif
        return empty_message;
    }

    #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
        queue_mutex.unlock();
    #endif
}

void ble_queue::push(BleMessage* msg)
{
    #if BLE_QUEUE_DEBUG
        logger::println("BleMessageQueue::push_queue");
        logger::print("m_size: ");
        logger::println(m_size);
        logger::print("msg: ");
        BleMessage::print(*msg);
    #endif

    #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
        queue_mutex.lock();
    #endif

    if (m_size >= k_max_size)
    {
        logger::println("BleMessageQueue::push_queue->Queue Full!", LogLevel::Warn);
        #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
            queue_mutex.unlock();
        #endif
        return;
    }

    queue[m_tail].copy(msg);
    m_tail = (m_tail + 1) % k_max_size;
    m_size++;

    #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
        queue_mutex.unlock();
    #endif

    #if BLE_QUEUE_DEBUG
        logger::print("m_size: ");
        logger::println(m_size);
        logger::print("msg: ");
        const int last_index = (m_tail + k_max_size - 1) % k_max_size;
        BleMessage::print(queue[last_index]);
    #endif
}

int ble_queue::size()
{
    #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
        queue_mutex.lock();
    #endif

    const int size = m_size;

    #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
        queue_mutex.unlock();
    #endif

    return size;
}

int ble_queue::check_for(BleMessage msg)
{
    #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
        queue_mutex.lock();
    #endif

    int found = 0;
    for (int i = 0; i < m_size; i++)
    {
        const int index = (m_head + i) % k_max_size;
        found += BleMessage::matching(queue[index], msg);
    }

    #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
        queue_mutex.unlock();
    #endif

    return found;
}

void ble_queue::clear()
{
    #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
        queue_mutex.lock();
    #endif

    m_head = 0;
    m_tail = 0;
    m_size = 0;

    #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
        queue_mutex.unlock();
    #endif
}

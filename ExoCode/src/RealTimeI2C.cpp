#include "RealTimeI2C.h"

#include "Config.h"
#include "Utilities.h"
#include "Logger.h"
#include <Wire.h>

//#define RT_I2C_DEBUG 1

#define FIXED_POINT_FACTOR 100

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
    #define HOST 1
    #if BOARD_VERSION == AK_Board_V0_3
    #define MY_WIRE Wire1
    #else
    #define MY_WIRE Wire
    #endif
#elif defined(ARDUINO_ARDUINO_NANO33BLE)
    #define HOST 0
    #define MY_WIRE Wire
#endif

#define RT_I2C_ADDR 0x02
#define RT_I2C_REG 0x02

namespace rt_data
{
    float float_values[len] = {0};
    bool new_rt_msg = false;
}

static volatile bool new_bytes = false;
static const uint8_t bytes_per_value = sizeof(short int);
static const int byte_buffer_len = rt_data::len * bytes_per_value + 2;
static volatile uint8_t byte_buffer[byte_buffer_len] = {0};

static void _pack(uint8_t msg_id, uint8_t len, float *data, uint8_t *data_to_pack)
{
    data_to_pack[0] = msg_id;
    data_to_pack[1] = len;

    uint8_t buf[bytes_per_value];
    for (uint8_t i = 0; i < len; i++)
    {
        utils::float_to_short_fixed_point_bytes(data[i], buf, FIXED_POINT_FACTOR);
        const uint8_t offset = 2 + bytes_per_value * i;
        memcpy((data_to_pack + offset), buf, bytes_per_value);
    }
}

void real_time_i2c::msg(float* data, int len)
{
    if (data == nullptr || len != rt_data::len)
    {
        return;
    }

    uint8_t bytes[byte_buffer_len] = {0};
    _pack((uint8_t)RT_I2C_REG, rt_data::len, data, bytes);

    #if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
        MY_WIRE.beginTransmission(RT_I2C_ADDR);
        MY_WIRE.send(bytes, byte_buffer_len);
        MY_WIRE.endTransmission();
    #endif
}

#if defined(ARDUINO_ARDUINO_NANO33BLE)
// Warning: this interrupt must stay short. Do not add serial prints here.
static void on_receive(int byte_len)
{
    if (byte_len != byte_buffer_len)
    {
        while (MY_WIRE.available())
        {
            MY_WIRE.read();
        }
        return;
    }

    int bytes_read = 0;
    while (bytes_read < byte_len && MY_WIRE.available())
    {
        byte_buffer[bytes_read++] = MY_WIRE.read();
    }
    new_bytes = (bytes_read == byte_buffer_len);
}
#endif

void real_time_i2c::init()
{
    #if HOST
        MY_WIRE.begin();
    #else
        MY_WIRE.begin(RT_I2C_ADDR);
        MY_WIRE.onReceive(on_receive);
    #endif
}

bool real_time_i2c::poll(float* pack_array)
{
    #if RT_I2C_DEBUG
        logger::println("real_time_i2c::poll()->Start");
    #endif

    if (pack_array == nullptr || !new_bytes)
    {
        return false;
    }

    uint8_t buff[byte_buffer_len];
    noInterrupts();
    for (int i = 0; i < byte_buffer_len; i++)
    {
        buff[i] = byte_buffer[i];
    }
    new_bytes = false;
    interrupts();

    const uint8_t msg_id = buff[0];
    const uint8_t len = buff[1];
    if (msg_id != RT_I2C_REG || len != rt_data::len)
    {
        return false;
    }

    #if RT_I2C_DEBUG
        logger::print("real_time_i2c::poll()->Done copying bytes, len: ");
        logger::print(len);
        logger::println();
    #endif

    for (uint8_t i = 0; i < len; i++)
    {
        const uint8_t data_offset = 2 + (i * bytes_per_value);
        float tmp = 0;
        utils::short_fixed_point_bytes_to_float(
            (uint8_t*)(buff + data_offset),
            &tmp,
            FIXED_POINT_FACTOR);
        pack_array[i] = tmp;
    }

    for (uint8_t i = len; i < rt_data::len; i++)
    {
        pack_array[i] = 0;
    }
    

    #if RT_I2C_DEBUG
        logger::println("real_time_i2c::poll()->End");
    #endif
    return true;
}

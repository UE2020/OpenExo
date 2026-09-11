#include "BleParser.h"
#include "ble_commands.h"
#include "Utilities.h"
#include "BleMessageQueue.h"
#include "Logger.h"
#include <limits.h>
#include <math.h>

#define BLE_PARSER_DEBUG 0  //Make 1 if you want to enable Debug prints

BleParser::BleParser()
{
    ;
}

void BleParser::reset()
{
    _working_message.clear();
    _waiting_for_data = false;
    _bytes_collected = 0;
    _last_activity_ms = 0;
    memset(_buffer, 0, sizeof(_buffer));
}

BleMessage *BleParser::handle_raw_data(char *buffer, int length)
{
    #if BLE_PARSER_DEBUG
        logger::println("BleParser::handle_raw_data");
        logger::print("length: ");
        logger::println(length);
        logger::print("buffer: ");
        for (int i = 0; i < length; i++)
        {
            logger::print(buffer[i]);
            logger::print(" ");
        }
        logger::print("\n");
    #endif

    static BleMessage return_message;
    BleMessage *return_msg = &return_message;

    return_msg->is_complete = false;
    if (buffer == nullptr || length <= 0)
    {
        return return_msg;
    }

    const uint32_t now_ms = millis();
    if (_waiting_for_data &&
        static_cast<uint32_t>(now_ms - _last_activity_ms) >
            _payload_timeout_ms)
    {
        reset();
        if (length != 1)
        {
            return return_msg;
        }
    }

    //If we are not waiting for data, then we are expecting a new command
    if (!_waiting_for_data)
    {
        // Commands and payloads are written separately by the GUI.
        if (length != 1)
        {
            reset();
            return return_msg;
        }

        _handle_command(*buffer);

        //If the message is complete, then we can return it
        if (_working_message.is_complete)
        {
            #if BLE_PARSER_DEBUG
                logger::print("BleParser::handle_raw_data->message is complete\n");
            #endif

            return_msg->copy(&_working_message);
            reset();
        }
        else
        {
            if (_waiting_for_data)
            {
                _last_activity_ms = now_ms;
            }
            #if BLE_PARSER_DEBUG
                logger::print("BleParser::handle_raw_data->message is not complete\n");
            #endif
            //_waiting_for_data = true;
        }
    }
    else
    {
        const int expected_bytes =
            _working_message.expecting * static_cast<int>(sizeof(double));
        const int remaining_bytes = expected_bytes - _bytes_collected;

        if (expected_bytes <= 0 ||
            expected_bytes > static_cast<int>(sizeof(_buffer)) ||
            length > remaining_bytes)
        {
            reset();
            return return_msg;
        }

        //Add the data packet to the working message
        memcpy(&_buffer[_bytes_collected], buffer, length);
        _bytes_collected += length;
        _last_activity_ms = now_ms;

        //If we have collected all the data that we were expecting, then we can return the message
        if (_bytes_collected == expected_bytes)
        {
            return_msg->copy(&_working_message);
            for (int i = 0; i < expected_bytes; i += sizeof(double))
            {
                double f_tmp = 0;
                memcpy(&f_tmp, &_buffer[i], sizeof(double));
                return_msg->data[i / sizeof(double)] = static_cast<float>(f_tmp);
            }
            return_msg->is_complete = true;
            reset();
        }
    }

    #if BLE_PARSER_DEBUG
        logger::print("return_msg: ");
        BleMessage::print(*return_msg);
    #endif

    return return_msg;
}

int BleParser::package_raw_data(
    byte *buffer,
    int buffer_capacity,
    BleMessage &msg)
{
    #if BLE_PARSER_DEBUG
        logger::print("BleParser::package_raw_data");
        BleMessage::print(msg);
    #endif

    if (buffer == nullptr ||
        buffer_capacity <= 0 ||
        msg.expecting < 0 ||
        msg.expecting > BleMessage::MAX_DATA)
    {
        logger::println(
            "BleParser::package_raw_data: invalid message size",
            LogLevel::Error);
        return 0;
    }

    char cBuffer[MAX_NUMBER_CHARS + 1] = {0};
    int buffer_index = 0;
    const int expecting_char_length =
        utils::get_char_length(msg.expecting);
    const int preamble_length = 3 + expecting_char_length;
    if (expecting_char_length <= 0 ||
        preamble_length > buffer_capacity)
    {
        return 0;
    }

    buffer[buffer_index++] = _start_char;
    buffer[buffer_index++] = msg.command;
    itoa(msg.expecting, &cBuffer[0], 10);
    memcpy(&buffer[buffer_index], &cBuffer[0], expecting_char_length);
    buffer_index += expecting_char_length;
    buffer[buffer_index++] = _start_data;
    for (int i = 0; i < msg.expecting; i++)
    {
        const double data_to_send = static_cast<double>(msg.data[i]);
        const double scaled_data = data_to_send * 100.0;

        // Send as an integer to reduce bytes without invoking an invalid
        // floating-point-to-integer conversion.
        int modData = 0;
        if (isfinite(data_to_send) &&
            scaled_data >= static_cast<double>(INT_MIN) &&
            scaled_data <= static_cast<double>(INT_MAX))
        {
            modData = static_cast<int>(scaled_data);
        }
        int cLength = utils::get_char_length(modData);
       
        if (cLength <= 0 || cLength > MAX_NUMBER_CHARS)
        {
            cLength = 1;
            modData = 0;
        }

        if (buffer_index + cLength + 1 > buffer_capacity)
        {
            logger::println(
                "BleParser::package_raw_data: output buffer full",
                LogLevel::Error);
            return 0;
        }

        //Populates cBuffer with a base 10 number
        itoa(modData, &cBuffer[0], 10);

        //Writes cLength indices of cBuffer into buffer
        memcpy(&buffer[buffer_index], &cBuffer[0], cLength);
        buffer_index += cLength;
        buffer[buffer_index++] = _delimiter;
    }
    

    #if BLE_PARSER_DEBUG
        logger::print("BleParser::package_raw_data: buffer: ");
        for (int i = 0; i < buffer_index; i++)
        {
            logger::print(buffer[i]);
            logger::print(" ");
        }
        logger::print("\n");
    #endif

    return buffer_index;
}

/*
 * Private Functions
 */

void BleParser::_handle_command(char command)
{
    #if BLE_PARSER_DEBUG
        logger::print("BleParser::_handle_command: ");
        logger::print(command);
        logger::print("\n");
    #endif

    int length = -1;

    //Get the ammount of characters to wait for
    for (unsigned int i = 0; i < sizeof(ble::commands) / sizeof(ble::commands[0]); i++)
    {
        if (command == ble::commands[i].command)
        {
            length = ble::commands[i].length;
            break;
        }
    }
    if (length < 0 || length > BleMessage::MAX_DATA)
    {
        _working_message.clear();
        logger::print(
            "BleParser::_handle_command: invalid command/length: ",
            LogLevel::Error);
        logger::println(command, LogLevel::Error);
    }
    else
    {
        _waiting_for_data = (length != 0);
        _working_message.command = command;
        _working_message.expecting = length;
        _working_message.is_complete = !_waiting_for_data;
    }

    #if BLE_PARSER_DEBUG
        logger::print("BleParser::_handle_command: ");
        BleMessage::print(_working_message);
    #endif
}

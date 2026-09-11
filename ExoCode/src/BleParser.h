/**
 * @file BleParser.h
 * @author Chance Cuddeback
 * @brief This class is used to serialize and deserialize the BLE data. The application uses Nordic's UART service to
 * pass all data. Therefore, the command-data pairs must be packaged together and unpackaged on the peripheral and central.
 * @date 2022-08-22
 *
 */

#ifndef BLEPARSER_H
#define BLEPARSER_H

#include "Arduino.h"
#include "ble_commands.h"
#include "BleMessage.h"
#include "GattDb.h"

class BleParser
{
public:
    /**
     * @brief Construct a new Ble Parser object
     *
     */
    BleParser();

    /**
     * @brief Given a buffer and buffer length, package the data into a BleMessage class.
     *
     * @param buffer Raw character buffer
     * @param length Length of buffer
     * @return BleMessage* A pointer to the working message
     */
    BleMessage *handle_raw_data(char *buffer, int length);

    /** Discard any incomplete command/payload at a connection boundary. */
    void reset();

    /**
     * @brief Packs data to be sent to the GUI, modifies the char array that you send it
     *
     * @param buffer Buffer to hold the serialized data. This will be modified and should be equal to the
     * maximum length that the message could be
     * @param msg The message that you would like serialized
     * @return int Length of the buffer used
     */
    int package_raw_data(
        byte *buffer,
        int buffer_capacity,
        BleMessage &msg);

    static const int MAX_NUMBER_CHARS = 11;
    static const int MAX_SERIALIZED_BYTES =
        5 + BleMessage::MAX_DATA * (MAX_NUMBER_CHARS + 1);

private:
    const char _start_char = 'S';
    const char _start_data = 'c';
    const char _delimiter = 'n';
    //Empty message
    const BleMessage _empty_message = BleMessage();

    //The current working message, used by handle_raw_data
    BleMessage _working_message = BleMessage();
    bool _waiting_for_data = false;
    int _bytes_collected = 0;
    byte _buffer[64] = {0};
    uint32_t _last_activity_ms = 0;
    static const uint32_t _payload_timeout_ms = 1000;

    /**
     * @brief Checks if the incoming command is valid, and sets expecting value in the working message
     *
     * @param command Command to check
     */
    void _handle_command(char command);
};

#endif

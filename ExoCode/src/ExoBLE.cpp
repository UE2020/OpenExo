#if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)

#include "ExoBLE.h"
#include "Utilities.h"
#include "Time_Helper.h"
#include "ComsLed.h"
#include "Config.h"
#include "error_codes.h"
#include "Logger.h"
#include "GetBulkChar.h"
#include "uart_commands.h"
#include <math.h>
#include <stdio.h>
#include <utility/ATT.h>
#include <utility/HCI.h>

#define EXOBLE_DEBUG 0

ExoBLE* ExoBLE::_instance = nullptr;

namespace
{
    constexpr size_t kHandshakeChunkSize = 19;

    bool send_handshake_payload(BLECharacteristic &characteristic)
    {
        const char ready_msg[] = "READY";
        characteristic.writeValue(ready_msg);
        delay(20);

        const char *raw_payload = rxBuffer_bulkStr;
        if (raw_payload == nullptr || raw_payload[0] == '\0')
        {
            return false;
        }

        static char sanitized_payload[MAX_MESSAGE_SIZE + 2] = {0};
        size_t write_index = 0;
        sanitized_payload[0] = '\0';

        for (size_t i = 0; raw_payload[i] != '\0' && i < MAX_MESSAGE_SIZE; ++i)
        {
            if (write_index >= (MAX_MESSAGE_SIZE - 1))
            {
                break;
            }

            char c = raw_payload[i];
            if (c == '\r')
            {
                continue;
            }

            sanitized_payload[write_index++] = (c == '\n') ? '|' : c;
        }

        if (write_index >= MAX_MESSAGE_SIZE)
        {
            write_index = MAX_MESSAGE_SIZE - 1;
        }

        sanitized_payload[write_index++] = '\n';
        sanitized_payload[write_index] = '\0';

        size_t offset = 0;
        size_t chunk_count = 0;
        while (offset < write_index)
        {
            size_t chunk_len = ((write_index - offset) > kHandshakeChunkSize) ? kHandshakeChunkSize : (write_index - offset);
            characteristic.writeValue((const uint8_t *)(sanitized_payload + offset), chunk_len);
            delay(20);
            offset += chunk_len;
            chunk_count++;
        }

        #ifdef SIMPLE_DEBUG
        size_t row_count = 0;
        size_t value_row_count = 0;
        bool row_start = true;
        for (size_t i = 0; i < write_index; ++i)
        {
            const char c = sanitized_payload[i];
            if (row_start)
            {
                if (c == 'v')
                {
                    value_row_count++;
                }
                row_start = false;
            }
            if (c == '|')
            {
                row_count++;
                row_start = true;
            }
        }

        Serial.print("\nExoBLE::handshake_payload bytes=");
        Serial.print(write_index);
        Serial.print(" chunks=");
        Serial.print(chunk_count);
        Serial.print(" rows=");
        Serial.print(row_count);
        Serial.print(" value_rows=");
        Serial.print(value_row_count);
        #endif

        return true;
    }
}

namespace ble_rx
{
    static BleParser parser;

    void reset_parser()
    {
        parser.reset();
    }
}

ExoBLE::ExoBLE()
{
    _instance = this;
}

bool ExoBLE::setup()
{
    if (!BLE.begin())
    {
        utils::spin_on_error_with("BLE.begin() failed");
        return false;
    }

    //Setup name and initialize data
    String name = utils::remove_all_chars(BLE.address(), ':');
    name.remove(name.length() - MAC_ADDRESS_NAME_LENGTH);
    name = NAME_PREAMBLE + name;

    //Using exo_info namespace defined in Config.h
    String FirmwareVersion = exo_info::FirmwareVersion; //String to add to firmware char
    String PCBVersion = exo_info::PCBVersion;           //String to add to pcb char
    String DeviceName = exo_info::DeviceName;           //String to add to device char

    //Check if the name is null, if it is use the name above, if not check for preamble
    if (DeviceName == "NULL")
    {
        DeviceName = name;
    }
    else
    {
        //Check if the name has the preamble, if not add it
        if (!DeviceName.startsWith(NAME_PREAMBLE))
        {
            DeviceName = NAME_PREAMBLE + DeviceName;
        }
    }

    //Initialize char arrays
    char name_char[name.length() + 1];
    char firmware_char[FirmwareVersion.length() + 1];
    char pcb_char[PCBVersion.length() + 1];
    char device_char[DeviceName.length() + 1];

    //Add data to array
    name.toCharArray(name_char, name.length() + 1);
    FirmwareVersion.toCharArray(firmware_char, FirmwareVersion.length() + 1);
    PCBVersion.toCharArray(pcb_char, PCBVersion.length() + 1);
    DeviceName.toCharArray(device_char, DeviceName.length() + 1);

    //Create pointer that pointes to array
    const char *k_name_pointer = name_char;
    const char *firmware_pointer = firmware_char;
    const char *pcb_pointer = pcb_char;
    const char *device_pointer = device_char;

    //Set name for device
    BLE.setLocalName(k_name_pointer);
    BLE.setDeviceName(k_name_pointer);

    //Initialize GATT DB
    _gatt_db.FirmwareChar.writeValue(firmware_char);
    _gatt_db.PCBChar.writeValue(pcb_char);
    _gatt_db.DeviceChar.writeValue(device_char);
    send_error(0, 0);

    //Configure services and advertising data
    BLE.setAdvertisedService(_gatt_db.UARTService);

    //UART Chars
    _gatt_db.UARTService.addCharacteristic(_gatt_db.TXChar);
    _gatt_db.UARTService.addCharacteristic(_gatt_db.RXChar);

    //Device Info Chars
    _gatt_db.UARTServiceDeviceInfo.addCharacteristic(_gatt_db.PCBChar);
    _gatt_db.UARTServiceDeviceInfo.addCharacteristic(_gatt_db.FirmwareChar);
    _gatt_db.UARTServiceDeviceInfo.addCharacteristic(_gatt_db.DeviceChar);

    //Error Char
    _gatt_db.ErrorService.addCharacteristic(_gatt_db.ErrorChar);

    BLE.addService(_gatt_db.UARTService);
    BLE.addService(_gatt_db.UARTServiceDeviceInfo);
    BLE.addService(_gatt_db.ErrorService);

    _gatt_db.RXChar.setEventHandler(BLEWritten, ble_rx::on_rx_recieved);


    // When the central subscribes to notifications on TX, deliver the READY handshake and payload
    _gatt_db.TXChar.setEventHandler(BLESubscribed, ExoBLE::_on_tx_subscribed);


    BLE.setConnectionInterval(6, 6);
    advertising_onoff(true);

    return true;
}

void ExoBLE::advertising_onoff(bool onoff)
{
    if (onoff)
    {
        //Start Advertising
        // logger::println("Start Advertising");
        BLE.advertise();

        //Turn the blue led off
        ComsLed *led = ComsLed::get_instance();
        uint8_t r, g, b;
        led->get_color(&r, &g, &b);
        led->set_color(r, g, 0);
    }
    else
    {
        //Stop Advertising
        // logger::println("Stop Advertising");
        BLE.stopAdvertise();

        //Turn the blue led on
        ComsLed *led = ComsLed::get_instance();
        uint8_t r, g, b;
        led->get_color(&r, &g, &b);
        led->set_color(r, g, 255);
    }
}

bool ExoBLE::handle_updates()
{
    #if EXOBLE_DEBUG
        logger::print("ExoBLE::handle_updates:Start");
        logger::print("\n");
    #endif

    static Time_Helper *t_helper = Time_Helper::get_instance();
    static float update_context = t_helper->generate_new_context();
    static float del_t = 0;
    del_t += t_helper->tick(update_context);

    if (del_t > BLE_times::_update_delay)
    {
        del_t = 0;
        #if EXOBLE_DEBUG
            static float poll_context = t_helper->generate_new_context();
            static float poll_time = 0;
            static float connected_context = t_helper->generate_new_context();
            static float connected_time = 0;
        #endif

        //Poll for updates and check connection status
        #if EXOBLE_DEBUG
            logger::print("Poll for updates and check connection status");
            logger::print("\n");
        #endif

        BLE.poll();
        int32_t current_status = BLE.connected();

        if (_connected != current_status)
        {

            // A command fragment can never cross a BLE connection boundary.
            ble_rx::reset_parser();
            _tx_queue.clear();
            _handshake_sent_this_connection = false;
            _handshake_payload_pending = true;

            if (current_status < _connected)
            {
                ble_queue::clear();
            }
            advertising_onoff(current_status == 0);
            _connected = current_status;
        }
        // BLE.poll can dispatch subscribe before the main loop observes the
        // connection. Do not overwrite that event with a false subscription.
        _tx_subscribed = current_status > 0 && _gatt_db.TXChar.subscribed();
    }

    if (_connected > 0 &&
        _tx_subscribed &&
        !_handshake_sent_this_connection &&
        _handshake_payload_pending &&
        rxBuffer_bulkStr[0] != '\0')
    {
        if (send_handshake_payload(_gatt_db.TXChar))
        {
            _handshake_sent_this_connection = true;
            _handshake_payload_pending = false;
        }
    }
    _flush_tx_queue();


    #if EXOBLE_DEBUG
        logger::print("ExoBLE::handle_updates:queue size:");
        logger::print(ble_queue::size());
        logger::print("\n");
    #endif

    return ble_queue::size();
}

bool ExoBLE::send_message(BleMessage &msg)
{
    if (!this->_connected || !_tx_subscribed || !_gatt_db.TXChar.subscribed())
    {
        return false;
    }

    #if EXOBLE_DEBUG
        BleMessage::print(msg);
    #endif

    uint8_t* buffer = _tx_queue.writable_data();
    if (buffer == nullptr)
    {
        logger::println("ExoBLE::send_message transmission queue full", LogLevel::Error);
        return false;
    }
    const int bytes_to_send = _ble_parser.package_raw_data(
        buffer,
        BleTxQueue::MAX_FRAME_BYTES,
        msg);
    if (bytes_to_send <= 0)
    {
        logger::println("ExoBLE::send_message failed to serialize message", LogLevel::Warn);
        return false;
    }

    return _tx_queue.commit(bytes_to_send);
}

void ExoBLE::_flush_tx_queue()
{
    if (!_connected || !_tx_subscribed || !_handshake_sent_this_connection ||
        !_gatt_db.TXChar.subscribed())
    {
        return;
    }
    const uint16_t payload = ATT.notificationPayloadSize();
    const size_t limit = payload < _gatt_db.BUFFER_SIZE ? payload : _gatt_db.BUFFER_SIZE;
    // One lossless chunk per loop; existing MTU-sized telemetry stays a single
    // notification. Credits are checked before ArduinoBLE's blocking send path.
    _tx_queue.flush_one(limit, HCI.canSendAclPkt(ATT.notificationPeerCount()),
        [this](const uint8_t* data, size_t length)
        {
            return _gatt_db.TXChar.writeValue(data, length) > 0;
        });
}

bool ExoBLE::send_controller_snapshot(const UART_msg_t &msg)
{
    const uint8_t param_start = (uint8_t)UART_command_enums::live_controller_params::PARAM_START;
    uint8_t controller_id = 0;
    uint8_t param_count = 0;
    if (!_connected || !_tx_subscribed || !_gatt_db.TXChar.subscribed() ||
        msg.command != UART_command_names::update_live_controller_params ||
        msg.len < param_start || msg.len > UART_MSG_T_MAX_DATA_LEN ||
        !param_update::has_valid_side(msg.joint_id) ||
        !param_update::has_valid_joint_type(msg.joint_id) ||
        !param_update::try_float_to_uint8(
            msg.data[(uint8_t)UART_command_enums::live_controller_params::CONTROLLER_ID], &controller_id) ||
        !param_update::try_float_to_uint8(
            msg.data[(uint8_t)UART_command_enums::live_controller_params::PARAM_LENGTH], &param_count) ||
        msg.data[(uint8_t)UART_command_enums::live_controller_params::CONTROLLER_ID] != (float)controller_id ||
        msg.data[(uint8_t)UART_command_enums::live_controller_params::PARAM_LENGTH] != (float)param_count ||
        param_count > controller_defs::max_parameters || param_count > 30 ||
        msg.len != param_start + param_count)
    {
        return false;
    }

    // Serialize directly into bounded FIFO storage, then commit atomically.
    char* frame = reinterpret_cast<char*>(_tx_queue.writable_data());
    if (frame == nullptr)
    {
        logger::println("ExoBLE::send_controller_snapshot transmission queue full", LogLevel::Error);
        return false;
    }
    const size_t capacity = BleTxQueue::MAX_FRAME_BYTES + 1;
    int written = snprintf(frame, capacity, "S%c%uc%un%un",
        ble_names::live_controller_params, (unsigned)(param_count + 2),
        (unsigned)msg.joint_id, (unsigned)controller_id);
    if (written <= 0 || written >= (int)capacity)
    {
        return false;
    }
    size_t length = (size_t)written;
    for (uint8_t i = 0; i < param_count; ++i)
    {
        const float value = msg.data[param_start + i];
        if (!isfinite(value))
        {
            return false;
        }
        written = snprintf(frame + length, capacity - length, "%.9g", (double)value);
        if (written <= 0 || written > 16 ||
            length + (size_t)written + 1 > BleTxQueue::MAX_FRAME_BYTES)
        {
            return false;
        }
        length += (size_t)written;
        frame[length++] = 'n';
        frame[length] = '\0';
    }

    return _tx_queue.commit(length);
}

void ExoBLE::send_error(int error_code, int joint_id)
{
    if (!this->_connected)
    {
        return; /* Don't bother sending anything if no one is listening */
    }

    #if EXOBLE_DEBUG
        logger::print("Exoble::send_error->Sending: ", LogLevel::Error);
        logger::print(joint_id, LogLevel::Error);
        logger::print(", ", LogLevel::Error);
        logger::print(error_code, LogLevel::Error);
        logger::print("\n");
    #endif

    String error_string = String(error_code) + ":" + String(joint_id);
    
    //Convert to char array
    char error_char[error_string.length() + 1];
    error_string.toCharArray(error_char, error_string.length() + 1);

    _gatt_db.ErrorChar.writeValue(error_char);
}

void ExoBLE::_on_tx_subscribed(BLEDevice /*central*/, BLECharacteristic characteristic)
{
    if (_instance != nullptr)
    {
        _instance->_handle_tx_subscribed(characteristic);
    }
}

void ExoBLE::_handle_tx_subscribed(BLECharacteristic /*characteristic*/)
{
    _tx_subscribed = true;
    // Sending from an ATT callback can re-enter HCI.poll while its receive
    // buffer is still being dispatched. Main-loop polling delivers the preamble.
    _handshake_payload_pending = !_handshake_sent_this_connection;
}

void ble_rx::on_rx_recieved(BLEDevice central, BLECharacteristic characteristic)
{
    (void)central;

    char data[255] = {0};
    int len = characteristic.valueLength();
    if (len > (int)sizeof(data))
    {
        len = sizeof(data);
    }
    characteristic.readValue(data, len);

        #if EXOBLE_DEBUG
            logger::print("On Rx Recieved: ");
            for (int i=0; i<len;i++)
            {
                logger::print(data[i]);
                logger::print(", ");
            }
            logger::print("\n");
        #endif

    BleMessage *msg = parser.handle_raw_data(data, len);
    
    if (msg->is_complete)
    {
        #if EXOBLE_DEBUG
            logger::print("on_rx_recieved->Command: ");
            BleMessage::print(*msg);
        #endif

        ble_queue::push(msg);
    }

    #if EXOBLE_DEBUG
        logger::print("on_rx_recieved->End\n");
    #endif
}

#endif // defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)

/**
 * @file CAN.h
 * @author Chancelor Cuddeback
 * @brief Uses the FlexCan library to send and receive CAN messages.
 * @date 2023-07-18
 * 
 */

#ifndef CAN_H
#define CAN_H

#include "Logger.h"
#include "Arduino.h"

 //Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "FlexCAN_T4.h"
#if defined(ARDUINO_TEENSY36)
    static FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;
#elif defined(ARDUINO_TEENSY41)
    static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
#endif

/**
 * @brief CAN class for sending and receiving CAN messages. Singleton
 * 
 */
class CAN 
{
    public:
        static const uint8_t max_registered_motors = 12;

        //Diagnostics. A failed Can0.write() is otherwise invisible: its only report
        //is a logger::println at LogLevel::Error, which LogLevel::Release suppresses.
        volatile uint32_t tx_ok_count = 0;
        volatile uint32_t tx_fail_count = 0;

        /**
         * @brief Get the Singleton object
         * 
         * @return CAN* 
         */
        static CAN* getInstance()
        {
            static CAN instance;
            return &instance;
        }

        /**
         * @brief Send a CAN message
         * 
         * @param msg CAN_message_t to send
         */
        void send(CAN_message_t msg)
        {
            if(!Can0.write(msg)) 
            {
                tx_fail_count++;
                logger::println("Error Sending" + String(msg.id), LogLevel::Error);
            }
            else
            {
                tx_ok_count++;
            }
        }

        /**
         * @brief Register one motor's exact CAN receive address and format.
         *
         * Registration is performed once while motor objects are constructed.
         * The fixed table avoids heap allocation and supports every left/right
         * OpenExo joint without hard-coding ankle IDs.
         *
         * @return true when the motor owns a receive slot.
         */
        bool register_motor(uint8_t motor_id, bool extended)
        {
            noInterrupts();

            for (uint8_t i = 0; i < max_registered_motors; i++)
            {
                if (_motor_slots[i].registered &&
                    _motor_slots[i].motor_id == motor_id)
                {
                    const bool same_format =
                        _motor_slots[i].extended == extended;
                    interrupts();
                    return same_format;
                }
            }

            for (uint8_t i = 0; i < max_registered_motors; i++)
            {
                if (!_motor_slots[i].registered)
                {
                    _motor_slots[i].motor_id = motor_id;
                    _motor_slots[i].extended = extended;
                    _motor_slots[i].valid = false;
                    _motor_slots[i].registered = true;
                    interrupts();
                    return true;
                }
            }

            interrupts();
            logger::println(
                "CAN receive-slot table is full; motor ID " +
                String(motor_id) + " was not registered.",
                LogLevel::Error);
            return false;
        }

        /**
         * @brief Read only the newest reply addressed to one motor.
         *
         * Reading one motor clears only that motor's slot. It cannot remove a
         * valid frame belonging to the other side or another joint.
         */
        bool read_for(
            uint8_t motor_id,
            bool extended,
            CAN_message_t& msg)
        {
            noInterrupts();

            for (uint8_t i = 0; i < max_registered_motors; i++)
            {
                MotorSlot& slot = _motor_slots[i];
                if (slot.registered &&
                    slot.motor_id == motor_id &&
                    slot.extended == extended)
                {
                    if (!slot.valid)
                    {
                        interrupts();
                        return false;
                    }

                    msg = slot.message;
                    slot.valid = false;
                    interrupts();
                    return true;
                }
            }

            interrupts();
            return false;
        }

    private:
        // OPENEXO_BILATERAL_CAN_PATCH_V1
        // Each registered motor owns one latest-value slot. At 500 Hz this
        // creates no growing queue, no accumulated delay, and no cross-motor
        // destructive reads.
        struct MotorSlot
        {
            uint8_t motor_id;
            bool extended;
            volatile bool registered;
            volatile bool valid;
            CAN_message_t message;
        };

        MotorSlot _motor_slots[max_registered_motors];

        static CAN*& _callback_instance()
        {
            static CAN* instance = nullptr;
            return instance;
        }

        static void _on_receive(const CAN_message_t& msg)
        {
            CAN* instance = _callback_instance();
            if (instance != nullptr)
            {
                instance->_route_received(msg);
            }
        }

        /**
         * @brief ISR-side CAN demultiplexer.
         *
         * AK60v3 feedback uses an extended raw ID whose low byte is the
         * OpenExo motor ID. Legacy standard feedback stores the motor ID in
         * byte zero. Only an exact registered ID and frame format may update a
         * slot.
         */
        void _route_received(const CAN_message_t& msg)
        {
            if (msg.len == 0 || msg.flags.remote)
            {
                return;
            }

            const bool extended =
                static_cast<bool>(msg.flags.extended);
            if ((extended && msg.len != 8) ||
                (!extended && msg.len < 6))
            {
                return;
            }

            const uint8_t motor_id = extended
                ? static_cast<uint8_t>(msg.id & 0xFFU)
                : msg.buf[0];

            for (uint8_t i = 0; i < max_registered_motors; i++)
            {
                MotorSlot& slot = _motor_slots[i];
                if (slot.registered &&
                    slot.motor_id == motor_id &&
                    slot.extended == extended)
                {
                    slot.message = msg;
                    slot.valid = true;
                    return;
                }
            }
        }

        /**
         * @brief Construct a new CAN object and initialize the CAN bus. This is private 
         * because this is a singleton.
         * 
         */
        CAN()
        {
            for (uint8_t i = 0; i < max_registered_motors; i++)
            {
                _motor_slots[i].motor_id = 0;
                _motor_slots[i].extended = false;
                _motor_slots[i].registered = false;
                _motor_slots[i].valid = false;
            }

            Can0.begin();
            Can0.setBaudRate(1000000);

            // Use physically separate receive paths for left and right
            // extended traffic. The side bits are 0x40 (left) and 0x20
            // (right); the remaining low-byte bits identify the exact joint.
            // Two standard mailboxes preserve legacy CAN-motor compatibility.
            Can0.setMaxMB(16);
            Can0.setMB(MB0, RX, EXT);
            Can0.setMB(MB1, RX, EXT);
            Can0.setMB(MB2, RX, STD);
            Can0.setMB(MB3, RX, STD);
            for (uint8_t mb = 4; mb < 16; mb++)
            {
                Can0.setMB(
                    static_cast<FLEXCAN_MAILBOX>(mb),
                    TX,
                    EXT);
            }

            Can0.setMBFilter(REJECT_ALL);
            Can0.setMBUserFilter(MB0, 0x40U, 0x60U);
            Can0.setMBUserFilter(MB1, 0x20U, 0x60U);
            Can0.setMBFilter(MB2, ACCEPT_ALL);
            Can0.setMBFilter(MB3, ACCEPT_ALL);

            _callback_instance() = this;
            for (uint8_t mb = 0; mb < 4; mb++)
            {
                Can0.onReceive(
                    static_cast<FLEXCAN_MAILBOX>(mb),
                    _on_receive);
            }
            Can0.enableMBInterrupts();
        }
};

#endif
#endif

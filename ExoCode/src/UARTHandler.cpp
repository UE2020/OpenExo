#include "UARTHandler.h"
#include "Utilities.h"
#include "Logger.h"
#include "uart_commands.h"

#define MAX_NUM_LEGS 2
#define MAX_NUM_JOINTS_PER_LEG 2 //Current PCB can only do 2 motors per side, if you have made a new PCB, update.
#define UART_DATA_TYPE short int //If type is changes you will need to comment/uncomment lines in pack_float and unpack_float
#define FIXED_POINT_FACTOR 100

// Ordinary Nano -> Teensy payloads are floats; the reverse is fixed-point.
// Live controller snapshots alone use raw float32 in both directions.
#if defined(ARDUINO_ARDUINO_NANO33BLE) || defined(ARDUINO_NANO_RP2040_CONNECT)
#define UART_PACK_FLOATS 1
#define UART_UNPACK_FLOATS 0
#else
#define UART_PACK_FLOATS 0
#define UART_UNPACK_FLOATS 1
#endif

//Set to 1 to enable debug prints
#define DEBUG_UART_HANDLER 0

typedef enum 
{
  COMMAND = 0,
  JOINT_ID = 1,
  DATA_START = 2
} UARTPackingIndex;

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
// Controller staging can deliver several small frames back-to-back. Keep them
// in hardware-assisted storage until the 500 Hz loop polls each frame.
static uint8_t s_uart_rx_extra_buffer[4096];
#endif

UARTHandler::UARTHandler()
{
  //_rx_raw = CircularBuffer_<char>(_rx_raw_buffer, _k_bufferSize);
  MY_SERIAL.begin(UART_BAUD);
  MY_SERIAL.setTimeout(0);
#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
  MY_SERIAL.addMemoryForRead(s_uart_rx_extra_buffer, sizeof(s_uart_rx_extra_buffer));
#endif
}

UARTHandler* UARTHandler::get_instance()
{
    static UARTHandler* instance = new UARTHandler();
    return instance;
}

void UARTHandler::UART_msg(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *buffer)
{
    if (len > UART_MSG_T_MAX_DATA_LEN || (len > 0 && buffer == nullptr))
    {
      logger::println("UARTHandler::UART_msg->Invalid payload", LogLevel::Error);
      return;
    }

    const uint16_t _packed_len =
        _get_packed_length(msg_id, len, joint_id, buffer);
    if (_packed_len > MAX_RX_LEN || _packed_len > UINT8_MAX)
    {
      logger::println("UARTHandler::UART_msg->Payload too large", LogLevel::Error);
      return;
    }

    #if DEBUG_UART_HANDLER
    logger::print("UARTHandler::UART_msg->Packing Bytes: "); logger::println(_packed_len);
    #endif

    uint8_t _byte_data[MAX_RX_LEN] = {0};
    _pack(msg_id, len, joint_id, buffer, _byte_data);

    #if DEBUG_UART_HANDLER
   logger::println("UARTHandler::UART_msg->Packed data:");
   for (int i=0; i<_packed_len; i++)
   {
     logger::print(_byte_data[i]); logger::print(", ");
   }
   logger::println();
    #endif

    _send_packet(_byte_data, static_cast<uint8_t>(_packed_len));
    MY_SERIAL.flush();

    #if DEBUG_UART_HANDLER
   logger::println("UARTHandler::UART_msg->Flushed tx buffer");
    #endif
}

void UARTHandler::UART_msg(UART_msg_t msg)
{
    #if DEBUG_UART_HANDLER
        logger::print("UARTHandler::UART_msg->Sending Message");
        UART_msg_t_utils::print_msg(msg);
    #endif

    UART_msg(msg.command, msg.len, msg.joint_id, msg.data);
}

UART_msg_t UARTHandler::poll(float timeout_us)
{
    static UART_msg_t empty_msg = {0, 0, 0, 0};
    _timeout_us = timeout_us;
    
    uint32_t _available_bytes = check_for_data();
    if (!_available_bytes) {return empty_msg;}

    #if DEBUG_UART_HANDLER
        logger::print("UARTHandler::poll->Bytes Available: "); logger::println(_available_bytes);
    #endif

    uint8_t _msg_buffer[MAX_RX_LEN];
    int _recv_len = _recv_packet(_msg_buffer, MAX_RX_LEN);
    
    if (_recv_len > 0)
    {
      //Add the partial data to the message
      if (_partial_packet_len) 
      {
        //This occurs if there was a timeout during _recv_packet and we have a complete message
        #if DEBUG_UART_HANDLER
            logger::println("UARTHandler::poll->_recv_len > 0 && _partial_packet_len");
        #endif

        if ((_partial_packet_len + _recv_len) > MAX_RX_LEN)
        {
          _reset_partial_packet();
          return empty_msg;
        }

        //Shift _msg_buffer _partial_packet_len bytes to fit the previous partial packet using memmove
        memmove(_msg_buffer + _partial_packet_len, _msg_buffer, _recv_len);
        
        //Copy the partial packet to the beginning of the buffer
        memcpy(_msg_buffer, _partial_packet, _partial_packet_len);

        _recv_len += _partial_packet_len;

       _reset_partial_packet();
     }

      UART_msg_t msg = _unpack(_msg_buffer, _recv_len);

      #if DEBUG_UART_HANDLER
          logger::print("UARTHandler::poll->Got Message: ");
          UART_msg_t_utils::print_msg(msg);
      #endif

      return msg;
    }

    if (_recv_len < 0)
    {
      //This only occurs if there was a timeout during the previous _recv_packet and an end flag before any new data, in this case the partial packet is the full message
      #if DEBUG_UART_HANDLER
        logger::println("UARTHandler::poll->_recv_len < 0");
      #endif

      //Append the _partial packet to the full message
      memcpy(_msg_buffer, _partial_packet, _partial_packet_len);
       
      #if DEBUG_UART_HANDLER
        logger::println("UARTHandler::poll->_msg_buffer after copyting _packed_data: ");
          for (int i=0; i<(_partial_packet_len); i++)
          {
            logger::print(_msg_buffer[i]); logger::print(", ");
          }
          logger::println();
      #endif

      UART_msg_t msg = _unpack(_msg_buffer, _partial_packet_len);

      _reset_partial_packet();

      #if DEBUG_UART_HANDLER
          logger::print("UARTHandler::poll->Got Message: ");
          UART_msg_t_utils::print_msg(msg);
      #endif

      return msg;
     }
    return empty_msg;
}

inline int UARTHandler::check_for_data()
{
    return MY_SERIAL.available();
}

void UARTHandler::_pack(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *data, uint8_t *data_to_pack)
{
    //Pack metadata
    data_to_pack[COMMAND] = msg_id;
    data_to_pack[JOINT_ID] = joint_id;
    
    const bool raw_floats = UART_PACK_FLOATS ||
        msg_id == UART_command_names::update_live_controller_params;
    const uint8_t num_bytes = raw_floats ? sizeof(float) : sizeof(UART_DATA_TYPE);
    uint8_t buf[sizeof(UART_DATA_TYPE)];
    for (int i = 0; i < len; ++i)
    {
        const uint8_t offset = DATA_START + num_bytes * i;
        if (raw_floats)
        {
            memcpy(data_to_pack + offset, &data[i], sizeof(float));
        }
        else
        {
            utils::float_to_short_fixed_point_bytes(data[i], buf, FIXED_POINT_FACTOR);
            memcpy(data_to_pack + offset, buf, sizeof(buf));
        }
    }
}

UART_msg_t UARTHandler::_unpack(uint8_t* data, uint8_t len)
{
    UART_msg_t msg = {0, 0, {0}, 0};
    if (data == nullptr || len < DATA_START)
    {
        return msg;
    }

    const bool raw_floats = UART_UNPACK_FLOATS ||
        data[COMMAND] == UART_command_names::update_live_controller_params;
    const uint8_t _bytes_per = raw_floats ? sizeof(float) : sizeof(UART_DATA_TYPE);
    const uint8_t _payload_bytes = len - DATA_START;
    if ((_payload_bytes % _bytes_per) != 0)
    {
        return msg;
    }

    const uint8_t _payload_count = _payload_bytes / _bytes_per;
    if (_payload_count > UART_MSG_T_MAX_DATA_LEN)
    {
        return msg;
    }

    msg.command = data[COMMAND];
    msg.joint_id = data[JOINT_ID];
    msg.len = _payload_count;

    //Fill msg.data, converting the payload to floats as needed.
    for (int i=0; i<msg.len; i++)
    {
        uint8_t _data_offset = DATA_START + (i * _bytes_per);
        if (raw_floats)
        {
            memcpy(&msg.data[i], data + _data_offset, sizeof(float));
        }
        else
        {
            utils::short_fixed_point_bytes_to_float(data + _data_offset, &msg.data[i], FIXED_POINT_FACTOR);
        }
    }

    return msg;
}

uint16_t UARTHandler::_get_packed_length(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *data)
{
    uint16_t _val = 0;
    const bool raw_floats = UART_PACK_FLOATS ||
        msg_id == UART_command_names::update_live_controller_params;
    _val += static_cast<uint16_t>(len) *
        (raw_floats ? sizeof(float) : sizeof(UART_DATA_TYPE));
    _val += sizeof(msg_id);
    _val += sizeof(joint_id); 
    return _val;
}


void UARTHandler::_send_char(uint8_t val)
{
  #if DEBUG_UART_HANDLER
      logger::print("UARTHandler::_send_char->Sending: 0x");
      logger::println(val);
  #endif

  MY_SERIAL.write(val);
}

uint8_t UARTHandler::_recv_char(void)
{  
  uint8_t _data = MY_SERIAL.read();

  #if DEBUG_UART_HANDLER
    logger::print("UARTHandler::_recv_char->Read: "); logger::println(_data);
  #endif

  return _data;
}

/* SEND_PACKET: sends a packet of length "len", starting at location "p". */
void UARTHandler::_send_packet(uint8_t* p, uint8_t len)
{
  /* Send an initial END character to flush out any data that may have accumulated in the receiver due to line noise */
  _send_char(END);

  /* For each byte in the packet, send the appropriate character sequence */
  while (len--) {
    switch (*p) {
      /* If it's the same code as an END character, we send a special two character code so as not to make the receiver think we sent an END */
      case END:
        _send_char(ESC);
        _send_char(ESC_END);
        break;

      /* If it's the same code as an ESC character, we send a special two character code so as not to make the receiver think we sent an ESC */
      case ESC:
        _send_char(ESC);
        _send_char(ESC_ESC);
        break;

      /* Otherwise, we just send the character */
      default:
        //logger::print("UARTHandler::_send_packet->Sending: 0x"); logger::println(*p);
        _send_char(*p);
    }

    p++;
  }

  /* Tell the receiver that we're done sending the packet */
  _send_char(END);
}

/* RECV_PACKET: receives a packet into the buffer located at "p".
           If more than len bytes are received, the packet will
           be truncated.
           Returns the number of bytes stored in the buffer.
*/
int UARTHandler::_recv_packet(uint8_t *p, uint8_t len)
{
  uint8_t c;
  int received = 0;
  
  _time_left(1);
  while (_time_left())
  {
    if (!check_for_data()) 
    {
      continue;
    }

    c = _recv_char();

    #if DEBUG_UART_HANDLER
        logger::print("UARTHandler::_recv_packet->Got char: ");
        logger::println(c);
    #endif

    if (_discard_until_end)
    {
      if (c == END)
      {
        _discard_until_end = false;
        _escape_pending = false;
        _reset_partial_packet();
        received = 0;
      }
      continue;
    }

    if (_escape_pending)
    {
      _escape_pending = false;
      if (c == ESC_END)
      {
        c = END;
      }
      else if (c == ESC_ESC)
      {
        c = ESC;
      }
      else
      {
        // A malformed escape invalidates the whole frame. Do not accept a
        // shifted payload that could become a valid but incorrect command.
        _reset_partial_packet();
        received = 0;
        _discard_until_end = (c != END);
        continue;
      }
    }
    else if (c == ESC)
    {
      // The escaped byte may arrive in a later poll. Preserve this state
      // instead of reading -1 from an empty serial buffer.
      _escape_pending = true;
      continue;
    }
    else if (c == END)
    {
        #if DEBUG_UART_HANDLER
            logger::println("UARTHandler::_recv_packet->END CASE");
        #endif

        if (received)
        {
            #if DEBUG_UART_HANDLER
                logger::print("UARTHandler::_recv_packet->Returning: ");
                logger::println(received);
            #endif

            return received;
        }
        else if (_partial_packet_len)
        {
            #if DEBUG_UART_HANDLER
                logger::print("UARTHandler::_recv_packet->Returning because of _partial_packet: ");
                logger::println(_partial_packet_len);
            #endif

            return -1;
        }
        else
        {   
            continue;
        }
    }

    if (received < len)
    {
      p[received++] = c;
    }
    else
    {
      // Never accept a truncated prefix as a complete command.
      _reset_partial_packet();
      received = 0;
      _discard_until_end = true;
    }
  }

  // A timeout may split a valid SLIP frame. Preserve only the bytes received
  // during this poll and reject any frame that cannot fit in the buffer.
  const int prior_packet_len = _partial_packet_len;
  if ((prior_packet_len + received) > MAX_RX_LEN)
  {
    _reset_partial_packet();
    _discard_until_end = true;
    return 0;
  }
  _partial_packet_len = prior_packet_len + received;

  #if DEBUG_UART_HANDLER
      logger::println("UARTHandler::_recv_packet->Timeout!");
      logger::print("UARTHandler::_recv_packet->Saved Bytes: "); 
      logger::print("Prior Packet Length: "); 
      logger::print(prior_packet_len);
      logger::print("\t");
      logger::print("Received: ");
      logger::print(received);
      logger::print("\t");
      logger::print("Partial Packet Length: ");
      logger::println(_partial_packet_len);
  #endif

  for (int i = 0; i < received; i++)
  {
    _partial_packet[i + prior_packet_len] = p[i];

    #if DEBUG_UART_HANDLER
        logger::print(_partial_packet[i + prior_packet_len]); logger::println(", ");
    #endif
  }

  #if DEBUG_UART_HANDLER
    logger::println();
  #endif

  return 0;
}


uint8_t UARTHandler::_time_left(uint8_t should_latch)
{
    static uint32_t start_time_us = 0;
    if (should_latch)
    {
      start_time_us = micros();

      #if DEBUG_UART_HANDLER
          logger::print("UARTHandler::_time_left->Latching on ");
          logger::println(start_time_us);
      #endif
    }

    const uint32_t elapsed_us =
        static_cast<uint32_t>(micros() - start_time_us);
    const uint32_t timeout_us =
        (_timeout_us > 0.0f)
            ? static_cast<uint32_t>(_timeout_us)
            : 0U;
    return elapsed_us <= timeout_us;
}

void UARTHandler::_reset_partial_packet()
{
  memset(_partial_packet, 0, _partial_packet_len);
  _partial_packet_len = 0;
  _escape_pending = false;
  _discard_until_end = false;
}

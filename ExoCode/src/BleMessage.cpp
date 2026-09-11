#include "BleMessage.h"
#include "Logger.h"

BleMessage::BleMessage() 
{ 
    this->clear();
}

void BleMessage::clear() 
{
    command = 0;
    is_complete = false;
    _size = 0;
    expecting = 0;
    for (int i = 0; i < MAX_DATA; i++)
    {
        data[i] = 0;
    }
}

void BleMessage::copy(BleMessage* n) 
{
    if (n == nullptr || n->expecting < 0 || n->expecting > MAX_DATA)
    {
        clear();
        return;
    }

    clear();
    command = n->command; 
    is_complete = n->is_complete;
    _size = n->_size;
    expecting = n->expecting;
    for (int i = 0; i < expecting; i++)
    {
        data[i] = (n->data[i]);
    }  
}

void BleMessage::print(BleMessage msg)
{
    logger::print(msg.command);
    logger::print("\t");
    logger::print(msg.is_complete);
    logger::print("\t");
    logger::print(msg.expecting);
    logger::print("\n");
    if (msg.expecting <= 0) 
    {
        return;
    }
    const int values_to_print =
        (msg.expecting > MAX_DATA) ? MAX_DATA : msg.expecting;
    for (int i = 0; i < values_to_print; i++)
    {
        logger::print(msg.data[i]);
        if (i == (values_to_print - 1))
        {
            continue;
        }
        logger::print(", ");
    }
    logger::print("\n");
}

int BleMessage::matching(BleMessage msg1, BleMessage msg2)
{
    int doesnt_match = 0;
    doesnt_match += msg1.command != msg2.command;
    doesnt_match += msg1.is_complete != msg2.is_complete;
    doesnt_match += msg1.expecting != msg2.expecting;
    doesnt_match += msg1._size != msg2._size;
    return (doesnt_match == 0) ? (1):(0);
}

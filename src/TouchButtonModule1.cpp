#include "Boards/BrytecBoard.h"

#include "Can.h"
#include "Deserializer/BinaryProgmemDeserializer.h"
#include "TouchButtonModule1.h"
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "Program.h"

namespace Brytec {

BinaryProgmemDeserializer des(nullptr, 0);

BinaryDeserializer* BrytecBoard::getDeserializer()
{
    des = Brytec::BinaryProgmemDeserializer(progmem_data, sizeof(progmem_data));
    return &des;
}

void BrytecBoard::error(EBrytecErrors error)
{
}

void BrytecBoard::setupBrytecCan(uint32_t mask, uint32_t filter)
{
    Can::Init(B0, CAN500kBPS, CAN_ID_EXT, mask, filter);
}

void BrytecBoard::setupPin(uint16_t index, IOTypes::Types type)
{
    switch (index) {
    case BT_PIN_Touch_1:
        break;
    case BT_PIN_Touch_2:
        break;
    }
}

void BrytecBoard::shutdownAllPins()
{
    // Not needed
}

float BrytecBoard::getPinValue(uint16_t index)
{
    switch (index) {
    case BT_PIN_Touch_1:
        return 0.0f;
    case BT_PIN_Touch_2:
        return 0.0f;
    }

    return 0.0f;
}

float BrytecBoard::getPinVoltage(uint16_t index)
{
    // Not supported
    return 0.0f;
}

float BrytecBoard::getPinCurrent(uint16_t index)
{
    // Not needed
    return 0.0f;
}

void BrytecBoard::setPinValue(uint16_t index, IOTypes::Types type, float value)
{
    // Not needed
}

void BrytecBoard::sendBrytecCan(CanExtFrame frame)
{
    cli();
    Can::SendCanMsg(frame);
    sei();
}

void BrytecBoard::ReserveConfigSize(uint16_t size)
{
    // Not supported yet
}

void BrytecBoard::updateConfig(uint8_t* data, uint32_t size, uint32_t offset)
{
    // Not supported yet
}
}
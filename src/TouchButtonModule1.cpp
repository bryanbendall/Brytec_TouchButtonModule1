#include "Boards/BrytecBoard.h"

#include "Can.h"
#include "Deserializer/BinaryProgmemDeserializer.h"
#include "TouchButtonModule1.h"
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "Program.h"

unsigned long timeoutCount = 1000000;
unsigned long pin4baseline = 0;
unsigned long pin5baseline = 0;
static constexpr uint32_t samples = 50;

int32_t CapReadPin4()
{
    uint32_t totalCharge = 0;

    for (uint32_t i = 0; i < samples; i++) {

        DDRF |= (1 << PINF6); // Make output
        PORTF &= ~(1 << PINF6); // Set low
        _delay_ms(1);

        DDRF &= ~(1 << PINF6); // Make input
        while (!(PINF & (1 << PINF6)) && (totalCharge < timeoutCount))
            totalCharge++;
    }

    if ((totalCharge - pin4baseline) <= 0)
        return 0;
    else
        return (totalCharge - pin4baseline);
}

int32_t CapReadPin5()
{
    uint32_t totalCharge = 0;

    for (uint32_t i = 0; i < samples; i++) {

        DDRD |= (1 << PIND4); // Make output
        PORTD &= ~(1 << PIND4); // Set low
        _delay_ms(1);

        DDRD &= ~(1 << PIND4); // Make input
        while (!(PIND & (1 << PIND4)) && (totalCharge < timeoutCount))
            totalCharge++;
    }

    if ((totalCharge - pin5baseline) <= 0)
        return 0;
    else
        return (totalCharge - pin5baseline);
}

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
    static bool first = true;
    if (first) {
        pin4baseline = CapReadPin4();
        pin5baseline = CapReadPin5();
        first = false;
    }

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
    case BT_PIN_Touch_1: {
        int32_t pin4reading = CapReadPin4();
        if (pin4reading >= 50)
            return 1.0f;
    }
    case BT_PIN_Touch_2: {
        int32_t pin5reading = CapReadPin5();
        if (pin5reading >= 50)
            return 1.0f;
    }
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
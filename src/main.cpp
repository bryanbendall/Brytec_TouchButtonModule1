#include "Can.h"
#include <EBrytecApp.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

Brytec::EBrytecApp app;

// Variables
volatile uint8_t preTimer = 0;
volatile uint64_t milli = 0;
bool timerState = false;
static uint64_t lastMillis = 0;

void setupAdc()
{
    // Setup ADC
    ADMUX |= (1 << REFS0); // External reference voltage
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC
}

void setupTimers()
{
    // Setup Timers for PWM
    // Timer Counter 0 A and B
    TCCR0A |= (1 << WGM01) | (1 << WGM00); // PWM Mode
    TCCR0B |= (1 << CS01); // Clock set to clk/8
    // Timer Counter 1 A and B
    TCCR1A |= (1 << WGM10);
    TCCR1B |= (1 << WGM12) | (1 << CS11);
    // Timer Counter 3 A
    TCCR3A |= (1 << WGM30);
    TCCR3B |= (1 << WGM32) | (1 << CS31);
    // Timer Counter 4 A and D
    TCCR4A |= (1 << PWM4A); // PWM on 4A
    TCCR4C |= (1 << PWM4D); // PWM on 4D
    TCCR4B |= (1 << CS42); // Clock set to clk/8
    OCR4C = 0xFF;

    // Timer Setup
    TIMSK3 |= (1 << TOIE3); // Timer 3 overflow interrupt

    sei();
}

ISR(TIMER3_OVF_vect)
{
    preTimer++;
    if (preTimer >= 7) {
        preTimer = 0;
        milli++;
    }
}

ISR(INT1_vect)
{
    Brytec::CanExtFrame frame = Can::GetCanMsg();
    app.brytecCanReceived(frame);
}

int main()
{
    // Test for rev 1 board
    // DDRE |= (1 << PINE6);
    // DDRF &= ~(1 << PINF5);
    ///////////////////

    setupAdc();
    setupTimers();

    app.deserializeModule();

    // if (EBrytecApp::isDeserializeOk())
    //     Serial::SendString("Des succ", 8, true);
    // else
    //     Serial::SendString("Des fail", 8, true);

    app.setMode(Brytec::EBrytecApp::Mode::Normal);

    while (1) {

        uint64_t difference = milli - lastMillis;
        if (difference > 1) {
            float timestep = ((float)difference * 0.001f);
            lastMillis = milli;

            app.update(timestep);
        }
    }
}

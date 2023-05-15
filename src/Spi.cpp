#include "Spi.h"

void Spi::SpiInit(int ssPin)
{
    slavePin = ssPin;

    switch (slavePin) { // Slave Select Output
    case B0:
        DDRB |= (1 << PINB0);
        break;
    case B4:
        DDRB |= (1 << PINB4);
        break;
    case B5:
        DDRB |= (1 << PINB5);
        break;
    }
    SSDisable();

    DDRB |= (1 << PINB2) | (1 << PINB1); // Set MOSI and SCK output
    SPCR |= (1 << SPE) | (1 << MSTR); // Enable SPI, Master
    SPSR |= (1 << SPI2X); // clock rate fck/2
}

char Spi::SpiTransfer(char data)
{
    SPDR = data;
    while (!(SPSR & (1 << SPIF)))
        ;
    return SPDR;
}

void Spi::SSEnable()
{
    switch (slavePin) {
    case B0:
        PORTB &= ~(1 << PINB0); // SS low
        break;
    case B4:
        PORTB &= ~(1 << PINB4); // SS low
        break;
    case B5:
        PORTB &= ~(1 << PINB5); // SS low
        break;
    }
}

void Spi::SSDisable()
{
    switch (slavePin) {
    case B0:
        PORTB |= (1 << PINB0); // SS high
        break;
    case B4:
        PORTB |= (1 << PINB4); // SS high
        break;
    case B5:
        PORTB |= (1 << PINB5); // SS high
        break;
    }
}
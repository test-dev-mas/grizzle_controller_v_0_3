#include <avr/io.h>

#include "multimeter_click.h"

void multimeter_init() {
    DDRD |= (1 << CS);
    PORTD |= (1 << CS);

    DDRB |= (1 << CLK) | (1 << D_IN);
    // PORTB |= (1 << D_IN);
}

static uint16_t multimeter_read_channel(uint8_t channel) {
    uint8_t write_data;
    // uint8_t read_buf[2];
    // uint16_t ret = -2045;               // code from previous lib
    uint16_t ret = 0;

    write_data = 0x60;                  // nNULL:NULL:START:SINGLE/DIFF:D2:D1:D0:S/H:NULL (0b00110000) everything is in initial state
    write_data |= (channel<<2);         // set channel bits:D2/D1/D0

    PORTD &= ~(1 << CS);                // bringing CS low initiates communication with MCP3204
    // PORTB |= (1 << CLK);                // first clock received with CS low and DIN high will constitute a start bit

    for (int i=6;i>=0;i--) {
        if (write_data&(1<<i)) {
            PORTB |= (1 << D_IN);       // write 1
        }
        else {
            PORTB &= ~(1 << D_IN);      // write 0
        }
        PORTB |= (1 << CLK);            // toggle clock
        PORTB &= ~(1 << CLK);
    }

    for (int i=11;i>=0;i--) {
        if (PINB & (1 << D_OUT)) {
            ret |= (1 << i);
            // ret += (1 << i);            // code from previous lib
        }
        PORTB |= (1 << CLK);            // toggle clock
        PORTB &= ~(1 << CLK);
    }

    PORTD |= (1 << CS);

    return ret;
}

uint16_t multimeter_read_voltage() {
    return multimeter_read_channel(MULTIMETER_U_CHANNEL);
}
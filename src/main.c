
/*
 * Irrigation system based on Atmega328P (Arduino Uno, Arduino Nano).
 *
 * pin 2 (PD2) - instant power for tank
 * pin 3 (PD3) - sensor for tank, should be pulled down with 10k resistor
 * pin 4 (PD4) - instant power for plant
 * pin 5 (PD5) - sensor for plant, should be pulled down with 10k resistor
 * pin 8 (PB0) - relay -> motor
 *
 */


#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stdint.h>

#define DEBUG  // uncomment this for debugging

/*
 * global helpers, shorthands
 */
#define set_bit(reg, bit_num) ((reg) |= (1 << (bit_num)))
#define clear_bit(reg, bit_num) ((reg) &= ~(1 << (bit_num)))
#define enable_wd_interrupt() (WDTCSR |= (1 << WDIE))
#ifdef DEBUG
    // toggle led on the board
    #define blink() (PINB |= (1 << PINB5))
#endif

/*
 * pin assignment
 *  and mapping
 */

#define PIN_TANK_POWER    //2
#define PIN_TANK_SENSOR   3
#define PIN_PLANT_POWER   4
#define PIN_PLANT_SENSOR  5
#define PIN_MOTOR_CONTROL   8

#define set_pin__tank_power
#define pin_tank_power_get
#define pin_tank_power_clear

/* ...

TODO: make a mapping to pins!
*/


/*
 * variables allocation
 */

// global counter
uint16_t power_down_counter = 0;  // count number of sleep instructions (power down)
#ifdef DEBUG
    const uint16_t power_down_idle_length = 1;
#else
    // 60[seconds in minute] * 60[minutes in hour]
    // 8s is the WDT timeout (from prescaler)
    const uint16_t power_down_idle_length = (60 * 60) / 8;
#endif

// bottle with a water (tank)
bool enough_water_in_tank = false;  // why false? safety

// if any water in the saucer
bool water_in_saucer = true;  // safety

// motor
bool motor_enabled = true;  // safety


/*
 * functions
 */

static inline void check_water_level_in_the_bottle(void) {
    set_bit(PORTD, PORTD2);  // pin 2 to HIGH
    _NOP();
    enough_water_in_tank = bit_is_set(PIND, PIND3);
    clear_bit(PORTD, PORTD2);  // pin 2 to LOW
}

static inline void check_water_in_saucer(void) {
    set_bit(PORTD, PORTD4);  // pin 4 to HIGH
    _NOP();
    water_in_saucer = bit_is_set(PIND, PIND5);
    clear_bit(PORTD, PORTD4);  // pin 4 to LOW
}

static inline void enable_motor(void) {
    if (!motor_enabled) {
        set_bit(PORTB, PORTB0);
        motor_enabled = true;
    }
}

static inline void disable_motor(void) {
    if (motor_enabled) {
        clear_bit(PORTB, PORTB0);
        motor_enabled = false;
    }
}


/*
 * interrupt handlers
 */

ISR(WDT_vect) {
    WDTCSR |= (1 << WDIE);
}


int main(void) {
    // disable interrupts
    cli();


    /*
     * Disable all that is possible to disable :)
     */
    // disable ADC
    power_adc_disable();

    // disable timers
    power_timer0_disable();
    power_timer1_disable();
    power_timer2_disable();

    // disable SPI
    power_spi_disable();

    // disable I2C
    power_twi_disable();

    // disable USART
    power_usart0_disable();

    // configure WD
#ifdef DEBUG
    wdt_enable(WDTO_1S);
#else
    wdt_enable(WDTO_8S);
#endif
    set_bit(WDTCSR, WDIE);  // enable WD interrupt, disable reset on timeout


    /*
     * configure pins
     */
    set_bit(DDRB, DDB5);  // pin 13 as output (led)
    clear_bit(PORTB, PORTB5);  // disable

    set_bit(DDRD, DDD2);  // pin 2 as output (tank power)
    clear_bit(PORTD, PORTD2);  // low

    clear_bit(DDRD, DDD3);  // pin 3 as input (tank sensor)
    clear_bit(PORTD, PORTD3);  // tri-state, pulled down

    set_bit(DDRD, DDD4);  // pin 4 as output (plant power)
    clear_bit(PORTD, PORTD4);  // low

    clear_bit(DDRD, DDD5);  // pin 5 as input (plant sensor)
    clear_bit(PORTD, PORTD5);  // tri-state, pulled down

    set_bit(DDRB, DDB0);  // pin 8 as output (motor control)
    disable_motor();


    // configure and enable sleep mode
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // power down - the most efficient
    sleep_enable();  // enable possibility to sleep

    // configure system clock prescaler
    // (does not affect WD prescaler)
    clock_prescale_set(clock_div_256);

    // enable interrupts
    sei();



    while (true) {

        // when `power_down_counter` is 0 (power on) this code should be run
        // and when the counter reaches the value of `power_down_idle_length`
        if (power_down_counter == 0 || power_down_counter == power_down_idle_length) {

            check_water_level_in_the_bottle();
            check_water_in_saucer();



            if (enough_water_in_tank && !water_in_saucer) {
                enable_motor();

                /*
                 * TODO
                 * - configure timer
                 * - configure interrupt
                 * - check tank and plant water levels
                 * - decide to disable or not the motor
                 * - introduce the timeout for motor in enabled state (disable it once the timeout is reached)
                 *
                 */



            } else {
                disable_motor();
                blink();
            }

            power_down_counter = 0;
        }

        power_down_counter ++ ;

        // enter sleep mode
        sleep_cpu();
    }

}













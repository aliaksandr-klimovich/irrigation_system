
/*
 * Irrigation system based on Atmega328P (Arduino Uno, Arduino Nano).
 *
 * This is a single-file program with detailed comments and explanations.
 * The system does next steps:
 *  - Checks the tank with water if there is anough water to irrigate the plant.
 *  - Check the pot (plant) saucer if it contains water (if there is a water in the saucer, the plant is irrigated). 
 *  - Tries to irrigate the plant using the motor (water pump) control. This is a multiple step procedure. 
 *      The motor is enabled for some amount of time and then goes to disabled state to give a water 
 *      to drain into the saucer; this cycle is repeated for defined number of times or till the water
 *      is present in the saucer.
 *  - Sleeps for very long time using a watchdog (WD) timeout and power down sleep mode.
 *  - Alerts the user if there is not enough water in the tank.
 *
 * +-----+----------+--------------+--------------------------------------------------------------------+
 * | IDE | Port pin | Physical pin | Assignment description                                             |
 * +-----+----------+--------------+--------------------------------------------------------------------+
 * | 2   | PD2      | 4            | Instant power for tank                                             |
 * | 3   | PD3      | 5            | Sensor for tank, should be pulled down with 10k resistor           |
 * | 4   | PD4      | 6            | Instant power for plant                                            |
 * | 5   | PD5      | 7            | Sensor for plant, should be pulled down with 10k resistor          |
 * | 8   | PB0      | 14           | Relay -> motor control pin                                         |
 * +-----+----------+--------------+--------------------------------------------------------------------+
 */


/*
 * Include
 *   libraries
 */

#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>


/*
 * Configuration
 */

// Set debug mode. Comment this after debug.
//#define DEBUG

/*
 * PIN, PORT, DDR mapping.
 *   Link it with <avr/io.h> library.
 */
#define TANK_POWER_PIN_ADDR         PIND
#define TANK_POWER_PIN_NUM          PIND2
#define TANK_POWER_PORT_ADDR        PORTD
#define TANK_POWER_PORT_NUM         PORTD2
#define TANK_POWER_DDR_ADDR         DDRD
#define TANK_POWER_DDR_NUM          DDD2

#define TANK_SENSOR_PIN_ADDR        PIND
#define TANK_SENSOR_PIN_NUM         PIND3
#define TANK_SENSOR_PORT_ADDR       PORTD
#define TANK_SENSOR_PORT_NUM        PORTD3
#define TANK_SENSOR_DDR_ADDR        DDRD
#define TANK_SENSOR_DDR_NUM         DDD3

#define PLANT_POWER_PIN_ADDR        PIND
#define PLANT_POWER_PIN_NUM         PIND4
#define PLANT_POWER_PORT_ADDR       PORTD
#define PLANT_POWER_PORT_NUM        PORTD4
#define PLANT_POWER_DDR_ADDR        DDRD
#define PLANT_POWER_DDR_NUM         DDD4

#define PLANT_SENSOR_PIN_ADDR        PIND
#define PLANT_SENSOR_PIN_NUM         PIND5
#define PLANT_SENSOR_PORT_ADDR       PORTD
#define PLANT_SENSOR_PORT_NUM        PORTD5
#define PLANT_SENSOR_DDR_ADDR        DDRD
#define PLANT_SENSOR_DDR_NUM         DDD5

#define MOTOR_CONTROL_PIN_ADDR        PINB
#define MOTOR_CONTROL_PIN_NUM         PINB0
#define MOTOR_CONTROL_PORT_ADDR       PORTB
#define MOTOR_CONTROL_PORT_NUM        PORTB0
#define MOTOR_CONTROL_DDR_ADDR        DDRB
#define MOTOR_CONTROL_DDR_NUM         DDB0


/*
 * Global helpers, shorthands
 */

// Set bit in register
#define set_bit(reg, bit_num)     ( (reg) |=  (1 << (bit_num)) )

// Clear bit in register
#define clear_bit(reg, bit_num)   ( (reg) &= ~(1 << (bit_num)) )

// Check if bit is set in register
#define is_bit_set(reg, bit_num)  ( (reg) &   (1 << (bit_num)) )

// Set WDIE bit in WDTCSR register to enable WD interrupt.
// WD interrupt clears this bit (see ref. man.), so need to set it again.
#define enable_wd_interrupt()  (WDTCSR |= (1 << WDIE))

// All sleep operations will use WD interrupt for wake up and deep sleep mode for sleeping.
#define sleep(timeout)          \
do {                            \
    wdt_enable(timeout);        \
    enable_wd_interrupt();      \
    sleep_cpu();                \
    wdt_disable();              \
} while(0);

// Toggle built-in LED on the board. This is a debug function.
#define toggle_led()  (PINB |= (1 << PINB5))  // Hard code pin B5 as it is a build-in LED on the board.
#define blink()  do { toggle_led(); for(uint8_t i=0; i<255; i++); toggle_led(); } while(0);


/*
 * Global variables
 */

#ifdef DEBUG
const uint8_t irrigation_cycles = 2;  // 2 * 0.5 = 1 [s]
const uint8_t wait_water_in_saucer_cycles = 1;  // 1 * 1 = 1 [s]
const uint8_t total_irrigation_cycles = 2;  // (1 + 1) * 2 - 1 = 3 [s]
const uint16_t power_down_wait_time = 2 / 1;  // 8[s]
#else
const uint8_t irrigation_cycles = 8;  // 8 * 0.5 = 4 [s]
const uint8_t wait_water_in_saucer_cycles = 2;  // 2 * 8 = 16 [s]
const uint8_t total_irrigation_cycles = 5;  // (4 + 16) * 5 = 100 [s]
const uint16_t power_down_wait_time = 4 * 60 * 60 / 8;  // 4[h]
#endif

uint8_t irrigation_counter;
uint8_t wait_water_in_saucer_counter;
uint8_t total_irrigation_cycles_counter;

/*
 * Bottle with a water (tank).
 * Set to worst case (`false`) to let the system change it.
 */
bool enough_water_in_tank = false;

/*
 * If any water in the saucer.
 * Set to worst case (`true`) to let the system change it.
 */
bool water_in_saucer = true;

/*
 * Motor status.
 *   Indicate if motor is enabled or not. True means enabled.
 *   Expected that the system configures the motor control pin to have motor disabled at startup.
 *   `disable_motor` function is called from the configuration. It checks this variable and decides
 *   if the motor needs to be enabled or disabled. So, to disable the motor, it should be
 *   enabled previously, that's why the initial state of this variable is `true`.
 */
bool motor_enabled = true;

/*
 * State machine.
 *   Next modes are defined. This list can be extended with other modes if it is required.
 */ 
enum mode_t {
    MODE_CHECK,
    MODE_IRRIGATE,
    MODE_WAIT,
    MODE_ALERT,
    MODE_DEBUG,
};
// Set initial mode
uint8_t mode = MODE_CHECK;  

/*
 * Functions to check the tank and saucer. 
 *   The main idea here: 
 *    - Set the power (output) pin to let the current to flow to input pin.
 *    - Wait settling time.
 *    - Check (read) the sensor (input) pin and save its stete.
 *    - Remove the power.
 */
#define check_tank()                                                                \
do {                                                                                \
    set_bit(TANK_POWER_PORT_ADDR, TANK_POWER_PORT_NUM);                             \
    _NOP();                                                                         \
    enough_water_in_tank = is_bit_set(TANK_SENSOR_PIN_ADDR, TANK_SENSOR_PIN_NUM);   \
    clear_bit(TANK_POWER_PORT_ADDR, TANK_POWER_PORT_NUM);                           \
} while(0);

#define check_saucer()                                                              \
do {                                                                                \
    set_bit(PLANT_POWER_PORT_ADDR, PLANT_POWER_PORT_NUM);                           \
    _NOP();                                                                         \
    water_in_saucer = is_bit_set(PLANT_SENSOR_PIN_ADDR, PLANT_SENSOR_PIN_NUM);      \
    clear_bit(PLANT_POWER_PORT_ADDR, PLANT_POWER_PORT_NUM);                         \
} while(0);

#define enable_motor()                                                              \
do {                                                                                \
    if (!motor_enabled) {                                                           \
        set_bit(MOTOR_CONTROL_PORT_ADDR, MOTOR_CONTROL_PORT_NUM);                   \
        motor_enabled = true;                                                       \
    }                                                                               \
} while(0);

#define disable_motor()                                                             \
do {                                                                                \
    if (motor_enabled) {                                                            \
        clear_bit(MOTOR_CONTROL_PORT_ADDR, MOTOR_CONTROL_PORT_NUM);                 \
        motor_enabled = false;                                                      \
    }                                                                               \
} while(0);


/*
 * Interrupt handlers
 */

// Watchdog time-out interrupt
ISR(WDT_vect) {
    // Do nothing
    // Do not set WDIE bit in WDTCSR register here to enable WD interrupt.
    //   It is prohibited for safety reason. See ref. man. 
}


int main(void) {

    /*
     * Startup code
     */

    // Disable all interrupts... Oh, paranoia... All interrupts are not configured at startup.
    cli();

    // Configure and enable sleep mode
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // power down - the most efficient for power saving
    sleep_enable();  // enable possibility to sleep using configured mode

    /*
     * Disable all that it is possible to disable.
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


    /*
     * Configure pins
     */

    // Built-in LED
    set_bit(DDRB, DDB5);  // IDE 13 as output
    clear_bit(PORTB, PORTB5);  // Switch off

    // Tank instant power
    set_bit(TANK_POWER_DDR_ADDR, TANK_POWER_DDR_NUM);  // output
    clear_bit(TANK_POWER_PORT_ADDR, TANK_POWER_PORT_NUM);  // low

    // Tank sensor
    clear_bit(TANK_SENSOR_DDR_ADDR, TANK_SENSOR_DDR_NUM);  // input
    clear_bit(TANK_SENSOR_PORT_ADDR, TANK_SENSOR_PORT_NUM);  // tri-state, pull down

    // Plant instant power
    set_bit(PLANT_POWER_DDR_ADDR, PLANT_POWER_DDR_NUM);  // output
    clear_bit(PLANT_POWER_PORT_ADDR, PLANT_POWER_PORT_NUM);  // low

    // Plant sensor
    clear_bit(PLANT_SENSOR_DDR_ADDR, PLANT_SENSOR_DDR_NUM); // input
    clear_bit(PLANT_SENSOR_PORT_ADDR, PLANT_SENSOR_PORT_NUM);  // tri-state, pull down

    // Relay control -> Motor control
    set_bit(MOTOR_CONTROL_DDR_ADDR, MOTOR_CONTROL_DDR_NUM);  // output
    disable_motor();


    // Configure system clock prescaler
    // (does not affect WD prescaler)
    clock_prescale_set(clock_div_256);

    // Enable interrupts
    sei();

    /*
     * Runtime cycle
     */
    while (true) {

        switch(mode) {

            case MODE_CHECK:

                check_tank();
                if (!enough_water_in_tank) {
                    mode = MODE_ALERT;
                    break;
                }

                check_saucer();
                if (water_in_saucer) {
                    mode = MODE_WAIT;
                    break;
                }

                mode = MODE_IRRIGATE;

                // no break, just execute next mode

            case MODE_IRRIGATE:

                enable_motor();

                irrigation_counter = 0;
                wait_water_in_saucer_counter = 0;
                total_irrigation_cycles_counter = 0;

                while(total_irrigation_cycles_counter < total_irrigation_cycles) {

                    if (motor_enabled) {

                        sleep(WDTO_500MS);
                        ++irrigation_counter;

                        check_tank();
                        if (!enough_water_in_tank) {
                            disable_motor();
                            mode = MODE_ALERT;
                            goto MODE_IRRIGATE_END;
                        }

                        check_saucer();
                        if (water_in_saucer) {
                            disable_motor();
                            // mode = MODE_WAIT;
                            break;
                        }

                        if (irrigation_counter == irrigation_cycles) {
                            irrigation_counter = 0;
                            ++total_irrigation_cycles_counter;
                            disable_motor();
                            continue;
                        }

                    } else {  /* motor is disabled */
#ifdef DEBUG
                        sleep(WDTO_1S);
#else
                        sleep(WDTO_8S);
#endif
                        ++wait_water_in_saucer_counter;

                        if (wait_water_in_saucer_counter == wait_water_in_saucer_cycles) {
                            enable_motor();
                            wait_water_in_saucer_counter = 0;
                            continue;
                        }
                    }
                }

                mode = MODE_WAIT;

MODE_IRRIGATE_END:
                break;

                // no break, just execute next mode

            case MODE_WAIT:
#ifdef DEBUG
                sleep(WDTO_1S);
#else
                for (uint16_t power_down_wait_time_counter = 0;
                     power_down_wait_time_counter < power_down_wait_time;
                     power_down_wait_time_counter++) {

                    sleep(WDTO_8S);
                }
#endif
                mode = MODE_CHECK;
                break;

            case MODE_ALERT:
                // Should alert the user that is not enough water in tank.
                // Currently it is a `blink` function from debug macros.
                // Later on will be a beep using a buzzer. 
                sleep(WDTO_1S);
                blink();
                mode = MODE_CHECK;
                break;

            case MODE_DEBUG:
                // Use it only for debug, 
                //  or to assure that some bits are sent and so on...
                
                if (DDRD & (1 << DDD2)) {
                    blink();
                }

                sleep(WDTO_500MS);
                break;
        }
    }
}


/*
 * Irrigation system based on Atmega328P (Arduino Uno, Arduino Nano).
 *
 * This is a single-file program with detailed comments and explanations.
 * The system does next steps:
 *  - Checks the tank with water if there is enough water to irrigate the plant.
 *  - Check the pot (plant) saucer if it contains water (if there is a water in the saucer, the plant is irrigated). 
 *  - Tries to irrigate the plant using the motor (water pump) control. This is a multiple step procedure. 
 *      The motor is enabled for some amount of time and then goes to disabled state to give a water 
 *      to drain into the saucer; this cycle is repeated for defined number of times or till the water
 *      is present in the saucer. At system first start the system tries to irrigate the plant till saucer
 *      sensor is triggered; then stores count of irrigation tries and deeply sleeps; next awake it will
 *      use stored value of tries and try to irrigate the plant this number of times; if it exceeds saved value,
 *      system enters emergency mode and the only way to exit this mode is to reset the ECU via reset button.
 *  - Sleeps for very long time using a watchdog (WD) timeout and power down sleep mode.
 *  - Alerts the user if there is not enough water in the tank.
 *
 *
 * Pin assignment and description:
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

#define PLANT_SENSOR_PIN_ADDR       PIND
#define PLANT_SENSOR_PIN_NUM        PIND5
#define PLANT_SENSOR_PORT_ADDR      PORTD
#define PLANT_SENSOR_PORT_NUM       PORTD5
#define PLANT_SENSOR_DDR_ADDR       DDRD
#define PLANT_SENSOR_DDR_NUM        DDD5

#define MOTOR_CONTROL_PIN_ADDR      PINB
#define MOTOR_CONTROL_PIN_NUM       PINB0
#define MOTOR_CONTROL_PORT_ADDR     PORTB
#define MOTOR_CONTROL_PORT_NUM      PORTB0
#define MOTOR_CONTROL_DDR_ADDR      DDRB
#define MOTOR_CONTROL_DDR_NUM       DDB0

#define BUILT_IN_LED_PIN_ADDR       PINB
#define BUILT_IN_LED_PIN_NUM        PINB5
#define BUILT_IN_LED_PORT_ADDR      PORTB
#define BUILT_IN_LED_PORT_NUM       PORTB5
#define BUILT_IN_LED_DDR_ADDR       DDRB
#define BUILT_IN_LED_DDR_NUM        DDB5


/*
 * Global variables
 */

#ifdef DEBUG

#define irrigation_delay    WDTO_250MS
uint8_t irrigation_cycles = 4;  // 1[s]
uint8_t irrigation_counter;

#define wait_water_in_saucer_delay    WDTO_1S
uint8_t wait_water_in_saucer_cycles = 1;  // 1[s]
uint8_t wait_water_in_saucer_counter;

#define  power_down_wait_delay    WDTO_1S
uint16_t power_down_wait_cycles = 3;  // 3[s]

#define  wait_before_irrigate_delay    WDTO_8S
uint16_t wait_before_irrigate = 1;  // 8[s]

#else

#define irrigation_delay    WDTO_250MS
uint8_t irrigation_cycles = 16;  // 16 * 0.25 = 4[s] - how long the motor is enabled
uint8_t irrigation_counter;

#define wait_water_in_saucer_delay    WDTO_8S
uint8_t wait_water_in_saucer_cycles = 2;  // 2 * 8 = 16[s] - how long the motor is disabled
uint8_t wait_water_in_saucer_counter;

#define  power_down_wait_delay    WDTO_8S
uint16_t power_down_wait_cycles = 15;  // (2 * 60 / 8) ~ 2[m]

#define  wait_before_irrigate_delay    WDTO_8S
uint16_t wait_before_irrigate_cycles = 15;  // (2 * 60 / 8) ~ 2[m]

#endif

uint8_t total_irrigation_cycles = 0;            // Will be set during first irrigation cycle.
uint8_t total_irrigation_cycles_max_delta = 2;  // If the counter exceeds total_irrigation_cycles + this value,
                                                //  the system will be switched to emergency mode of operation.
uint8_t total_irrigation_cycles_min_delta = 0;  // Currently is not in real use.
uint8_t total_irrigation_counter;
bool is_first_irrigation = true;

#define  wait_before_first_start_delay    WDTO_8S
uint8_t  wait_before_first_start_cycles = 15;  // (2 * 60 / 8) ~ 2[m]

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
    MODE_CHECK_BEFORE_IRRIGATE,
    MODE_IRRIGATE,
    MODE_WAIT,
    MODE_WAIT_BEFORE_IRRIGATE,
    MODE_ALERT__NOT_ENOUGH_WATER_IN_TANK,
    MODE_EMERGENCY,
    MODE_DEBUG,
};

// set initial mode
int mode = MODE_CHECK;


/*
 * Global helpers, shorthands
 */

// set bit in register
#define set_bit(reg, bit_num)     ( (reg) |=  (1 << (bit_num)) )

// clear bit in register
#define clear_bit(reg, bit_num)   ( (reg) &= ~(1 << (bit_num)) )

// check if bit is set in register
#define is_bit_set(reg, bit_num)  ( (reg) &   (1 << (bit_num)) )

// Set WDIE bit in WDTCSR register to enable WD interrupt.
// WD interrupt clears this bit (see reference manual), so need to set it again.
#define enable_wd_interrupt()  (WDTCSR |= (1 << WDIE))

// All sleep operations will use WD interrupt for wake up and deep sleep mode for sleeping.
#define sleep(timeout)          \
do {                            \
    wdt_enable(timeout);        \
    enable_wd_interrupt();      \
    sleep_cpu();                \
    wdt_disable();              \
} while(0);

// Toggle built-in LED on the board. These are debug functions.
#define toggle_built_in_led()  set_bit(BUILT_IN_LED_PIN_ADDR, BUILT_IN_LED_PIN_NUM);
#define blink()                         \
do {                                    \
    toggle_built_in_led();              \
    for(uint16_t i=0; i<32000; ++i);    \
    toggle_built_in_led();              \
} while(0);


/*
 * Functions to check the tank and saucer. 
 *   The main idea here: 
 *    - Set the power (output) pin to let the current to flow to input pin.
 *    - Wait settling time.
 *    - Check (read) the sensor (input) pin and save its state.
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

// WD time-out interrupt
ISR(WDT_vect) {
    // Do nothing.
    // Do not set WDIE bit in WDTCSR register here to enable WD interrupt.
    //   It is prohibited for safety reason. See reference manual.
}


int main(void) {

    /*
     * Startup code
     */

    // Disable all interrupts... Oh, paranoia... All interrupts are not configured at startup.
    cli();

    // configure and enable sleep mode
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

    // built-in LED
    set_bit(BUILT_IN_LED_DDR_ADDR, BUILT_IN_LED_DDR_NUM);  // IDE 13 as output
    clear_bit(BUILT_IN_LED_PORT_ADDR, BUILT_IN_LED_PORT_NUM);  // switch off

    // tank instant power
    set_bit(TANK_POWER_DDR_ADDR, TANK_POWER_DDR_NUM);  // output
    clear_bit(TANK_POWER_PORT_ADDR, TANK_POWER_PORT_NUM);  // low

    // tank sensor
    clear_bit(TANK_SENSOR_DDR_ADDR, TANK_SENSOR_DDR_NUM);  // input
    clear_bit(TANK_SENSOR_PORT_ADDR, TANK_SENSOR_PORT_NUM);  // tri-state, but should be pull down with resistor

    // plant instant power
    set_bit(PLANT_POWER_DDR_ADDR, PLANT_POWER_DDR_NUM);  // output
    clear_bit(PLANT_POWER_PORT_ADDR, PLANT_POWER_PORT_NUM);  // low

    // plant sensor
    clear_bit(PLANT_SENSOR_DDR_ADDR, PLANT_SENSOR_DDR_NUM); // input
    clear_bit(PLANT_SENSOR_PORT_ADDR, PLANT_SENSOR_PORT_NUM);  // tri-state, but should be pull down with resistor

    // relay control -> motor control
    set_bit(MOTOR_CONTROL_DDR_ADDR, MOTOR_CONTROL_DDR_NUM);  // output
    disable_motor();


    // Configure system clock prescaler
    // (does not affect WD prescaler)
//    clock_prescale_set(clock_div_256);

    // enable interrupts
    sei();


    for (uint8_t i=0; i<wait_before_first_start_cycles; ++i) {
        sleep(wait_before_first_start_delay);
    }

    /*
     * Runtime cycle
     */
    while (true) {

        switch(mode) {

            case MODE_CHECK:
            case MODE_CHECK_BEFORE_IRRIGATE:

                check_tank();
                if (!enough_water_in_tank) {
                    if (total_irrigation_cycles == 0) {
                        // This is a first start of the system and the tank is empty...
                        // TODO warn the user about it!
                    }
                    mode = MODE_ALERT__NOT_ENOUGH_WATER_IN_TANK;
                    break;
                }

                check_saucer();
                if (water_in_saucer) {
                    if (total_irrigation_cycles == 0) {
                        // This is a first start of the system and the plant is irrigated...
                        // TODO warn the user about it!
                    }
                    mode = MODE_WAIT;
                    break;
                }

                // Checks done, let's irrigate.
                if (mode == MODE_CHECK_BEFORE_IRRIGATE || total_irrigation_cycles == 0) {
                    mode = MODE_IRRIGATE;
                    break;
                }

                // Or wait some time before irrigation process can start.
                mode = MODE_WAIT_BEFORE_IRRIGATE;

                // No break, just execute next mode.

            case MODE_WAIT_BEFORE_IRRIGATE:
                for (uint16_t i = 0; i < wait_before_irrigate_cycles; ++i) {
                    sleep(wait_before_irrigate_delay);
                }

                mode = MODE_CHECK_BEFORE_IRRIGATE;
                break;

            case MODE_IRRIGATE:

                irrigation_counter = 0;
                wait_water_in_saucer_counter = 0;
                total_irrigation_counter = 0;

                enable_motor();

                while(1) {

                    if (motor_enabled) {  /* motor is enabled */

                        sleep(irrigation_delay);
                        ++irrigation_counter;

                        check_tank();
                        if (!enough_water_in_tank) {
                            disable_motor();
                            mode = MODE_ALERT__NOT_ENOUGH_WATER_IN_TANK;
                            break;
                        }

                        check_saucer();
                        if (water_in_saucer) {
                            disable_motor();
                            // If it is triggered at this point (when the motor is enabled),
                            //   the counter should also be incremented. Think it as a `ceil` function
                            //   from `math` library. Example: ceil(1.2 times) -> 2 times.
                            ++total_irrigation_counter;
                            mode = MODE_WAIT;
                            break;
                        }

                        // Check counter when the motor is enabled
                        if (irrigation_counter == irrigation_cycles) {
                            irrigation_counter = 0;
                            ++total_irrigation_counter;
                            disable_motor();

                            if (total_irrigation_counter >= total_irrigation_cycles &&
                                    total_irrigation_cycles != 0) {

                                // At this point the counter reaches the maximum value.
                                // Let the system add +1 to the final counter. Do nothing...

                            }

                            if (total_irrigation_counter == total_irrigation_cycles +
                                    total_irrigation_cycles_max_delta && total_irrigation_cycles != 0) {

                                // Emergency! Something goes wrong!
                                // Maybe the sensor is outside the saucer.
                                // Or the motor has low power (power bank is discharged)
                                //   and can't pour water into the plant.
                                mode = MODE_EMERGENCY;
                                break;

                            }
                        }

                    } else {  /* motor is disabled */

                        sleep(wait_water_in_saucer_delay);
                        ++wait_water_in_saucer_counter;

                        // Check counter when the motor is disabled
                        if (wait_water_in_saucer_counter == wait_water_in_saucer_cycles) {
                            wait_water_in_saucer_counter = 0;

                            check_saucer();
                            if (water_in_saucer) {
                                disable_motor();
                                mode = MODE_WAIT;
                                break;
                            }

                            enable_motor();

                        }
                    }
                }

                // Now the mode is set. Let's check it.

                if (mode == MODE_WAIT) {
                    // This is expected state, need to check counters once again.

                    if (total_irrigation_cycles == 0) {

                        if (is_first_irrigation == false) {

                            // This is a first start of the system.
                            // Save the total cycles counter. It will be used as a expected counter all the time.
                            total_irrigation_cycles = total_irrigation_counter;

                        } else {  // on first irrigation do not change the global counter
                            is_first_irrigation = false;
                        }

                    } else {

                        if (total_irrigation_counter <=
                                total_irrigation_cycles - total_irrigation_cycles_min_delta) {

                            // This happens when the water reaches the saucer very fast.
                            // Do nothing...

                        }
                    }
                }

                break;

            /*
             * This mode is triggered to wait between sensor checks.
             */
            case MODE_WAIT:

                for (uint16_t power_down_wait_time_counter = 0;
                              power_down_wait_time_counter < power_down_wait_cycles;
                              power_down_wait_time_counter++) {

                     sleep(power_down_wait_delay);
                }

                mode = MODE_CHECK;
                break;

            case MODE_ALERT__NOT_ENOUGH_WATER_IN_TANK:
                // Should alert the user that is not enough water in tank.
                // Currently it is a `blink` function from debug macros.
                // Later on will be a beep using a buzzer.
                sleep(WDTO_8S);
                blink();
                mode = MODE_CHECK;
                break;

            case MODE_EMERGENCY:
                // Do not change the `mode`. This causes an infinite loop to execute.
                // Need to push the reset button on the board to let the system exit this loop.
                sleep(WDTO_8S);
                blink();
                break;

            case MODE_DEBUG:
                // Use it only for debug, 
                //  or to assure that some bits are set or maybe not.. and so on...
                
                // Your pretty code goes here...
                //  ...
                blink();

                sleep(WDTO_500MS);
                break;
        }
    }
}

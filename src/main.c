
/*
 * Irrigation system based on Atmega328P (Arduino Uno, Arduino Nano).
 *
 * This is a single-file program with detailed comments and explanations.
 *
 * +-----+----------+--------------+--------------------------------------------------------------------+
 * | IDE | Port pin | Physical pin | Assignment description                                             |
 * +-----+----------+--------------+--------------------------------------------------------------------+
 * | 2   | PD2      | 4            | Instant power for tank                                             |
 * | 3   | PD3      | 5            | Sensor for tank, should be pulled down with 10k resistor           |
 * | 4   | PD4      | 6            | Instant power for plant                                            |
 * | 5   | PD5      | 7            | Sensor for plant, should be pulled down with 10k resistor          |
 * | 8   | PB0      | 14           | Relay -> motor                                                     |
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
#define DEBUG

/* IDE assignment */
#define IDE_TANK_POWER      2
#define IDE_TANK_SENSOR     3
#define IDE_PLANT_POWER     4
#define IDE_PLANT_SENSOR    5
#define IDE_MOTOR_CONTROL   8


/*
 * Global helpers, shorthands
 */

// Set bit in register
#define set_bit(reg, bit_num)     ( (reg) |=  (1 << (bit_num)) )

// Clear bit in register
#define clear_bit(reg, bit_num)   ( (reg) &= ~(1 << (bit_num)) )

// Check if bit is set
#define is_bit_set(reg, bit_num)  ( (reg) &   (1 << (bit_num)) )

// Set WDIE bit in WDTCSR register to enable WD interrupt.
// WD interrupt clears this bit (see ref. man.), so need to set it again.
#define enable_wd_interrupt()  (WDTCSR |= (1 << WDIE))

// Toggle built-in led on the board
#define toggle_led()  (PINB |= (1 << PINB5))


/*
 * IDE - port pin
 *   mapping table
 */

enum ide_port_mapping_t {
    PIN_ADDR = 0,
    PIN_NUM,
    PORT_ADDR,
    PORT_NUM,
    DDR_ADDR,
    DDR_NUM,
    ide_port_mapping_length,
};

uint8_t port[][ide_port_mapping_length] = {
  /* IDE        PIN_ADDR,   PIN_NUM,    PORT_ADDR,  PORT_NUM,   DDR_ADDR,   DDR_NUM
     ----       --------    -------     ---------   --------    --------    -------    */
  /*  0 */  {   &PIND,      PIND0,      &PORTD,     PORTD0,     &DDRD,      DDD0    },
  /*  1 */  {   &PIND,      PIND1,      &PORTD,     PORTD1,     &DDRD,      DDD1    },
  /*  2 */  {   &PIND,      PIND2,      &PORTD,     PORTD2,     &DDRD,      DDD2    },
  /*  3 */  {   &PIND,      PIND3,      &PORTD,     PORTD3,     &DDRD,      DDD3    },
  /*  4 */  {   &PIND,      PIND4,      &PORTD,     PORTD4,     &DDRD,      DDD4    },
  /*  5 */  {   &PIND,      PIND5,      &PORTD,     PORTD5,     &DDRD,      DDD5    },
  /*  6 */  {   &PIND,      PIND6,      &PORTD,     PORTD6,     &DDRD,      DDD6    },
  /*  7 */  {   &PIND,      PIND7,      &PORTD,     PORTD7,     &DDRD,      DDD7    },
  /*  8 */  {   &PINB,      PINB0,      &PORTB,     PORTB0,     &DDRB,      DDB0    },
  /*  9 */  {   &PINB,      PINB1,      &PORTB,     PORTB1,     &DDRB,      DDB1    },
  /* 10 */  {   &PINB,      PINB2,      &PORTB,     PORTB2,     &DDRB,      DDB2    },
  /* 11 */  {   &PINB,      PINB3,      &PORTB,     PORTB3,     &DDRB,      DDB3    },
  /* 12 */  {   &PINB,      PINB4,      &PORTB,     PORTB4,     &DDRB,      DDB4    },
  /* 13 */  {   &PINB,      PINB5,      &PORTB,     PORTB5,     &DDRB,      DDB5    },
  // TODO: map remain IDE - Port pins
};

#define set_ide(IDE)       set_bit( port[IDE][PORT_ADDR], port[IDE][PORT_NUM] )
#define clear_ide(IDE)   clear_bit( port[IDE][PORT_ADDR], port[IDE][PORT_NUM] )
#define check_ide(IDE)  is_bit_set( port[IDE][PIN_ADDR ], port[IDE][PIN_NUM ] )


/*
 * Global variables
 *
 */

/*
 * Cycle counter.
 *   To count how many times WD interrupt fires.
 *   Another explanation: Count how many times ECU goes to sleep.
 *   Another explanation: How many times ECU wakes up after sleep.
 */
uint16_t power_down_counter = 0;

//#ifdef DEBUG
//    const uint16_t power_down_idle_length = 1;
//#else
//    // 60[seconds in minute] * 60[minutes in hour]
//    // 8s is the WDT timeout (from prescaler)
//    const uint16_t power_down_idle_length = (60 * 60) / 8;
//#endif

/*
 * Bottle with a water (tank).
 * Set to worst case (`false`) to let the system change it.
 */
bool enough_water_in_tank = false;

/*
 * If any water in the saucer.
 * Set to worst case (`false`) to let the system change it.
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

#define MODE_WAIT       0
#define MODE_RUN        1
#define MODE_ALERT      2
uint8_t mode = MODE_RUN;

static inline void check_tank(void) {
    set_ide(IDE_TANK_POWER);
    _NOP();
    enough_water_in_tank = check_ide(IDE_TANK_SENSOR);
    clear_ide(IDE_TANK_POWER);
}

static inline void check_saucer(void) {
    set_ide(IDE_PLANT_POWER);
    _NOP();
    water_in_saucer = check_ide(IDE_PLANT_SENSOR);
    clear_ide(IDE_PLANT_POWER);
}

static inline void enable_motor(void) {
    if (!motor_enabled) {
        set_ide(IDE_MOTOR_CONTROL);
        motor_enabled = true;
    }
}

static inline void disable_motor(void) {
    if (motor_enabled) {
        clear_ide(IDE_MOTOR_CONTROL);
        motor_enabled = false;
    }
}


/*
 * Interrupt handlers
 */

// Watchdog time-out interrupt
ISR(WDT_vect) {
  // do nothing
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

    // Configure WD and its timer initial value
#ifdef DEBUG
    wdt_enable(WDTO_1S);
#else
    wdt_enable(WDTO_8S);
#endif


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
    set_bit(port[IDE_TANK_POWER][DDR_ADDR], port[IDE_TANK_POWER][DDR_NUM]);  // output
    clear_ide(IDE_TANK_POWER);  // low

    // Tank sensor
    clear_bit(port[IDE_TANK_SENSOR][DDR_ADDR],
              port[IDE_TANK_SENSOR][DDR_NUM]);  // input

    clear_ide(IDE_TANK_SENSOR);  // tri-state, pull down

    // Plant instant power
    set_bit(port[IDE_PLANT_POWER][DDR_ADDR], port[IDE_PLANT_POWER][DDR_NUM]);  // output
    clear_ide(IDE_PLANT_POWER);  // low

    // Plant sensor
    clear_bit(port[IDE_PLANT_SENSOR][DDR_ADDR],
              port[IDE_PLANT_SENSOR][DDR_NUM]);  // input

    clear_ide(IDE_PLANT_SENSOR);  // tri-state, pull down

    // Relay control -> Motor control
    set_bit(port[IDE_MOTOR_CONTROL][DDR_ADDR], port[IDE_MOTOR_CONTROL][DDR_NUM]);  // output
    disable_motor();


    // Configure system clock prescaler
    // (does not affect WD prescaler)
    clock_prescale_set(clock_div_256);

    // Reset watchdog timer before main loop
    wdt_reset();

    // Enable interrupts
    sei();

    /*
     * Runtime cycle
     */
    while (true) {

        switch(mode) {

            case MODE_RUN:

                // TODO introduce the timeout for motor in enabled state
                // (disable it once the timeout is reached)

                check_tank();
                if (!enough_water_in_tank) {
                    mode = MODE_ALERT;
                    enable_wd_interrupt();
                    break;
                }

                check_saucer();
                if (water_in_saucer) {
                    mode = MODE_WAIT;
                    enable_wd_interrupt();
                    break;
                }


                break;

            case MODE_WAIT:
                ++power_down_counter;
                enable_wd_interrupt();
                break;

            case MODE_ALERT:

                break;
        }

        // Enter sleep mode. No command will be executed till reset or WD interrupt.
        sleep_cpu();

    }
}













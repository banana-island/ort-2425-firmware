#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/i2c_slave.h"

// gpio 2,6,10,21 for motors
// matching pins for motors are 3,7,11,20
// i2c pins 0,1
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// I2C properties
#define I2C_SLAVE_ADDRESS 0x42
#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1

const uint MOTOR_PINS[4] = {3, 6, 10, 20}; // setting 3 and 20 to control the rover to go forward
const uint SETS_OF_MOTOR_PINS[8] = {2, 3, 6, 7, 10, 11, 20, 21};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// this function is the set up for the loop which will go through each pin in the array in the main
void setup_pwm(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice_num, 255); // 8-bit res - change to 16 bit for higher pwm frequency? //scale it accordingly to -127 and 127
    // setting channel now for duty cycles
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pin), 1); // starting off at 100% duty, then finding channel for pin - sets all pins to high
    pwm_set_enabled(slice_num, true);
}

// function set the speed -no need to re enable; it happens once the duty is changed simultaneously
void set_pwm_duty(uint pin, uint8_t duty)
{
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pin), duty); // starting off at 0% duty, then finding channel for pin
}

uint8_t receive_data[32]; // amount of data we can receive from the controller
uint8_t send_data[32];    // amount we send

int main()
{
    stdio_init_all();

    // initializing the i2c port at 100khz(the standard apparently) as peripheral
    i2c_init(I2C_PORT, 100000);
    i2c_set_slave_mode(I2C_PORT, true, I2C_SLAVE_ADDRESS);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Setup PWM for motors
    for (int i = 0; i < 8; i++)
    {
        setup_pwm(SETS_OF_MOTOR_PINS[i]);
    }

    uint8_t pi_reception[4]; // reception area for pi instructions

    while (true)
    { // receive data from controller
        if (i2c_get_read_available(i2c0) < 4)
            continue;
        i2c_read_raw_blocking(i2c0, receive_data, 4);
        {
            // scaling 0-255 to handle negative (backward motion)
            // making 128 the neutral (stop), 129-255 the forward, 0-127 the backward

            for (int i = 0; i < 4; i++)
            {
                set_pwm_duty(MOTOR_PINS[i], receive_data[i]);
            }
        }

        sleep_ms(10); // avoids overcrowding the I2C
    }
}
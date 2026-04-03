#include "l298n.h"
#include "config.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// ----------------------------------------------------------------
//  Internal helper: configure one motor channel
// ----------------------------------------------------------------
static void init_channel(uint in1, uint in2, uint en)
{
    // Direction pins — plain GPIO output, start LOW (braked / off)
    gpio_init(in1);
    gpio_set_dir(in1, GPIO_OUT);
    gpio_put(in1, 0);

    gpio_init(in2);
    gpio_set_dir(in2, GPIO_OUT);
    gpio_put(in2, 0);

    // Enable pin — configured for hardware PWM
    gpio_set_function(en, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(en);

    // Wrap at 1000 so speed values map 1-to-1 to PWM counts.
    // With clkdiv = 125 and sys_clock = 125 MHz:
    //   PWM freq = 125 000 000 / 125 / 1001 ≈ 1 kHz  (safe for L298N)
    pwm_set_clkdiv(slice, 125.0f);
    pwm_set_wrap(slice, 1000);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(en), 0);
    pwm_set_enabled(slice, true);
}

// ----------------------------------------------------------------
//  Internal helper: drive one motor
// ----------------------------------------------------------------
static void drive_channel(uint in1, uint in2, uint en, int speed)
{
    // Clamp
    if (speed >  MAX_SPEED) speed =  MAX_SPEED;
    if (speed < -MAX_SPEED) speed = -MAX_SPEED;

    if (speed > 0) {
        gpio_put(in1, 1);
        gpio_put(in2, 0);
    } else if (speed < 0) {
        gpio_put(in1, 0);
        gpio_put(in2, 1);
        speed = -speed;         // duty cycle is always positive
    } else {
        // Brake: both LOW → coast (change to HIGH/HIGH for hard brake)
        gpio_put(in1, 0);
        gpio_put(in2, 0);
    }

    uint slice   = pwm_gpio_to_slice_num(en);
    uint channel = pwm_gpio_to_channel(en);
    pwm_set_chan_level(slice, channel, (uint16_t)speed);
}

// ----------------------------------------------------------------
//  Public API
// ----------------------------------------------------------------
void motor_init(void)
{
    init_channel(FL_IN1, FL_IN2, FL_EN);
    init_channel(RL_IN1, RL_IN2, RL_EN);
    init_channel(FR_IN1, FR_IN2, FR_EN);
    init_channel(RR_IN1, RR_IN2, RR_EN);
    motor_stop();
}

void motor_set(int fl, int rl, int fr, int rr)
{
    drive_channel(FL_IN1, FL_IN2, FL_EN, fl);
    drive_channel(RL_IN1, RL_IN2, RL_EN, rl);
    drive_channel(FR_IN1, FR_IN2, FR_EN, fr);
    drive_channel(RR_IN1, RR_IN2, RR_EN, rr);
}

void motor_stop(void)
{
    motor_set(0, 0, 0, 0);
}

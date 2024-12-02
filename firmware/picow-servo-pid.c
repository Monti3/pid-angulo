#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

#define IN1 14
#define IN2 15
#define ADC_PIN 27

void start_pwm(uint8_t pin);
void set_pwm(uint8_t pin, float percentage);

int main()
{
    stdio_init_all();
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(1);

    start_pwm(IN1);
    start_pwm(IN2);

    float K_p = 3;

    uint16_t full_duty = 50000; // Wrap del PWM
    float desired_angle = 1000;
    float previous_error = 0; // Error del ciclo anterior
    float dt = 0.1f;
    float integral_error;
    float K_i = 0.01;
    float K_d = 1;
    uint16_t adc;

    while (true)
    {
        adc = adc_read();
        float error = (float)(desired_angle - adc);
        float derivative = (error - previous_error) / dt;
        float control_signal = (K_p * error) + (K_i * integral_error) + (K_d * derivative);
        integral_error += error * dt;
        if (integral_error > full_duty)
            integral_error = full_duty;
        if (integral_error < -full_duty)
            integral_error = -full_duty;

        if (integral_error > full_duty)
            integral_error = full_duty;
        if (integral_error < -full_duty)
            integral_error = -full_duty;

        if (error < 500 && error > -500)
        {
            control_signal = 0;
        }
        else
        {
            // Saturación del control_signal
            if (control_signal > full_duty)
                control_signal = full_duty;
            else if (control_signal < -full_duty)
                control_signal = -full_duty;
        }

        pwm_set_gpio_level(IN1, (control_signal > 0) ? (uint16_t)control_signal : 0);
        pwm_set_gpio_level(IN2, (control_signal < 0) ? (uint16_t)(-control_signal) : 0);

        printf("Error:%.2f\t Setpoint:%.2f\t ADC:%u\t ContolS:%.2f\n", error, desired_angle, adc, control_signal);

        sleep_ms(10);
    }
}

void start_pwm(uint8_t pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint8_t slice = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 50.0f); // Divisor = 50
    pwm_config_set_wrap(&config, 50000);   // Wrap = 50000
    pwm_init(slice, &config, true);
    pwm_set_gpio_level(pin, 0);
}

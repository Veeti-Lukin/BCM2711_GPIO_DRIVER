
#ifndef BCM2711_GPIO_DRIVER_GPIO_TYPES_H
#define BCM2711_GPIO_DRIVER_GPIO_TYPES_H

#include <cstdint>

// This means pin HIGH, true, 3.3 volts on a pin.
// Can be given as argument to <set_output_pin()>
const uint8_t HIGH = 1;
// This means pin LOW, false, 0 volts on a pin.
// Can be given as argument to <set_output_pin()>
const uint8_t LOW = 0;

// ALTERNATIVE PIN FUNCTIONS PER PIN CAN BE FOUND IN BCM2711_peripherals.pdf SECTION 5.3
// These enum constant can be used to define the operation of the GPIO pin.
// Each of the 58 GPIO pins has at least two alternative functions.
// Encoding each function requires 3 bits, 000 - 111.
// One of these constants can be given as arg to <setup_gpio_function()>
enum class PinFunction
{
    INPUT = 0b000,
    OUTPUT = 0b001,
    ALT_FUNC0 = 0b100,
    ALT_FUNC1 = 0b101,
    ALT_FUNC2 = 0b110,
    ALT_FUNC3 = 0b111,
    ALT_FUNC4 = 0b011,
    ALT_FUNC5 = 0b010
};

struct PwmPin {
    uint8_t pin_num;
    PinFunction pwm_func;
};


const PwmPin K_PWM_PIN12{12, PinFunction::ALT_FUNC0};
const PwmPin K_PWM_PIN13{13, PinFunction::ALT_FUNC0};
const PwmPin K_PWM_PIN18{18, PinFunction::ALT_FUNC5};
const PwmPin K_PWM_PIN19{12, PinFunction::ALT_FUNC5};

#endif //BCM2711_GPIO_DRIVER_GPIO_TYPES_H

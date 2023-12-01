#include <iostream>
#include "BCM2711_GPIO_Driver/bcm2711_gpio_driver.h"
#include "BCM2711_GPIO_Driver/gpio_types.h"


int main()
{
    BCM2711_GPIO_DRIVER::initialize();

    BCM2711_GPIO_DRIVER::setPinGpioFunction(K_PWM_PIN13.pin_num, K_PWM_PIN13.pwm_func);

    std::cout << (unsigned int)BCM2711_GPIO_DRIVER::getGpioFunction(K_PWM_PIN13.pin_num) <<
    std::endl;

    BCM2711_GPIO_DRIVER::terminate();

    return EXIT_SUCCESS;
}

#include <iostream>
#include "BCM2711_GPIO_Driver/bcm2711_gpio_driver.h"
#include "BCM2711_GPIO_Driver/gpio_types.h"


int main()
{
    BCM2711_GPIO_DRIVER::initialize();

    BCM2711_GPIO_DRIVER::setPinGpioFunction(K_PWM_PIN13.pin_num, K_PWM_PIN13.pwm_func);
    BCM2711_GPIO_DRIVER::setPinGpioFunction(K_PWM_PIN12.pin_num, K_PWM_PIN12.pwm_func);
    BCM2711_GPIO_DRIVER::setPinGpioFunction(6,PinFunction::OUTPUT);
    BCM2711_GPIO_DRIVER::setPinGpioFunction(5,PinFunction::OUTPUT);
    BCM2711_GPIO_DRIVER::setPinGpioFunction(7,PinFunction::OUTPUT);
    BCM2711_GPIO_DRIVER::setPinGpioFunction(1,PinFunction::OUTPUT);

    BCM2711_GPIO_DRIVER::setOuputPin(6,1);
    BCM2711_GPIO_DRIVER::setOuputPin(5,0);
    BCM2711_GPIO_DRIVER::setOuputPin(7, 1);
    BCM2711_GPIO_DRIVER::setOuputPin(1, 0);

    std::cout << (unsigned int)BCM2711_GPIO_DRIVER::getGpioFunction(K_PWM_PIN13.pin_num) <<
    std::endl;

    BCM2711_GPIO_DRIVER::enablePwmPin(K_PWM_PIN12.pin_num, true);
    BCM2711_GPIO_DRIVER::enablePwmPin(K_PWM_PIN13.pin_num, true);

    while (true){
        int16_t duty_cycle_percentage;
        std::cin >> duty_cycle_percentage;
        if(duty_cycle_percentage < 0) break;

        BCM2711_GPIO_DRIVER::configPwmPin(10000, duty_cycle_percentage,
                                          K_PWM_PIN12.pin_num);
        std::cin >> duty_cycle_percentage;
        BCM2711_GPIO_DRIVER::configPwmPin(10000, duty_cycle_percentage,
                                          K_PWM_PIN13.pin_num);

        std::cout << "----" << std::endl;
    }

    BCM2711_GPIO_DRIVER::terminate();

    return EXIT_SUCCESS;
}

#include <iostream>
#include "bcm2711_gpio_driver.hh"
#include "CarController.hh"


int main()
{
    CarController controller(K_PWM_PIN18, 15, 14, K_PWM_PIN12, 1, 7);
    int angle;
    int speed;
    while (true) {
        std::cin >> angle;
        std::cin >> speed;
        controller.driveRelativeDirection(angle, speed);
    }
    return 0;
}

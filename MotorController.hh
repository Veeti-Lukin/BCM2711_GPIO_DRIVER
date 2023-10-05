#ifndef MOTORCONTROLLER_HH
#define MOTORCONTROLLER_HH

#include <cstdint>
#include "bcm2711_gpio_driver.hh"

enum MotorSpinDirection{
    forward,
    backwards
};

class L289N_MotorController
{
public:
    L289N_MotorController( BCM2711_GPIO_DRIVER& gpio_driver, uint8_t dir1_pin, uint8_t dir2_pin, PwmPin speed_pin);
    void setSpeed(uint8_t speed_percentage);
    void setDirection(MotorSpinDirection direction);
    void startMotor();
    void stopMotor();

    /**
     * @brief forceStop to leave this force stop state use setDirection
     */
    void forceStop();
private:
    BCM2711_GPIO_DRIVER& gpio_driver_;
     uint8_t dir1_pin_;
     uint8_t dir2_pin;
     uint8_t speed_pin_;
     static const uint32_t K_PWM_FREQUENCY = 20000;
     // for tt motor = 86
     uint8_t effective_ducty_cycle_start = 75;
     bool is_stopped_ = true;
};

#endif // MOTORCONTROLLER_HH

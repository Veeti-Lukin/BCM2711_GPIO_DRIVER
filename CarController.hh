#ifndef CARCONTROLLER_HH
#define CARCONTROLLER_HH

#include <cstdint>
#include "MotorController.hh"
#include "bcm2711_gpio_driver.hh"

class CarController
{
public:
    CarController(PwmPin left_motor_speed_pin, uint8_t left_motor_dir1_pin,
                  uint8_t left_motor_dir2_pin, PwmPin right_motor_speed_pin,
                  uint8_t right_motor_dir1_pin, uint8_t right_motor_dir2_pin);

    void driveForward();
    void driveBackwards();
    void turnLeft();
    void turnRight();

    void setSpeed(uint8_t speed_percentage);

    void driveRelativeDirection(int16_t angle, uint8_t speed);

    void stop();
private:
    BCM2711_GPIO_DRIVER gpio_driver;

    L289N_MotorController right_motor;
    L289N_MotorController left_motor;

    uint8_t speed_;
};

#endif // CARCONTROLLER_HH

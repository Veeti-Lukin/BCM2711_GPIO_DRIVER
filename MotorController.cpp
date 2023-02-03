#include "MotorController.hh"
#include <chrono>
#include <thread>


L289N_MotorController::L289N_MotorController(BCM2711_GPIO_DRIVER &gpio_driver, uint8_t dir1_pin, uint8_t dir2_pin, PwmPin speed_pin):
gpio_driver_(gpio_driver), dir1_pin_(dir1_pin), dir2_pin(dir2_pin), speed_pin_(speed_pin.pin_num){
    gpio_driver_.setPinGpioFunction(speed_pin.pin_num, speed_pin.pwm_func);
    gpio_driver_.setPinGpioFunction(dir1_pin, PinFunction::OUTPUT);
    gpio_driver_.setPinGpioFunction(dir2_pin, PinFunction::OUTPUT);
}


void L289N_MotorController::setSpeed(uint8_t speed_percentage)
{
    if (speed_percentage > 100) speed_percentage = 100;
    uint8_t duty_cycle_percentage = (speed_percentage != 0) ?
            effective_ducty_cycle_start + (100 - effective_ducty_cycle_start) *
            (speed_percentage / 100.0) : 0;

    if (is_stopped_) {
        // get rid of static friction
        gpio_driver_.configPwmPin(K_PWM_FREQUENCY, 100, speed_pin_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        is_stopped_ = false;
    }

    gpio_driver_.configPwmPin(K_PWM_FREQUENCY, duty_cycle_percentage, speed_pin_);

    if(speed_percentage == 0) is_stopped_ = true;
}

void L289N_MotorController::setDirection(MotorSpinDirection direction)
{
    if(direction == forward) {
       gpio_driver_.setOuputPin(dir1_pin_, HIGH);
       gpio_driver_.setOuputPin(dir2_pin, LOW);
    }
    else{
        gpio_driver_.setOuputPin(dir1_pin_, LOW);
        gpio_driver_.setOuputPin(dir2_pin, HIGH);
    }
}

void L289N_MotorController::startMotor()
{
    gpio_driver_.enablePwmPin(speed_pin_, true);
}

void L289N_MotorController::stopMotor()
{
    is_stopped_ = true;
    gpio_driver_.enablePwmPin(speed_pin_, false);
}

void L289N_MotorController::forceStop()
{
    is_stopped_ = true;
    gpio_driver_.setOuputPin(dir1_pin_, HIGH);
    gpio_driver_.setOuputPin(dir2_pin, HIGH);
}


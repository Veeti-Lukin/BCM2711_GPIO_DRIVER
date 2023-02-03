#include "CarController.hh"
#include "bcm2711_gpio_driver.hh"

CarController::CarController(PwmPin left_motor_speed_pin, uint8_t left_motor_dir1_pin,
                             uint8_t left_motor_dir2_pin, PwmPin right_motor_speed_pin,
                             uint8_t right_motor_dir1_pin, uint8_t right_motor_dir2_pin) :
    right_motor(gpio_driver, right_motor_dir1_pin, right_motor_dir2_pin, right_motor_speed_pin),
    left_motor(gpio_driver, left_motor_dir1_pin, left_motor_dir2_pin, left_motor_speed_pin)
{
    right_motor.startMotor();
    left_motor.startMotor();
}

void CarController::driveForward()
{
    right_motor.setDirection(forward);
    left_motor.setDirection(forward);
    right_motor.setSpeed(speed_);
    left_motor.setSpeed(speed_);
}

void CarController::driveBackwards()
{
    right_motor.setDirection(backwards);
    left_motor.setDirection(backwards);
    right_motor.setSpeed(speed_);
    left_motor.setSpeed(speed_);
}

void CarController::turnLeft()
{

    right_motor.setDirection(forward);
    left_motor.setDirection(backwards);
    right_motor.setSpeed(speed_);
    left_motor.setSpeed(speed_);
}

void CarController::turnRight()
{
    {

        right_motor.setDirection(backwards);
        left_motor.setDirection(forward);
        right_motor.setSpeed(speed_);
        left_motor.setSpeed(speed_);
    }

}

void CarController::driveRelativeDirection(int16_t angle, uint8_t speed)
{
    float angle_factor = static_cast<float>(90 - abs(angle)) / 90.0;
    uint8_t inner_motor_speed = static_cast<int>(angle_factor * speed);

    //todo: refactor
    if (angle < 90 && angle > -90) {
        right_motor.setDirection(forward);
        left_motor.setDirection(forward);
        if (angle > 0) { // turning right
            right_motor.setDirection(forward);
            left_motor.setDirection(forward);
            left_motor.setSpeed(speed);
            right_motor.setSpeed(inner_motor_speed);
        }

        else if (angle < 0) { // turning left
            right_motor.setDirection(forward);
            left_motor.setDirection(forward);
            right_motor.setSpeed(speed);
            left_motor.setSpeed(inner_motor_speed);
        }

        else { // angle = 0 hence driving straight forward
            right_motor.setDirection(forward);
            left_motor.setDirection(forward);
            right_motor.setSpeed(speed);
            left_motor.setSpeed(speed);
        }
    }
    else { // driving backwards
        if (angle > 0) { // turning right
            right_motor.setDirection(backwards);
            left_motor.setDirection(backwards);
            left_motor.setSpeed(speed);
            right_motor.setSpeed(inner_motor_speed);
        }

        else if (angle < 0) { // turning left
            right_motor.setDirection(backwards);
            left_motor.setDirection(backwards);
            right_motor.setSpeed(speed);
            left_motor.setSpeed(inner_motor_speed);
        }

        else { // angle = 180 hence driving straight backwards
            right_motor.setDirection(backwards);
            left_motor.setDirection(backwards);
            right_motor.setSpeed(speed);
            left_motor.setSpeed(speed);
        }

    }
}

void CarController::setSpeed(uint8_t speed_percentage)
{
    speed_ = speed_percentage;
}

void CarController::stop()
{
    left_motor.forceStop();
    right_motor.forceStop();
}

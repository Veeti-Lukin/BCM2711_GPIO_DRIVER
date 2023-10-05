#include "PiezoControllorer.hh"
#include <future>
#include <chrono>
#include <thread>

PiezoControllorer::PiezoControllorer(BCM2711_GPIO_DRIVER &gpio_driver, PwmPin pin) :
    pin_num_(pin.pin_num), gpio_driver_(gpio_driver)
{
    gpio_driver_.setPinGpioFunction(pin.pin_num, pin.pwm_func);
}

void PiezoControllorer::setVolume(uint8_t percentage)
{
    if (percentage > 100) percentage = 100;
    volume_percentage_ = percentage;
}

void PiezoControllorer::playSound(uint16_t freq, uint16_t duration_ms, SoundQueueType prot)
{
    using namespace std::chrono;

    auto func = [&](){
        gpio_driver_.configPwmPin(freq, volume_percentage_/2, pin_num_);

        time_point timeout_time_point = system_clock::now() + milliseconds(duration_ms);
        std::this_thread::sleep_until(timeout_time_point);

        gpio_driver_.configPwmPin(freq, 0, pin_num_);
    };

    auto ret = std::async(std::launch::async, func);
}

void PiezoControllorer::playSoundPattern()
{

}

/// C++ class library that allows use of GPIO pins in raspberry 4b
/// all operations on BCM2711 peripherals are accomplished by
/// manipulating the registers associated for that type of peripheral.

#ifndef BCM_2711_DRIVER_HH
#define BCM_2711_DRIVER_HH

#include <cstdint>
#include "GpioDriverException.h"
#include "gpio_types.h"

// Uncommenting this line disables all the platform access of the driver.
// The driver can still be used but it just (roughly) emulates the behaviour of how the
// driver would work in the real case. However, i.e. pin functions will always be set
// to 0 when the driver is initialized unlike on the real platform case.
// Can be used to develop code depended on this driver in dev machine instead than on
// the target
//#define EMULATE_GPIO_DRIVER

// bcm2711-peripherals.pdf section 5.2 states:
// "The GPIO register base address is 0x7e20_0000".
// However, addresses starting at 0x7e00_0000 are in VPU address space.
// If bcm2711 is running in "low peripheral mode" (default on rpi4)
// this address range is re-mapped to start at 0xFE00_0000
// in arm physical address space.
static const uint32_t K_GPIO_BASE_ADDRESS = 0xFE200000;

// There are 58 General-Purpose Input/Output (GPIO) pin lines in BCM2711
// However rpi4 has only 40 pins, of which 28 are BCM2711 GPIO pins
static const uint8_t K_GPIO_PIN_AMOUNT = 58;


// BCM2711 has two hardware pwm channel on board.
// Each of these channels have its own controlling registers.
//0x7e20c000
static const uint32_t K_PWM0_BASE_ADDRESS = 0xFE20c000;
//0x7e20c800.
static const uint32_t K_PWM1_BASE_ADDRESS =  0xFE20c800;
// 0x7e101000
static const uint32_t  K_CLOCK_MANAGER_BASE_ADDRESS = 0xFE101000;

static const uint32_t K_DEFAULT_CLOCK_FREQ = 19200000;
static const uint32_t K_CLOCK_MANAGER_PASSWORD = 0x5A << 24;


class BCM2711_GPIO_DRIVER {
public:
    // disable constructor
    BCM2711_GPIO_DRIVER() = delete;
    ~BCM2711_GPIO_DRIVER() = delete;
    BCM2711_GPIO_DRIVER(const BCM2711_GPIO_DRIVER&) = delete;
    BCM2711_GPIO_DRIVER(BCM2711_GPIO_DRIVER&&) = delete;
    BCM2711_GPIO_DRIVER& operator=(const BCM2711_GPIO_DRIVER&) = delete;
    BCM2711_GPIO_DRIVER& operator=(BCM2711_GPIO_DRIVER&&) = delete;

    static void initialize();
    static void terminate();

    static void setPinGpioFunction(uint8_t pin_num, PinFunction func);
    static PinFunction getGpioFunction(uint8_t pin_num);

    // clear if value 0
    // set to high if value 1
    // checkkaa onko oikeesti output pinni ja palauttaa false jo ei
    static bool setOuputPin(uint8_t pin_num, bool value);

    static void configPwmPin(uint32_t freq, uint8_t duty_cycle_precentage, uint8_t pin_num);
    static void enablePwmPin(uint8_t pin_num, bool enable);

private:
    // This pointer points to start (more spesifically to 32 first bits)
    // of address range where GPIO controlling registers are memory mapped
    // in the virtual address space.
    static volatile uint32_t* virtual_memory_gpio_base_ptr;


    // CONTROL REGISTER POINTERS
    // Each register is 32 bits long
    static volatile uint32_t* GPFSEL0;
    static volatile uint32_t* GPFSEL1;
    static volatile uint32_t* GPFSEL2;
    static volatile uint32_t* GPFSEL3;
    static volatile uint32_t* GPFSEL4;
    static volatile uint32_t* GPFSEL5;
    static volatile uint32_t* GPSET0;
    static volatile uint32_t* GPSET1;
    static volatile uint32_t* GPCLR0;
    static volatile uint32_t* GPCLR1;

    // These pointers point to start of address range where pwm channel control
    // registers are memory mapped in the virtual address space.
    static volatile uint32_t* virtual_memory_pwm0_base_ptr;
    static volatile uint32_t* virtual_memory_pwm1_base_ptr;

    // CONTROL REGISTER POINTERS FOR PWM0
    // Each register is 32 bits long
    static  volatile uint32_t* PWM0CTL;
    static volatile uint32_t* PWM0RNG1;
    static volatile uint32_t* PWM0DAT1;

    // CONTROL REGISTER POINTERS FOR PWM1
    // Each register is 32 bits long
    static volatile uint32_t* PWM1CTL;
    static volatile uint32_t* PWM1RNG1;
    static volatile uint32_t* PWM1DAT1;

    // This pointer points to start of address range where
    // Clockmanager registers are memory mapped in the virtual address space.
    static volatile uint32_t* virtual_memory_clock_manager_base_address;

    // Control registers for clock manager
    static volatile uint32_t* CM_GP0CTL;
    static volatile uint32_t* CM_GP0DIV;

    // Handles the memorymapping the physical address range
    // where control registers are located to virtual address space.
    // this method will be called by the classes constructor
    static void memMampRegisters();
};


#endif // BCM_2711_DRIVER_HH

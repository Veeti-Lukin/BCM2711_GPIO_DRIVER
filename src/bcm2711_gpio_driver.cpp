#include "BCM2711_GPIO_Driver/bcm2711_gpio_driver.h"

#include <unistd.h> // for sleep()
#ifndef EMULATE_GPIO_DRIVER
    #include <system_error>
    #include <fcntl.h>  // for open()
    #include <sys/mman.h> // for mmap() and global varaibles helping in that
    #include <errno.h>  // for errno errors
    #include <cstring> // to be used with strerror()
#endif

// "*" are placeholders spots that will be replaced
// Check BCM2711_DRIVER_EXCEPTION in bcm2711_gpio_driver.hh for more
const std::string INVALID_PIN_NUM_ERROR = "Pin number: * is not a valid GPIO pin num."
                                          " Valid pin numbers are 0-*";
const std::string INVALID_PIN_FUNCTIONALITY = "GPIO pin: * functionality mode"
                                              " is not set to be *";
const std::string MEMORY_MAPPING_ERROR = "Error when trying to memory map * : *";

//Define static class members
volatile uint32_t* BCM2711_GPIO_DRIVER::virtual_memory_gpio_base_ptr = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::GPFSEL0 = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::GPFSEL1 = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::GPFSEL2 = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::GPFSEL3 = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::GPFSEL4 = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::GPFSEL5 = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::GPSET0 = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::GPSET1 = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::GPCLR0 = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::GPCLR1 = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::virtual_memory_pwm0_base_ptr = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::PWM0CTL = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::PWM0RNG1 = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::PWM0DAT1 = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::PWM0RNG2 = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::PWM0DAT2 = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::virtual_memory_clock_manager_base_address = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::CM_GP0CTL = nullptr;
volatile uint32_t* BCM2711_GPIO_DRIVER::CM_GP0DIV = nullptr;


void BCM2711_GPIO_DRIVER::initialize() {
    #ifndef EMULATE_GPIO_DRIVER
    memMampRegisters();

    // REGISTER OFFSETS ARE FOUND IN BCM2711_peripherals.pdf SECTION 5.2
    // Pointer arithmetics works by multiplying the int to be added with
    // the size of the pointer which the int is getting added to.
    // So in this case +1 means 32bits or 4 bytes increacment.
    GPFSEL0 = virtual_memory_gpio_base_ptr;
    GPFSEL1 = virtual_memory_gpio_base_ptr + 1; //4 byte jump between registers
    GPFSEL2 = virtual_memory_gpio_base_ptr + 2;
    GPFSEL3 = virtual_memory_gpio_base_ptr + 3;
    GPFSEL4 = virtual_memory_gpio_base_ptr + 4;
    GPFSEL5 = virtual_memory_gpio_base_ptr + 5;
    GPSET0 = virtual_memory_gpio_base_ptr + 7;  // 8 byte jump between register "sections"
    GPSET1 = virtual_memory_gpio_base_ptr + 8;
    GPCLR0 = virtual_memory_gpio_base_ptr + 10;
    GPCLR1 = virtual_memory_gpio_base_ptr + 11;

    PWM0CTL = virtual_memory_pwm0_base_ptr;
    PWM0RNG1 = virtual_memory_pwm0_base_ptr + 4;
    PWM0DAT1 = virtual_memory_pwm0_base_ptr + 5;
    PWM0RNG2 = virtual_memory_pwm0_base_ptr + 8;
    PWM0DAT2 = virtual_memory_pwm0_base_ptr + 9;


    CM_GP0CTL = virtual_memory_clock_manager_base_address + 28;
    CM_GP0DIV = virtual_memory_clock_manager_base_address + 29;
    #endif

    #ifdef EMULATE_GPIO_DRIVER
    GPFSEL0 = new uint32_t;
    GPFSEL1 = new uint32_t;
    GPFSEL2 = new uint32_t;
    GPFSEL3 = new uint32_t;
    GPFSEL4 = new uint32_t;
    GPFSEL5 = new uint32_t;
    GPSET0 = new uint32_t;
    GPSET1 = new uint32_t;
    GPCLR0 = new uint32_t;
    GPCLR1 = new uint32_t;

    PWM0CTL = new uint32_t;
    PWM0RNG1 = new uint32_t;
    PWM0DAT1 = new uint32_t;
    PWM0RNG2 = new uint32_t;
    PWM0DAT2 = new uint32_t;

    CM_GP0CTL = new uint32_t;
    CM_GP0DIV = new uint32_t;
    #endif
}

void BCM2711_GPIO_DRIVER::terminate() {
    /* todo: gives seg fault
    //unmap the mapped memory
    munmap((void*)virtual_memory_gpio_base_ptr, 4096);
    munmap((void*)virtual_memory_pwm0_base_ptr, 4096);
    munmap((void*)virtual_memory_pwm1_base_ptr, 4096);
    munmap((void*)virtual_memory_clock_manager_base_address, 4096);
    /*if (memfd >= 0) {
        close(memfd);
    }*
     */
    #ifdef EMULATE_GPIO_DRIVER
        delete GPFSEL0;
        delete GPFSEL1;
        delete GPFSEL2;
        delete GPFSEL3;
        delete GPFSEL4;
        delete GPFSEL5;
        delete GPSET0;
        delete GPSET1;
        delete GPCLR0;
        delete GPCLR1;

        delete PWM0CTL;
        delete PWM0RNG1;
        delete PWM0DAT1;

        delete CM_GP0CTL;
        delete CM_GP0DIV;
    #endif
}

void BCM2711_GPIO_DRIVER::setPinGpioFunction(uint8_t pin_num, PinFunction func) {
    // A pin’s functionality is defined via one of 8 alternative functions.
    // Encoding each function requires 3 bits, 000 - 111.
    // so 32 bit functionality register can control 10 pins with 2 bits left over.
    // Therefore, to control functionality of all pins there are total 6 functionality registers.
    // These registers are pointed to by <GPFSEL0> - <GPFSEL1>

    // Finding the correct register where functionality of pin by *pin_num* is controlled
    // Since each register controls 10 pins and Integer division rounds down
    // Correct register for pin 17 would be found like this:
    // GPFSEL0 + 17/10(rounds down to 0) = GPFSEL0 + 0(pointer arithmetics) = GPFSEL0
    volatile uint32_t *pins_gpio_function_selection_register = GPFSEL0 + pin_num / 10;

    // Finding the starting index where the the 3 bits
    // controlling functionality of pin by *pin_num* are located.
    // By exctracting the least signifigant digit of the *pin_num*
    // and multiplaying it with amount of bits per pin.
    uint8_t lsb_control_bit_index = (pin_num % 10) * 3;
    // Forming a bitmask, that defines which bits in the register are going to get affected
    // Bits set 1 to are the ones that will be affected.
    // Shifting 3 ones so that the leftmost 1 will be at the <lsb_control_bit_index>
    uint32_t mask = (0b111 << lsb_control_bit_index);
    /*uint32_t mask = (1 << lsb_control_bit_index) |
            (1 << (lsb_control_bit_index+1)) | (1 << (lsb_control_bit_index+2));
    */

    // First masked bits will be ANDed with invesred mask
    // Therefore masked bits will be set to 0 and other bits wont be affected
    // Then the bits of the func (3 encoding bits) will be ORed to the masked spot
    *pins_gpio_function_selection_register =
            (*pins_gpio_function_selection_register & ~(mask)) | ((uint8_t)func << lsb_control_bit_index);
}

PinFunction BCM2711_GPIO_DRIVER::getGpioFunction(uint8_t pin_num) {
    // Finding the correct register where functionality of pin by *pin_num* is controlled
    // Since each register controls 10 pins and Integer division rounds down
    // Correct register for pin 17 would be found like this:
    // GPFSEL0 + 17/10(rounds down to 0) = GPFSEL0 + 0(pointer arithmetics) = GPFSEL0
    volatile uint32_t *pins_gpio_function_selection_register = GPFSEL0 + pin_num / 10;

    uint32_t register_value_copy = *pins_gpio_function_selection_register;
    // Finding the starting index where the the 3 bits
    // controlling functionality of pin by *pin_num* are located.
    uint8_t lsb_control_bit_index = (pin_num % 10) * 3;

    //uint32_t mask = (0b111 << lsb_control_bit_index);

    return PinFunction(((1 << 3) - 1) & (register_value_copy >> lsb_control_bit_index));
}

bool BCM2711_GPIO_DRIVER::setOuputPin(uint8_t pin_num, bool value) {
    // Chek that *pun_num* is a valid GPIO pins numer
    if (pin_num > K_GPIO_PIN_AMOUNT - 1) {
        /*throw GPIO_DRIVER_EXCEPTION(
                    std::string("Pin number is not a valid GPIO pin num. ") +
                    std::string("Valid pin numbers are 0-") +
                    std::to_string(GPIO_PIN_AMOUNT-1));
        */
        throw GPIO_DRIVER_EXCEPTION(INVALID_PIN_NUM_ERROR,
                                    {std::to_string(pin_num), std::to_string(K_GPIO_PIN_AMOUNT - 1)});
    }

    //Chek that GPIO pins functionality is set to OUTPUT
    //TODO EI TAIDA TOIMIA
    if (getGpioFunction(pin_num) != PinFunction::OUTPUT) {
        throw GPIO_DRIVER_EXCEPTION(INVALID_PIN_FUNCTIONALITY,
                                    {std::to_string(pin_num), "OUTPUT"});
        /*throw GPIO_DRIVER_EXCEPTION(
                    std::string("GPIO pin:") + std::to_string(pin_num) +
                    std::string("functionality is not set to be OUTPUT"));*/
    }

    // pointer thst is used right controlling register to set/clear
    // a GPIO pin, referenced by *pin_num*
    volatile uint32_t *pins_control_register;

    // Setting an Clearing output pin is seperated to differrnt registers in BCM2711
    // Writing 1 in the poisition of the pin in the corresponding
    // GPIO output set register is used to set a GPIO pin.
    if (value == HIGH) {
        // set pin
        pins_control_register = GPSET0 + pin_num / 32;
    }
        // Writing 1 in the poisition of the pin in the corresponding
        // GPIO output clear registers is used to clear a GPIO pin.
    else // value == LOW
    {
        // clear pin
        pins_control_register = GPCLR0 + pin_num / 32;
    }

    *pins_control_register = 1 << pin_num;
    return true;
}

void BCM2711_GPIO_DRIVER::configPwmPin(uint32_t freq, uint8_t duty_cycle_precentage, uint8_t pin_num) {/*
    // Stop PWM clock
    *CM_GP0CTL |=  0x5A | 0x01;
    // while clock busy
    while ((*CM_GP0CTL & 0x80) != 0)
        usleep(1);

    //set clock divisor
    *CM_GP0DIV = 0x5A | (16 << 12);
    // set clock source to osc and enable
    *CM_GP0CTL |=  0x5A | 0x11;

    // Forming a bitmask, that defines which bits in the register are going to get affected
    // Bits set 1 to are the ones that will be affected.
    // Shifting 3 ones so that the leftmost 1 will be at the <lsb_control_bit_index>
    uint32_t mask = (0b1 << 7);
    /*uint32_t mask = (1 << lsb_control_bit_index) |
            (1 << (lsb_control_bit_index+1)) | (1 << (lsb_control_bit_index+2));
    */

    // First masked bits will be ANDed with invesred mask
    // Therefore masked bits will be set to 0 and other bits wont be affected
    // Then the bits of the func (3 encoding bits) will be ORed to the masked spot
    // set markspace mode on
    /*
    *PWM0CTL = (*PWM0CTL & ~(mask)) | (1 << 7);
    //set channel enabled
    *PWM0CTL |= 1;
    // set range
    *PWM0RNG1 = 1024;

    *PWM0DAT1 = 1;*/

    /*
    // TÄMÄ TOIMIII
    // Configure PWM pin as ALT_FUNC5 (PWM functionality)
        setup_gpio_function(18, ALT_FUNC5);

        // Stop PWM clock
        *CM_GP0CTL = 0x5A << 24 | 0x01;
        // Wait until clock is not busy
        while ((*CM_GP0CTL & 0x80) != 0)
            usleep(1);

        // Set clock divisor for desired PWM frequency (e.g., 1 kHz)
        // Assuming an example divisor value of 50 (use your desired value)
        *CM_GP0DIV = 0x5A << 24 | (50 << 12);

        // Set clock source to oscillator and enable
        *CM_GP0CTL = 0x5A << 24 | 0x11;

        // Configure PWM control and range registers
        *PWM0CTL = 0;         // Disable PWM for now
        *PWM0RNG1 = 1024;     // PWM range/divisor value

        // Enable MSEN1 and use PWM mode
        *PWM0CTL |= (1 << 7) | (1 << 0);

        // Set channel enabled
        *PWM0CTL |= 1;

        // Set initial PWM duty cycle (e.g., 50%)
        *PWM0DAT1 = 512;      // Adjust for your desired duty cycle*/

    // Determine whether to use channel1 or channel2 of the PWM0 controller
    // based on the selected pin number
    volatile uint32_t *pwm_rng_register = nullptr;
    volatile uint32_t *pwm_dat_register = nullptr;

    uint8_t pwm_channel = 0;

    if (pin_num == 12 || pin_num == 18) {
        pwm_rng_register = PWM0RNG1;
        pwm_dat_register = PWM0DAT1;
        pwm_channel = 1;

    } else if (pin_num == 13 || pin_num == 19) {
        pwm_rng_register = PWM0RNG2;
        pwm_dat_register = PWM0DAT2;
        pwm_channel = 2;

    } else {
        throw GPIO_DRIVER_EXCEPTION("Invalid GPIO pin number for PWM");
    }

    // Stop PWM clock
    *CM_GP0CTL = K_CLOCK_MANAGER_PASSWORD | 0x01;

    #ifndef EMULATE_GPIO_DRIVER
    // Wait until clock is not busy
    while ((*CM_GP0CTL & 0x80) != 0)
        usleep(1);
    #endif

    // Divisor = Clock frequency / Desired PWM frequency
    // Assuming clock frequency is 19.2 MHz (default oscillator)
    // and desired PWM frequency is 5 kHz
    // Divisor = 19200000 / 5000 = 3840
    uint32_t clock_divisor = K_DEFAULT_CLOCK_FREQ / freq;

    *CM_GP0DIV = K_CLOCK_MANAGER_PASSWORD | (clock_divisor << 12);

    // Set clock source to oscillator and enable
    *CM_GP0CTL = K_CLOCK_MANAGER_PASSWORD | 0x11;

    // Configure PWM control and range registers
    uint32_t pwm_range = clock_divisor;
    *pwm_rng_register = pwm_range;      // PWM range/divisor value for 5 kHz (19200000 / 5000)

    // Set MSEN bit for the channel
    // can be used to change between MarkSpace mode and hardware pwm algorithm
    if (pwm_channel == 1) {
        *PWM0CTL |= (1 << 7);
    } else {
        *PWM0CTL |= (1 << 15);
    }

    // set duty cycle for the channel
    // todo: does this go in as a float(iee-754) or int
    // todo: bcs if it goes in as a float it might flow out of the register if 64 bit
    // todo: and if it is only 32 bit it will still be large as hell
    *pwm_dat_register = pwm_range * (duty_cycle_precentage / 100.0);
}

void BCM2711_GPIO_DRIVER::enablePwmPin(uint8_t pin_num, bool enable) {
    // Determine whether to use channel1 or channel2 of the PWM0 controller
    // based on the selected pin number
    if (pin_num == 12 || pin_num == 18) {
        // For GPIO pins 12 and 18, enable or disable channel1
        if (enable) {
            *PWM0CTL |= (1 << 0);
        } else {
            *PWM0CTL &= ~(1 << 0);
        }
    } else if (pin_num == 13 || pin_num == 19) {
        // For GPIO pins 13 and 19, enable or disable channel2
        if (enable) {
            *PWM0CTL |= (1 << 8);
        } else {
            *PWM0CTL &= ~(1 << 8);
        }
    } else {
        throw GPIO_DRIVER_EXCEPTION("Invalid GPIO pin number for PWM");
    }


}

void BCM2711_GPIO_DRIVER::memMampRegisters() {
    #ifndef EMULATE_GPIO_DRIVER
    // check if progrm is run as root/sudo
    // (linux-kernel-call geteuid:
    //          https://man7.org/linux/man-pages/man2/getuid.2.html)
    if (geteuid() != 0) {
        throw GPIO_DRIVER_EXCEPTION("Driver needs to be run with ROOT privilages");
    }

    // Open the master /dev/mem devicefile for obtaining the handle to physical memory
    // will set errno variable in case of error
    // (linux-kernel-call open:
    //          https://man7.org/linux/man-pages/man2/open.2.html)
    auto memfd = open("/dev/mem", (O_RDWR | O_SYNC));

    // test if opening /dev/mem failed
    if (memfd < 0) {
        // strerror() turns the errno integer into a human readable error message.
        throw GPIO_DRIVER_EXCEPTION(strerror(errno));
    }

    // Memory map the physical address range where control registers are located
    // to virtual address space.
    // Will set errno variable in case of error.
    // first argument being NULL the kernel chooses where in the virtual address space to map
    // https://man7.org/linux/man-pages/man2/mmap.2.html
    virtual_memory_gpio_base_ptr = (uint32_t *) mmap(NULL, 4096, (PROT_READ | PROT_WRITE),
                                                     MAP_SHARED, memfd, K_GPIO_BASE_ADDRESS);

    // check if memorymapping failed
    if (virtual_memory_gpio_base_ptr == MAP_FAILED) {
        throw GPIO_DRIVER_EXCEPTION(MEMORY_MAPPING_ERROR,
                                    {"GPIO REGISTERS", strerror(errno)});
    }

    // Memorymap both PMW control registers addres ranges to vritual address space
    virtual_memory_pwm0_base_ptr = (uint32_t *) mmap(NULL, 4096, (PROT_READ | PROT_WRITE),
                                                     MAP_SHARED, memfd, K_PWM0_BASE_ADDRESS);

    // check if memorymapping failed for PWM controllers
    if (virtual_memory_pwm0_base_ptr == MAP_FAILED) {
        throw GPIO_DRIVER_EXCEPTION(MEMORY_MAPPING_ERROR,
                                    {"PWM REGISTERS", strerror(errno)});
    }

    virtual_memory_clock_manager_base_address =
            (uint32_t *) mmap(NULL, 4096, (PROT_READ | PROT_WRITE),
                              MAP_SHARED, memfd, K_CLOCK_MANAGER_BASE_ADDRESS);

    if (virtual_memory_clock_manager_base_address == MAP_FAILED) {
        throw GPIO_DRIVER_EXCEPTION(MEMORY_MAPPING_ERROR,
                                    {"CLOCK MANAGER REGISTERS", strerror(errno)});
    }
    #endif
}

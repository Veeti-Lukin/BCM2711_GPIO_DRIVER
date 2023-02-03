#include <system_error>
#include <unistd.h>
#include <fcntl.h>  // for open()
#include <sys/mman.h> // for mmap()
#include <errno.h>  // for errno errors
#include <cstring> // to be used with strerror()
#include "bcm2711_gpio_driver.hh"

BCM2711_GPIO_DRIVER::BCM2711_GPIO_DRIVER()
{

    gpio_init();

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
}

void BCM2711_GPIO_DRIVER::setup_gpio_function(uint8_t pin_num, pin_function func)
{
    // A pin’s functionality is defined via one of 8 alternative functions.
    // Encoding each function requires 3 bits, 000 - 111.
    // so 32 bit functionality register can control 10 pins with 2 bits left over.
    // Therefore to control functionality of all pins there are total 6 functionality registers.
    // These registers are pointed to by <GPFSEL0> - <GPFSEL1>

    // Finding the correct register where functionality of pin by *pin_num* is controlled
    // Since each register controls 10 pins and Integer division rounds down
    // Correct register for pin 17 would be found like this:
    // GPFSEL0 + 17/10(rounds down to 0) = GPFSEL0 + 0(pointer arithmetics) = GPFSEL0
    volatile uint32_t* pins_gpio_function_selection_register = GPFSEL0 + pin_num/10;

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
            (*pins_gpio_function_selection_register & ~(mask)) | (func << lsb_control_bit_index);
}

bool BCM2711_GPIO_DRIVER::set_output_pin(uint8_t pin_num, bool value)
{

}

void BCM2711_GPIO_DRIVER::gpio_init()
{
    // check if progrm is run as root/sudo
    // (linux-kernel-call geteuid:
    //          https://man7.org/linux/man-pages/man2/getuid.2.html)
    if (geteuid() != 0)
    {
        throw BCM2711_GPIO_DRIVER_EXCEPTION("Driver needs to be run with ROOT privilages");
    }

    // Open the master /dev/mem devicefile for obtaining the handle to physical memory
    // will set errno variable in case of error
    // (linux-kernel-call open:
    //          https://man7.org/linux/man-pages/man2/open.2.html)
    auto memfd = open("/dev/mem", (O_RDWR | O_SYNC));

    // test if opening /dev/mem failed
    if (memfd < 0)
    {
        // strerror() turns the errno integer into a human readable error message.
        throw BCM2711_GPIO_DRIVER_EXCEPTION(strerror(errno));
    }

    // Memory map the physical address range where control registers are located
    // to virtual address space.
    // Will set errno variable in case of error.
    // first argument being NULL the kernel chooses where in the virtual address space to map
    // https://man7.org/linux/man-pages/man2/mmap.2.html
    virtual_memory_gpio_base_ptr = (uint32_t*)mmap(NULL, 4096, (PROT_READ | PROT_WRITE),
                                    MAP_SHARED, memfd, GPIO_BASE_ADDRESS);

    // check if memorymapping failed
    if (virtual_memory_gpio_base_ptr == MAP_FAILED)
    {
       throw BCM2711_GPIO_DRIVER_EXCEPTION(strerror(errno));
    }
}

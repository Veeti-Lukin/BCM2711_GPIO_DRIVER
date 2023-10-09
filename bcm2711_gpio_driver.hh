/// C++ class library tha allows use of GPIO pins in raspberry 4b
/// all operations on BCM2711 peripherals are accomplished by
/// manipulating the registers associated for that type of peripheral.

#ifndef BCM_2711_DRIVER_HH
#define BCM_2711_DRIVER_HH

#include<cstdint>
#include<string>
#include<exception>
#include<vector>

// This means pin HIGH, true, 3.3 volts on a pin.
// Can be given as argument to <set_output_pin()>
const uint8_t HIGH = 1;
// This means pin LOW, false, 0 volts on a pin.
// Can be given as argument to <set_output_pin()>
const uint8_t LOW = 0;

// ALTERNATIVE PIN FUNCTIONS PER PIN CAN BE FOUND IN IN BCM2711_peripherals.pdf SECTION 5.3
// These enum constant can be used to define the operation of the GPIO pin.
// Each of the 58 GPIO pins has at least two alternative functions.
// Encoding each function requires 3 bits, 000 - 111.
// One of these constants can be given as arg to <setup_gpio_function()>
enum PinFunction
{
    INPUT = 0b000,
    OUTPUT = 0b001,
    ALT_FUNC0 = 0b100,
    ALT_FUNC1 = 0b101,
    ALT_FUNC2 = 0b110,
    ALT_FUNC3 = 0b111,
    ALT_FUNC4 = 0b011,
    ALT_FUNC5 = 0b010
};

struct PwmPin {
    uint8_t pin_num;
    PinFunction pwm_func;
};


const PwmPin K_PWM_PIN12{12, ALT_FUNC0};
const PwmPin K_PWM_PIN13{13, ALT_FUNC0};
const PwmPin K_PWM_PIN18{18, ALT_FUNC5};
const PwmPin K_PWM_PIN19{12, ALT_FUNC5};

// Exception class. Instances of this class will be thrown in case of errors
// or faulty use of the driver class
class BCM2711_GPIO_DRIVER_EXCEPTION : public std::exception
{
public:
    // Default constructor where  error message is given as parameter
    BCM2711_GPIO_DRIVER_EXCEPTION(const std::string msg) :
        message_("Error: "+msg){}
    // Constructor where error message can have placeholders that are marked with "*"
    // values from *values_to_format* will be formatted in order to these spots
    BCM2711_GPIO_DRIVER_EXCEPTION(const std::string msg,
                                  std::vector<std::string> values_to_format) :
        message_("Error: " + msg), values_to_format_(values_to_format)
    {
        // format the values to to placeholder spots
        for (std::string& value : values_to_format)
        {
            message_.replace(message_.find("*"), 1, value);

        }
    }

    virtual const char* what() const noexcept override
        {
            return message_.c_str();
        }

private:
    std::string message_;
    std::vector<std::string> values_to_format_;
};




class BCM2711_GPIO_DRIVER
{
public:
    // default constructor
    BCM2711_GPIO_DRIVER();
    // default destructor
    ~BCM2711_GPIO_DRIVER();

    void setPinGpioFunction(uint8_t pin_num, PinFunction func);
    PinFunction getGpioFunction(uint8_t pin_num);

    // clear if value 0
    // set to high if value 1
    // checkkaa onko oikeesti output pinni ja palauttaa false jo ei
    bool setOuputPin(uint8_t pin_num, bool value);

    void configPwmPin(uint32_t freq, uint8_t duty_cycle_precentage, uint8_t pin_num);
    void enablePwmPin(uint8_t pin_num, bool enable);

private:
    // bcm2711-peripherals.pdf section 5.2 states:
    // "The GPIO register base address is 0x7e20_0000".
    // However addresses starting at 0x7e00_0000 are in VPU address space.
    // If bcm2711 is running in "low peripheral mode" (default on rpi4)
    // this address range is re-mapped to start at 0xFE00_0000
    // in arm physical address space.
    static const uint32_t K_GPIO_BASE_ADDRESS = 0xFE200000;

    // There are 58 General-Purpose Input/Output (GPIO) pin lines in BCM2711
    // However rpi4 has only 40 pins, of which 28 are BCM2711 GPIO pins
    static const uint8_t K_GPIO_PIN_AMOUNT = 58;

    // This pointer points to start (more spesifically to 32 first bits)
    // of address range where GPIO controlling registers are memory mapped
    // in the virtual address space.
    volatile uint32_t* virtual_memory_gpio_base_ptr;


    // CONTROL REGISTER POINTERS
    // Each register is 32 bits long
    volatile uint32_t* GPFSEL0;
    volatile uint32_t* GPFSEL1;
    volatile uint32_t* GPFSEL2;
    volatile uint32_t* GPFSEL3;
    volatile uint32_t* GPFSEL4;
    volatile uint32_t* GPFSEL5;
    volatile uint32_t* GPSET0;
    volatile uint32_t* GPSET1;
    volatile uint32_t* GPCLR0;
    volatile uint32_t* GPCLR1;

    // BCM2711 has two hardware pwm channel on board.
    // Each of these channels have its own controlling registers.
    //0x7e20c000
    static const uint32_t K_PWM0_BASE_ADDRESS = 0xFE20c000;
    //0x7e20c800.
    static const uint32_t K_PWM1_BASE_ADDRESS =  0xFE20c800;

    // These pointers point to start of address range where pwm channel control
    // registers are memory mapped in the virtual address space.
    volatile uint32_t* virtual_memory_pwm0_base_ptr;
    volatile uint32_t* virtual_memory_pwm1_base_ptr;

    // CONTROL REGISTER POINTERS FOR PWM0
    // Each register is 32 bits long
    volatile uint32_t* PWM0CTL;
    volatile uint32_t* PWM0RNG1;
    volatile uint32_t* PWM0DAT1;

    // CONTROL REGISTER POINTERS FOR PWM1
    // Each register is 32 bits long
    volatile uint32_t* PWM1CTL;
    volatile uint32_t* PWM1RNG1;
    volatile uint32_t* PWM1DAT1;

    // 0x7e101000
    static const uint32_t  K_CLOCK_MANAGER_BASE_ADDRESS = 0xFE101000;

    // This pointer points to start of address range where
    // Clockmanager registers are memory mapped in the virtual address space.
    volatile uint32_t* virtual_memory_clock_manager_base_address;

    // Control registers for clock manager
    volatile uint32_t* CM_GP0CTL;
    volatile uint32_t* CM_GP0DIV;

    static const uint32_t K_DEFAULT_CLOCK_FREQ = 19200000;
    static const uint32_t K_CLOCK_MANAGER_PASSWORD = 0x5A << 24;

    // Handles the memorymapping the physical address range
    // where control registers are located to virtual address space.
    // this method will be called by the classes constructor
    void memMampRegisters();
};


#endif // BCM_2711_DRIVER_HH

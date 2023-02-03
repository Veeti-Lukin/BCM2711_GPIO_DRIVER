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
enum pin_function
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

// Exception class. Instances of this class will be thrown in case of errors
// or faulty use of the driver class
class BCM2711_GPIO_DRIVER_EXCEPTION : public std::exception
{
public:
    // Default constructor where  error message is given as parameter
    BCM2711_GPIO_DRIVER_EXCEPTION(const std::string msg) :
        message("Error: "+msg){}
    // Constructor where error message can have placeholders that are marked with "*"
    // values from *values_to_format* will be formatted in order to these spots
    BCM2711_GPIO_DRIVER_EXCEPTION(const std::string msg,
                                  std::vector<std::string> values_to_format) :
        message("Error: " + msg), values_to_format(values_to_format)
    {
        // format the values to to placeholder spots
        for (std::string& value : values_to_format)
        {
            message.replace(message.find("*"), 1, value);

        }
    }

    virtual const char* what() const noexcept override
        {
            return message.c_str();
        }

private:
        std::string message;
        std::vector<std::string> values_to_format;
};

class BCM2711_GPIO_DRIVER
{
public:
    // default constructor
    BCM2711_GPIO_DRIVER();
    // default destructor
    ~BCM2711_GPIO_DRIVER();

    void setup_gpio_function(uint8_t pin_num, pin_function func);

    // clear if value 0
    // set to high if value 1
    // checkkaa onko oikeesti output pinni ja palauttaa false jo ei
    bool set_output_pin(uint8_t pin_num, bool value);

private:

};


#endif // BCM_2711_DRIVER_HH

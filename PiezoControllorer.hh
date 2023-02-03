#ifndef PIEZOCONTROLLORER_HH
#define PIEZOCONTROLLORER_HH

#include "bcm2711_gpio_driver.hh"
#include <thread>

struct NoteFrequencies {
    // Octave 1
    static constexpr double C1  = 32.70;
    static constexpr double Cs1 = 34.65;
    static constexpr double D1  = 36.71;
    static constexpr double Ds1 = 38.89;
    static constexpr double E1  = 41.20;
    static constexpr double F1  = 43.65;
    static constexpr double Fs1 = 46.25;
    static constexpr double G1  = 49.00;
    static constexpr double Gs1 = 51.91;
    static constexpr double A1  = 55.00;
    static constexpr double As1 = 58.27;
    static constexpr double B1  = 61.74;

    // Octave 2
    static constexpr double C2  = 65.41;
    static constexpr double Cs2 = 69.30;
    static constexpr double D2  = 73.42;
    static constexpr double Ds2 = 77.78;
    static constexpr double E2  = 82.41;
    static constexpr double F2  = 87.31;
    static constexpr double Fs2 = 92.50;
    static constexpr double G2  = 98.00;
    static constexpr double Gs2 = 103.83;
    static constexpr double A2  = 110.00;
    static constexpr double As2 = 116.54;
    static constexpr double B2  = 123.47;

    // Octave 3
    static constexpr double C3  = 130.81;
    static constexpr double Cs3 = 138.59;
    static constexpr double D3  = 146.83;
    static constexpr double Ds3 = 155.56;
    static constexpr double E3  = 164.81;
    static constexpr double F3  = 174.61;
    static constexpr double Fs3 = 185.00;
    static constexpr double G3  = 196.00;
    static constexpr double Gs3 = 207.65;
    static constexpr double A3  = 220.00;
    static constexpr double As3 = 233.08;
    static constexpr double B3  = 246.94;

    // Octave 4
    static constexpr double C4  = 261.63;
    static constexpr double Cs4 = 277.18;
    static constexpr double D4  = 293.66;
    static constexpr double Ds4 = 311.13;
    static constexpr double E4  = 329.63;
    static constexpr double F4  = 349.23;
    static constexpr double Fs4 = 369.99;
    static constexpr double G4  = 392.00;
    static constexpr double Gs4 = 415.30;
    static constexpr double A4  = 440.00;
    static constexpr double As4 = 466.16;
    static constexpr double B4  = 493.88;

    // Octave 5
    static constexpr double C5  = 523.25;
    static constexpr double Cs5 = 554.37;
    static constexpr double D5  = 587.33;
    static constexpr double Ds5 = 622.25;
    static constexpr double E5  = 659.26;
    static constexpr double F5  = 698.46;
    static constexpr double Fs5 = 739.99;
    static constexpr double G5  = 783.99;
    static constexpr double Gs5 = 830.61;
    static constexpr double A5  = 880.00;
    static constexpr double As5 = 932.33;
    static constexpr double B5  = 987.77;

    // Octave 6
    static constexpr double C6  = 1046.50;
    static constexpr double Cs6 = 1108.73;
    static constexpr double D6  = 1174.66;
    static constexpr double Ds6 = 1244.51;
    static constexpr double E6  = 1318.51;
    static constexpr double F6  = 1396.91;
    static constexpr double Fs6 = 1479.98;
    static constexpr double G6  = 1567.98;
    static constexpr double Gs6 = 1661.22;
    static constexpr double A6  = 1760.00;
    static constexpr double As6 = 1864.66;
    static constexpr double B6  = 1975.53;
};


class PiezoControllorer
{
public:
    enum SoundQueueType {
        queue,
        override
    };

    PiezoControllorer(BCM2711_GPIO_DRIVER &gpio_driver, PwmPin pin);
    void setVolume(uint8_t percentage);

    void playSound(uint16_t freq, uint16_t duration_ms, SoundQueueType prot = SoundQueueType::override);
    void playSoundPattern();

private:
    uint8_t volume_percentage_;
    uint8_t pin_num_;
    BCM2711_GPIO_DRIVER gpio_driver_;

    std::vector<std::pair<uint16_t, uint16_t>> sound_queue_;
    std::thread* working_thread_;
};

#endif // PIEZOCONTROLLORER_HH

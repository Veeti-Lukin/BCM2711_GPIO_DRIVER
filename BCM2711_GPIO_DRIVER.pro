TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        CarController.cpp \
        MotorController.cpp \
        PiezoControllorer.cpp \
        bcm2711_gpio_driver.cpp \
        main.cpp

HEADERS += \
    CarController.hh \
    MotorController.hh \
    PiezoControllorer.hh \
    bcm2711_gpio_driver.hh


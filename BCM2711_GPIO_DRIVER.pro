TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        bcm2711_gpio_driver.cpp \
        main.cpp

HEADERS += \
    bcm2711_gpio_driver.hh


scp bcm2711_gpio_driver.* bcm2711_gpio_driver.* main.cpp MotorController.* CarController.* user@raspberrypi.local:tmp/
ssh user@raspberrypi.local "cd tmp && g++ *.cpp && sudo ./a.out "
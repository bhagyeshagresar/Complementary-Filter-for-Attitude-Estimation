# Smart Temperature Monitored Laptop Cooling Fan using STM32

I am working on a Smart Laptop Cooling Fan project. Following is the list of components I am using as of now:
1) 2.13 inch E-Paper Hat for Temperature Monitoring
2) 12V DC Brushless Fan, 0.3A
3) MLX90614 IR Temperature Sensor Module
4) WS2812B LEDS


# Schematic of the Project





## Drivers for SSD1680 EPD Driver
WIP on writing drivers for the EPD Driver SSD1680

## Drivers for MLX90614 IR Temperature Sensor
Added Drivers for MLX90614. The `mlx90614.h` and `mlx90614.c` files contain functions to read Temperature of the obj placed in front of the sensor.
The sensor has a resolution of 0.02C.


# Proportional Controller for fan control

I found some ICs which are used in personal computers like the MAX6653 fan controllers for temperature monitoring inside the cpu. These use temperature measure as an input and output pwm in different mode(automatic mode gives proportional pwm increase based on increase of temperature in the threshold). Based on this approach, the first controller to test will be a simple Proportional Controller.

The MLX90614 Temperature sensor will be used to measure the temperature. 

pwm_output = kp*e where e = Tmeas - Tset_point






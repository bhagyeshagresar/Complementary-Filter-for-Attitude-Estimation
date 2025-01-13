# Smart Temperature Monitored Laptop Cooling Fan using STM32

I am working on a Smart Laptop Cooling Fan project. Following is the list of components I am using as of now:
1) 2.13 inch E-Paper Hat for Temperature Monitoring
2) 5V DC Fan
3) MLX90614 IR Temperature Sensor Module
4) WS2812B LEDS


## Drivers for SSD1680 EPD Driver
WIP on writing drivers for the EPD Driver SSD1680

## Drivers for MLX90614 IR Temperature Sensor
Added Drivers for MLX90614. The `mlx90614.h` and `mlx90614.c` files contain functions to read Temperature of the obj placed in front of the sensor.
The sensor has a resolution of 0.02C.




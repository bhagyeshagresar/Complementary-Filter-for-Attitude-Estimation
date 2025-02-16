/*
 * speed_controller.c
 *
 *  Created on: Jan 29, 2025
 *      Author: bhagy
 */


#include "mlx90614.h"


volatile float kp = 0.01;


float get_speed_kp(){
	return kp;
}


void set_speed_gains(float set_kp){
	kp = set_kp;

}

//This function will be called by the TIM2 ISR ever 200ms to check the

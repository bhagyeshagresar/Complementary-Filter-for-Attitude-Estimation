/*
 * RCFilter.c
 *
 *  Created on: Mar 31, 2024
 *      Author: bhagy
 */

#include "RCFilter.h"

void RCFilter_Init(RCFilter *filt, float cutoffFreqHz, float sampleTimeS){

	//Compute RC constant  from Cut-Off Freq
	float RC = 1.0f/(6.283185*cutoffFreqHz);

	//Pre Compute Filters for low pass filter
	filt->coeff[0] = sampleTimeS/(sampleTimeS + RC);
	filt->coeff[1] = RC /(sampleTimeS + RC);

	//clear output buffer
	filt->out[0] = 0.0f;
	filt->out[1] = 0.0f;

}



//RC filter update function
float RCFilter_Update(RCFilter *filt, float inp){

	filt->out[1] = filt->out[0];


	//compute new output sample
	filt->out[0] = filt->coeff[0]*inp + filt->coeff[1]*filt->out[1];

	return  filt->out[0];


}

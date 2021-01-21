#include "Resampler.h"
#include <Arduino.h>

Downsampler::Downsampler(int downsampling_factor,float32_t* filter_coefs,int num_taps, int blocksize){

    //Filter is symetric.
    //reverse_filter_coefs(filter_coefs);
    filter_state = (float32_t*) malloc((num_taps+blocksize-1)*sizeof(float32_t));

    arm_status res = arm_fir_decimate_init_f32(&filter, num_taps,downsampling_factor,filter_coefs,filter_state,blocksize);
    if(res != ARM_MATH_SUCCESS){

        Serial.println("Failed to setup downsampler");
    }
}

void Downsampler::downsample(float32_t* input, float32_t* output,int num_samples){

    arm_fir_decimate_f32(&filter,input,output,num_samples);
}

void Downsampler::reverse_filter_coefs(float32_t* coefs, int num_taps){

    float32_t temp[num_taps];
    for(int i=0;i<num_taps;i++){

        temp[i] = coefs[num_taps-i-1];
    }
}
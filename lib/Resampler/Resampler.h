#ifndef DOWNSAMPLING_H
#define DOWNSAMPLING_H

#include <arm_math.h>

class Downsampler{

public:

    //blocksize is the maximum chunk of samples you want to process.
    Downsampler(int downsampling_factor,float32_t* filter_coefs,int num_taps,int blocksize);

    //num_samples must be multiple of downsampling factor, and less or equal to blocksize of downsampler.
    void downsample(float32_t* input, float32_t* output,int num_samples);

private:

    //Arm needs filter coefs in reverse order compared to how many filter generators give the coefficients.
    void reverse_filter_coefs(float32_t* coefs,int num_taps);

    float32_t* filter_state; //State variable for filter, don't touch!
    arm_fir_decimate_instance_f32 filter; //cmsis filter structure.

};

#endif
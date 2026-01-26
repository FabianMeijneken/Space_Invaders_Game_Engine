/**
  ******************************************************************************
  * @file           : dft.h
  * @brief          : dft header file.
  ******************************************************************************
  *
  * @author 		: Bram Laurens
  ******************************************************************************
  */

#include "main.h"


// External variables
extern int ADC_buffer[ADC_BUFFER_SIZE];

// Defines
#define Fs 10000.0f		 			// Sampling frequency
#define Nf (float)ADC_BUFFER_SIZE;
#define scale 3.0f / 4095.0f; 		// Scale factor to convert ADC value to absolut volts

// Functie prototypes
void init_LUTs();
float lut_lookup(float *lut, float index);
float DFT_compute_LUT(float DFT_frequency);

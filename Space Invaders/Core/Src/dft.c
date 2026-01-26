/**
  ******************************************************************************
  * @file           : dft.c
  * @brief          : Alle functies betreft de dft
  ******************************************************************************
  *
  * @author 		: Bram Laurens
  ******************************************************************************
  */

#include "dft.h"
#include "main.h"

// TODO hoezo static?
static float sin_LUT[ADC_BUFFER_SIZE];
static float cos_LUT[ADC_BUFFER_SIZE];



/**
 * @brief Fill the Look-Up Tables (LUTs) for sine and cosine values
 *
 */
void init_LUTs()
{
    for (int i = 0; i < ADC_BUFFER_SIZE; i++)
    {
        float phase = 2.0f * M_PI * ((float)i / (float)ADC_BUFFER_SIZE);
        sin_LUT[i] = sinf(phase);
        cos_LUT[i] = cosf(phase);
    }
}

/**
 * @brief Perform a LUT lookup with linear interpolation
 *
 * @param lut
 * @param index
 * @return float lut value at index
 */
float lut_lookup(float *lut, float index)
{
  //Initialize integer indices
  int i0 = (int)index;
  int i1 = i0 + 1;

  //Wrap around if needed
  if(i1 >= ADC_BUFFER_SIZE)
    i1 = 0; // wrap around

  //Calculate the fraction
  float frac = index - (float)i0;

  return lut[i0] + frac * (lut[i1] - lut[i0]);
}

/**
 * @brief Calculate the DFT of the ADC buffer at a specific frequency using LUTs
 *
 * @param DFT_frequency The frequency to analyze
 * @return float
 */
float DFT_compute_LUT(float DFT_frequency)
{
  if (DFT_frequency <= 0.0f || DFT_frequency > 5000.0f)
  {
    return 0.0f; // Frequency out of range
  }



  float real = 0.0f;
  float imag = 0.0f;

  //Calculate index stap, based on input frequency, sampling frequency and buffer size
  float index_step = DFT_frequency * (float)ADC_BUFFER_SIZE / Fs;
  float index = 0.0f;

  for (int n = 0; n < ADC_BUFFER_SIZE; n++)
  {
    float sample_volts = (float)ADC_buffer[n] * scale;
    float cos_val = lut_lookup(cos_LUT, index);
    float sin_val = lut_lookup(sin_LUT, index);

    real += sample_volts * cos_val;
    imag -= sample_volts * sin_val;

    index += index_step;

    // Wrap around index if it exceeds ADC_BUFFER_SIZE
    if (index >= (float)ADC_BUFFER_SIZE)
    {
      index -= (float)ADC_BUFFER_SIZE;
    }
  }

  // Average the real and imaginary parts
  real /= Nf;
  imag /= Nf;

  // Return the magnitude
  return sqrtf(real * real + imag * imag);
}

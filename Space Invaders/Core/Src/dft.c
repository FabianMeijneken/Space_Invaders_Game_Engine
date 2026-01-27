/**
  ******************************************************************************
  * @file           : dft.c
  * @brief          : Alle functies betreft de dft
  ******************************************************************************
  *
  * @author 		: Bram Laurens, Fabian Meijneken
  ******************************************************************************
  */

#include "dft.h"
#include "main.h"

// TODO hoezo static?
static float sin_LUT[ADC_BUFFER_SIZE];
static float cos_LUT[ADC_BUFFER_SIZE];

const commanddft_struct command_dft_list[CMD_amount] = {
  {0, 4009,4500, 0.7},    // Command 0: pauze
  {1, 1100, 1500, 0.7},   // Command 1: beweeg links
  {2, 1500, 2000, 0.5},  // Command 2: beweeg rechts
  {3, 600, 1100, 0.2}   // Command 3: schiet
};



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

float DFT_range_peak(float freq_min, float freq_max)
{
    float max_magnitude = 0.0f;

    // Scan through the frequency range with a step size
    float step_size = 50.0f; // Adjust step size as needed
    for (float freq = freq_min; freq <= freq_max; freq += step_size)
    {
        float magnitude = DFT_compute_LUT(freq);
        if (magnitude > max_magnitude)
        {
            max_magnitude = magnitude;
        }
    }

    return max_magnitude;
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





/**
 * @brief Deze functie handelt commando's af. Hiervoor ontvangt hij informatie van de DFT.
 *
 * @author Fabian meijneken
 * @date 24/01/2026
 *
 * @param command - Het commando dat uitgevoerd moet worden
 * @param player - pointer naar de player struct
 * @param bullets - pointer naar het eerste item in de bullets array
 * @param sprite - pointer naar het eerste item in de sprite array
 *
 * @return void
 */
void command_handler(uint8_t command, player_struct* player, bullet_struct* bullets, sprite_struct* sprite)
{
	switch (command)
	{
		// Pauzeer het spel
		case 0:
			if (game_status == GAME_RUNNING)
				game_status = GAME_PAUSED;
			if (game_status == GAME_PAUSED)
				game_status = GAME_RUNNING;
      break;

		// Beweeg naar links
		case 1:
			player_move(player, 1);
      break;

		// Beweeg naar rechts
		case 2:
			player_move(player, 0);
      break;

		// Schiet
		case 3:
			player_shoot(player, bullets, sprite);
      break;

	}
}

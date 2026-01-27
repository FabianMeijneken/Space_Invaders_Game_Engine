/**
  ******************************************************************************
  * @file           : dft.h
  * @brief          : dft header file.
  ******************************************************************************
  *
  * @author 		: Bram Laurens
  ******************************************************************************
  */

#ifndef DFT_H
#define DFT_H

#include "main.h"


// External variables
extern int ADC_buffer[ADC_BUFFER_SIZE];
extern enum gamestates game_status;


// Defines
#define Fs 10000.0f		 			// Sampling frequency
#define Nf (float)ADC_BUFFER_SIZE;
#define scale 3.0f / 4095.0f; 		// Scale factor to convert ADC value to absolut volts
#define CMD_amount 4

// Functie prototypes
void init_LUTs();
float lut_lookup(float *lut, float index);
float DFT_compute_LUT(float DFT_frequency);
float DFT_range_peak(float freq_min, float freq_max);
void command_handler(uint8_t command, player_struct* player, bullet_struct* bullets, sprite_struct* sprite);

typedef struct {
    uint8_t command_id;
    uint16_t frequency_min;
    uint16_t frequency_max;
    float threshold;
} commanddft_struct, *commandsdft_ptr;

// Defined in exactly one .c file (see `Core/Src/dft.c`).
extern const commanddft_struct command_dft_list[CMD_amount];

#endif /* DFT_H */





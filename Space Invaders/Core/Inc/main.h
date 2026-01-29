/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define buttons_interrupt_Pin GPIO_PIN_0
#define buttons_interrupt_GPIO_Port GPIOB
#define buttons_interrupt_EXTI_IRQn EXTI0_IRQn
#define buttons_col_2_Pin GPIO_PIN_8
#define buttons_col_2_GPIO_Port GPIOD
#define buttons_row_0_Pin GPIO_PIN_11
#define buttons_row_0_GPIO_Port GPIOD
#define LED_groen_Pin GPIO_PIN_12
#define LED_groen_GPIO_Port GPIOD
#define LED_oranje_Pin GPIO_PIN_13
#define LED_oranje_GPIO_Port GPIOD
#define LED_rood_Pin GPIO_PIN_14
#define LED_rood_GPIO_Port GPIOD
#define buttons_row_1_Pin GPIO_PIN_1
#define buttons_row_1_GPIO_Port GPIOD
#define buttons_col_0_Pin GPIO_PIN_6
#define buttons_col_0_GPIO_Port GPIOD
#define buttons_col_1_Pin GPIO_PIN_7
#define buttons_col_1_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define OUTPUT_FPGA
//#define OUTPUT_DFT_DEBUG

enum gamestates {
	GAME_RESET,					// Wordt gebruikt als het spel naar zijn origiele staat moet worden gereset.
	GAME_STARTING,				// Wordt automatisch na GAME_RESET getriggert, hierin wordt gewacht totdat alle UART Commando's uitgestuurd zijn.
	GAME_PAUSED,				// Tijdens een pauze beweegt niets meer, maar UART updates worden nogsteeds verzonden (voor bijv. tekst).
	GAME_RUNNING,
	GAME_OVER,
	GAME_WON
};



//----- Finn van Zijverden hieronder -----//
#define MAX_BULLET_AMOUNT 		5
#define BULLET_MOVE_SPEED 		10
#define BULLET_Y_MIN			20
#define BULLET_Y_MAX			450



//----- Kim lobstein hieronder -----//
#define MAX_Y_SPRITES 			400
#define MAX_X_SPRITES 			550
#define MIN_X_SPRITES 			50

#define SPRITES_PER_RIJ 		6
#define AANTAL_RIJEN_SPRITES 	5
#define SPRITE_X_MOVE_SPEED		5		// Deze variable bepaald de x_move_speed. Dit is geen define omdat de sprites naarmate het spel vordert sneller gaan bewegen.
#define SPRITE_Y_MOVE_SPEED 	20

#define SPRITES_X_OFFSET		50		// Start X van sprite linksboven
#define SPRITES_Y_OFFSET		30		// Start Y van sprite linksboven
#define SPRITES_X_AFSTAND		36 		// NIET VERANDEREN ZONDER FPGA TE VERANDEREN // (32 + 4)  Afstand tussen sprites (X) (begin tot volgende begin)
#define SPRITES_Y_AFSTAND		42		// (32 + 10) Afstand tussen sprites (Y) (begin tot volgende begin)

#define SPRITE_LENGTE 			32
#define SPRITE_BREEDTE 			32



//----- Fabian Meijneken hieronder -----//
// Game specific
#define SYSTEM_OBJ_ID			15
#define DEF_SPRITES_MOVE_FREQ 	30		// Deze factor / 3 is de waarde van de klokdeler vanaf 20 Hz.
#define BULLET_FREQUENCY		10		// Hoe hoger, hoe minder bullets: ... % BULLET_FREQUENCY == 1;
#define UART2_TX_BUFFER_SIZE	16		// De grootte van de UART TX buffer (hoeveelheid berichten)

// Player
#define PLAYER_X_START 			320
#define PLAYER_Y_START 			440
#define PLAYER_X_MIN			5
#define PLAYER_X_MAX			600
#define SPELER_BREEDTE 			20
#define SPELER_LENGTE 			22

#define PLAYER_MOVE_SPEED 		8
#define MAX_PLAYER_BULLET_COUNT 2

// Enemy
#define SCORE_PER_ENEMIE		40		// 10 punten per enemie die je doodschiet.

// Bullet
#define BULLET_BREEDTE 			4
#define BULLET_LENGTE 			9
#define MAX_SPRITE_BULLET_COUNT 4

// Game end durations
#define game_over_flicker_duration 	11
#define game_won_flicker_duration 	10


//----- Bram Laurens hieronder -----//
#define ADC_BUFFER_SIZE			200




//----- Finn van Zijverden -----//
typedef struct {
	uint16_t X_pos; 				//locatie van de kogel op de X-as
	uint16_t Y_pos; 				//locatie van de kogel op de Y-as
	uint8_t richting; 				//Richting van de kogel					0 is omhoog, 1 is omlaag
	bool actief; 					//Status van de kogel
	uint8_t object_ID; 				//ID van de kogel
} bullet_struct;

//----- Einde Finn van Zijverden -----//



//-------------------- Kim Lobstein --------------------//
typedef struct {
	uint8_t id;
	uint16_t X_pos;
	uint16_t Y_pos;
	uint8_t alive;
} sprite_struct;

//-------------------- Einde Kim Lobstein --------------------//



//-------------------- Fabian Meijneken --------------------//
typedef struct
{
    int8_t obj_ID;
    int16_t X_pos, Y_pos, speed;
    int8_t active_bullet_count;
    int8_t lives;
    int16_t score;
}player_struct;


//-------------------- Einde Fabian Meijneken --------------------//






//-------------------- Function prototypes --------------------//

//----- Finn van Zijverden -----//
void BulletBeheer(bullet_struct* bullets, char actie, int bulletID, bool shot_by_user, player_struct* player, sprite_struct* sprite, int sprite_num);
uint8_t ID_checker(bullet_struct* bullets);
void bulletUpdater(bullet_struct* bullets, player_struct* player);
//----- Einde Finn van Zijverden -----//


//----- Kim Lobstein -----//
void move_sprites(sprite_struct* sprites);
void collision_check_all_bullets(sprite_struct* sprites, player_struct* player, bullet_struct* bullets);
//----- Einde Kim Lobstein -----//


//----- Fabian Meijneken -----//
void player_move(player_struct* player, char move_left);
void player_shoot(player_struct* player, bullet_struct* bullets, sprite_struct* sprite);
void sprite_shoot(player_struct* player, bullet_struct* bullets, sprite_struct* sprite, int sprite_num);
int collision_per_bullet(sprite_struct* sprites, player_struct* player, bullet_struct* bullets, uint8_t bulletIndex);				//BB 0 voor omhoog, 1 voor omlaag

//----- Einde Fabian Meijneken -----//


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

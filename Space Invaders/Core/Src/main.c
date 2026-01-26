/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  * @author 		: Finn van Zijverden, Kim Lobstein, Fabian Meijneken
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "UART_handler.h"
#include "dft.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
// Game variables
bool update_tick_20hz = false;				// Globaal gedefinieerd omdat deze ook in de it.c wordt gebruikt.
enum gamestates game_status = GAME_STARTING;
const bullet_struct bullet_empty = {
	.X_pos = 0,
	.Y_pos = 0,
	.actief = 0,
	.object_ID = 0,
	.richting = 0
};

// Sprite move variables
uint8_t links_rechts = 1; 					// Deze variabelle geeft aan of de sprites naar links of rechts bewegen. (0 voor links, 1 voor rechts)

// UART variables
uint32_t tx_buffer[UART2_TX_BUFFER_SIZE];
uint8_t uart2_DMA_index, uart2_upload_index = 0;
bool UART2_busy;


// DFT variables
uint32_t current_ADC_value;
int ADC_buffer[ADC_BUFFER_SIZE];
bool buffer_full = false;
bool new_ADC_value_recieved = false;
int ADC_index = 0;

int command_freqs[] = {500, 1000, 1500, 2000}; 	// TODO: Deze frequenties aanpassen.



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

//----- Finn van Zijverden -----//
void BulletBeheer(bullet_struct* bullets, char actie, int bulletID, bool shot_by_user, player_struct* player, sprite_struct* sprite);
uint8_t ID_checker(bullet_struct* bullets);
void bulletUpdater(bullet_struct* bullets);
//----- Einde Finn van Zijverden -----//


//----- Kim Lobstein -----//
void move_sprites(sprite_struct* sprites);
void collision_per_bullet(sprite_struct* sprites, player_struct* player, bullet_struct* bullets, uint8_t bulletIndex, uint8_t BB);
void collision_check_all_bullets(sprite_struct* sprites, player_struct* player, bullet_struct* bullets);
//----- Einde Kim Lobstein -----//


//----- Fabian Meijneken -----//
void player_move(player_struct* player, char move_left);
void player_shoot(player_struct* player, bullet_struct* bullets, sprite_struct* sprite);
void command_handler(uint8_t command, player_struct* player, bullet_struct* bullets, sprite_struct* sprite);
//----- Einde Fabian Meijneken -----//


//----- Bram Laurens -----//
void run_dft();
//----- Einde Bram Laurens -----//

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	// Klokdeler init:
	char klok_10_hz_counter = 1;
	bool klok_10_hz = false;

	// Player init:
	player_struct player;

	// Sprite init:
	sprite_struct sprites[SPRITES_PER_RIJ * AANTAL_RIJEN_SPRITES];

	// Bulet init:
	bullet_struct bullets[MAX_BULLET_AMOUNT];


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim3);		// Gebruikt voor DFT.
  HAL_TIM_Base_Start_IT(&htim4);		// Gebruikt voor game updates.
  HAL_ADC_Start_DMA(&hadc1, &current_ADC_value, 1); // Link the ADC DMA to current_ADC_value.

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		// Update het spel alleen als de game loopt.
		switch (game_status)
		{
			// Wordt gebruikt aan het begin van het spel, reset alle posities en waardes.
			case GAME_STARTING:
				// Reset clock
				klok_10_hz_counter = 1;
				klok_10_hz = false;

				// Reset player
				player.obj_ID =					0;
				player.X_pos = 					PLAYER_X_START;
				player.Y_pos = 					PLAYER_Y_START;
				player.speed = 					PLAYER_MOVE_SPEED;
				player.active_bullet_count = 	0;
				player.lives = 					3;
				player.score = 					0;

				// Reset Sprites
				for (uint8_t rij = 0; rij < (AANTAL_RIJEN_SPRITES); rij++)
				{
					for (uint8_t kolom = 0; kolom < (SPRITES_PER_RIJ); kolom ++)
					{
						sprites[(rij * SPRITES_PER_RIJ) + kolom].id = (rij * SPRITES_PER_RIJ) + kolom;
						sprites[(rij * SPRITES_PER_RIJ) + kolom].X_pos = SPRITES_X_OFFSET + (kolom * SPRITES_X_AFSTAND); //=offset + plek * distance
						sprites[(rij * SPRITES_PER_RIJ) + kolom].Y_pos = SPRITES_Y_OFFSET + (rij * SPRITES_Y_AFSTAND); //=offset + plek * distance
						sprites[(rij * SPRITES_PER_RIJ) + kolom].alive = 1;
					}

				}

				// Reset Bullets
				for (int i = 0; i < MAX_BULLET_AMOUNT - 1; i++)
				{
					bullets[i] = bullet_empty;
				}


				game_status = GAME_RUNNING;


			case GAME_RUNNING :
				// DFT, bullet en player update.
				if (update_tick_20hz)
				{
					HAL_GPIO_TogglePin(LED_oranje_GPIO_Port, LED_oranje_Pin);

					//----- DFT update -----//
					run_dft();

					//----- Bullet locatie updaten -----//

					// Update locatie
					bulletUpdater(bullets);
					// Check voor collision
					collision_check_all_bullets(sprites, &player, bullets);



					update_tick_20hz = false;

					if (klok_10_hz_counter++ == KLOKDELER_SPRITE_UPDATE)
					{
					  klok_10_hz_counter = 1;
					  klok_10_hz = true;
					}

				}

				// Sprite update
				if (klok_10_hz)
				{
					HAL_GPIO_TogglePin(LED_groen_GPIO_Port, LED_groen_Pin);

					//----- Enemy locatie updaten -----//
					move_sprites(sprites);
					// TODO: send UART message to move sprites.


					klok_10_hz = false;
				}
			case GAME_WON :
				// TODO: GAME_WON actie maken
			case GAME_OVER :
				// TODO: GAME_OVER actie maken
			case GAME_PAUSED :
				// TODO: GAME_PAUSED actie maken

		}


		if (!UART2_busy && !(uart2_DMA_index == uart2_upload_index))
		{
			// UART2 is vrij en er is een nieuw bericht, verstuur deze.

			UART2_busy = true;

			// Haal het te versturen commando op
			uint32_t current_command = tx_buffer[uart2_DMA_index];

			// Splits het commando (uint32_t) op in 4 uint8_t delen. Dit is nodig voor de UART transmision.
			uint8_t uart_message[4] = {0};

			uart_message[0] = (uint8_t) current_command;
			uart_message[1] = (uint8_t) (current_command >> 8);
			uart_message[2] = (uint8_t) (current_command >> 16);
			uart_message[3] = (uint8_t) (current_command >> 24);

			HAL_UART_Transmit_DMA(&huart2, uart_message, 4);
		}


		// Add new ADC value to the circular ADC buffer
		if (new_ADC_value_recieved)
		{
			if(ADC_index < ADC_BUFFER_SIZE)
			  {
			      ADC_buffer[ADC_index] = current_ADC_value;
			      ADC_index++;

			      if(ADC_index >= ADC_BUFFER_SIZE)
			      {
			          buffer_full = true;
			          ADC_index = 0;
			      }
			  }
			  else
			  {
			    // Fallback
				  ADC_index = 0;
			  }

			new_ADC_value_recieved = false;
		}


		if (uart2_upload_index == uart2_DMA_index - 1)
		{
			game_status = GAME_OVER;
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_groen_Pin|LED_oranje_Pin|LED_rood_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_groen_Pin LED_oranje_Pin LED_rood_Pin */
  GPIO_InitStruct.Pin = LED_groen_Pin|LED_oranje_Pin|LED_rood_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */





//-------------------- Finn van Zijverden --------------------//

// Functie voor het beheren van de 'bullers', functie heeft twee args
// Deze functie returned een uint8_t, het ID van de bullet die zojuist is aangemaakt.

// 'char actie', beheer wat er moet gebeuren:
// actie 1, maak nieuwe bullet
// actie 2, verwijder bullet


// Als actie 2 is, bepaald bulletID welke bullet [0 ... MAX_BULLETS-1] wordt verwijderd. Als de actie 1 is, wordt deze variabel niet gebruikt.
// Als actie 2 wordt spriteID niet gebruikt
// Als actie 1 is, en shot_by_user is 1. Dan schiet de speler 1 bullet, en wordt spriteID niet gebruikt. Als shot_by_user 0 is, dan wordt de bullet vanaf sprite[spriteID] geschoten.


// de pointer naar de sprite point naar de sprite die schiet, dus NIET altijd naar de eerste waarde van de array.
void BulletBeheer(bullet_struct* bullets, char actie, int bulletIndex, bool shot_by_user, player_struct* player, sprite_struct* sprite)
{
	uint8_t currentBullet = 0;

	switch (actie)
	{
	case 1:
		if (shot_by_user)
		{
			// Vragen naar een ID van een bullet die niet in gebruik is
			currentBullet = ID_checker(bullets);

			// De ID-checker geeft 0xFF als er geen bullet vrij is
			if (currentBullet == 0xFF)
				return;

			// Variabelen schrijven naar de vrije bullet index
			(bullets + currentBullet) -> richting = 0;
			(bullets + currentBullet) -> X_pos = player->X_pos;
			(bullets + currentBullet) -> Y_pos = player->Y_pos;

			// Aangeven dat de bullet in gebruik is
			(bullets + currentBullet) -> actief = 1;

			// Er wordt hier geen update naar de FPGA gestuurd, omdat dat door de bulletUpdater wordt gedaan.
		}

		else
		{
			currentBullet = ID_checker(bullets);

			// De ID-checker geeft 0xFF als er geen bullet vrij is
			if (currentBullet == 0xFF)
				return;

			(bullets + currentBullet) -> richting = 1;
			(bullets + currentBullet) -> X_pos = sprite -> X_pos;
			(bullets + currentBullet) -> Y_pos = sprite -> Y_pos;
			(bullets + currentBullet) -> actief = 1;

			// Er wordt hier geen update naar de FPGA gestuurd, omdat dat door de bulletUpdater wordt gedaan.
		}

	case 2:

		(bullets + bulletIndex)->actief = 0;
		// Er wordt hier geen update naar de FPGA gestuurd, omdat dat door de bulletUpdater wordt gedaan.

		break;

	default:
		return;


	} // einde van switch case voor actie selecteren
}


// Functie voor het geven van de eerste bullet die niet actief is
// Deze functie heeft geen argumenten
// Functie returned een ID van een vrije bullet, in de vorm van een uint8_t
uint8_t ID_checker(bullet_struct* bullets)
{
	for (int FreeIDchecker = 0; FreeIDchecker < MAX_BULLET_AMOUNT; FreeIDchecker++)
	{
		if ((bullets + FreeIDchecker)->actief == 0)
		{
			return FreeIDchecker;
			break;
		}
	}

	// Als er geen vrije bullets zijn, return 0xFF
	return 0xFF;
}

// Functie voor het updaten van de locaties van de bullets
// Deze update de locatie op basis van de richting van de kogel
void bulletUpdater(bullet_struct* bullets)
{
	for (uint8_t i = 0; i < MAX_BULLET_AMOUNT; i++)
	{
		if ((bullets + i)->actief)
		{
			if ((bullets + i)->richting)
				(bullets + i)->X_pos += BULLET_MOVE_SPEED;
			else
				(bullets + i)->X_pos -= BULLET_MOVE_SPEED;
		}

		update_FPGA((bullets + i)->object_ID, (bullets + i)->X_pos, (bullets + i)->Y_pos, 0b1);
	}
}

//-------------------- Einde Finn van Zijverden --------------------//





//-------------------- Kim Lobstein --------------------//

void move_sprites(sprite_struct* sprites){
	// TODO: herschrijven zodat ook de bovenste enemies als eerste dood kunnen gaan.
	// Alleen de bovenste rij wordt gechecked of ze aan de zijkant zijn.

	// Nu is het zo dat de check of je de kant hebt geraakt wordt gedaan met ALLEEN de bovenste sprites.

	// Check of je de kant al hebt geraakt.
	for (uint8_t i = 0; i < SPRITES_PER_RIJ; i++){
		if ((((sprites + i)->X_pos >= MAX_X_SPRITES)||((sprites + i)->X_pos <= MIN_X_SPRITES)) && ((sprites + i)->alive == 1))			//als een levende sprite de rechter rand raakt:
		{
			links_rechts = !links_rechts;																								//draai dan de beweegrichting om

			for (uint8_t k = 0; k < (SPRITES_PER_RIJ * AANTAL_RIJEN_SPRITES); k++){														//en beweeg de sprites naar beneden
				(sprites + k)->Y_pos = (sprites + k)->Y_pos + SPRITE_Y_MOVE_SPEED;														//sprites x pixels naar beneden

				if ((sprites + k)->alive
					&& (sprites + k)->Y_pos >= MAX_SPRITE_Y)
				{
					game_status = GAME_OVER;
					return;
					// Game over
				}
			}
		}
	}

	//beweeg vervolgens alle sprites in de ingestelde richting
	for (uint8_t j = 0; j < (SPRITES_PER_RIJ * AANTAL_RIJEN_SPRITES); j++){
		(sprites + j)->X_pos += links_rechts ? SPRITE_X_MOVE_SPEED : -SPRITE_X_MOVE_SPEED;				// Ternary operator ;)
	}


//	if (links_rechts){
//		for (uint8_t j = 0; j < (SPRITES_PER_RIJ * AANTAL_RIJEN_SPRITES); j++){
//			(sprites + j)->X_pos += SPRITE_MOVE_SPEED;											//sprites x pixels naar rechts
//
//			(sprites + j)->X_pos += links_rechts ? SPRITE_MOVE_SPEED : -SPRITE_MOVE_SPEED;
//		}
//	} else {
//		for (uint8_t m = 0; m < (SPRITES_PER_RIJ * AANTAL_RIJEN_SPRITES); m++){
//			(sprites + m)->X_pos -= SPRITE_MOVE_SPEED; 										//sprites x pixels naar links
//		}
//	}
}


void collision_per_bullet(sprite_struct* sprites, player_struct* player, bullet_struct* bullets, uint8_t bulletIndex, uint8_t BB)				//BB 0 voor omhoog, 1 voor omlaag
{
	//als de kogel omlaag beweegt:
	if (BB) {
		for (uint8_t i = 0; i < BULLET_BREEDTE; i++){ 		//kijk voor elke pixel in de breedte van de kogel of:
			if ((bullets + bulletIndex)->X_pos + i >= player->X_pos && (bullets + bulletIndex)->X_pos + i <= player->X_pos + SPELER_BREEDTE){  		//of de x positie binnen de x positie van de speler valt.
				if ((bullets + bulletIndex)->Y_pos + BULLET_LENGTE == player->Y_pos){												//en of de y positie vervolgens ook overeenkomt

					//De speler verliest een leven
					player->lives -= 1;

					// Verwijder betreffende bullet
					BulletBeheer(bullets, 2, bulletIndex, false, player, sprites);						// In deze call maken alleen de argumenten "bullets", "actie" (2) en "bulletIndex", uit. player en sprites worden bij actie=2 genegeerd.

				}

			}
		}
	}

	//als de kogel omhoog beweegt:
	else
	{
		for (uint8_t j = 0; j < BULLET_BREEDTE; j++)															// kijk voor elke pixel in de breedte van de kogel of:
		{
			for (uint8_t k = 0; k < SPRITES_PER_RIJ; k++)														// kijk voor alle sprites in de bovenste rij of:
			{
				if ((bullets + bulletIndex)->X_pos + j >= (sprites + k)->X_pos													// of de x posities van de kogel overeenkomen met een die van een sprite
					&& (bullets + bulletIndex)->X_pos + j <= (sprites + k)->X_pos + SPRITE_BREEDTE)
				{
					for (uint8_t m = 0; m < AANTAL_RIJEN_SPRITES; m++)											// kijk of voor 1 van de sprites in de kolom k:
					{
						if (((bullets + bulletIndex)->Y_pos == (sprites + (k + SPRITES_PER_RIJ * m))->Y_pos + SPRITE_LENGTE)		// of de sprite nog leeft en de y posities van de kogel en sprite overeenkomen
							&& (sprites + (k + SPRITES_PER_RIJ * m))->alive == 1)
						{
							//als dat allemaal zo is; dan is er een collision met deze sprite. Deze gaat dan dood. daarna gaat de kogel ook dood
							(sprites + (k + SPRITES_PER_RIJ * m))->alive = 0;

							// Update score
							player->score += SCORE_PER_ENEMIE;

							// Verwijder betreffende bullet
							BulletBeheer(bullets, 2, bulletIndex, true, player, sprites);		// In deze call maken alleen de argumenten "bullets", "actie" (2) en "bulletIndex", uit. player en sprites worden bij actie=2 genegeerd.
						}
					}
				}
			}
		}
	}
}


// Functie die voor elke bullet een collision check uitvoert.
void collision_check_all_bullets(sprite_struct* sprites, player_struct* player, bullet_struct* bullets)
{
	for (int i = 0; i < (MAX_BULLET_AMOUNT - 1); i++)
	{
		collision_per_bullet(sprites, player, bullets, i, (bullets + i)->richting);
	}
}

//-------------------- Einde Kim Lobstein --------------------//





//-------------------- Fabian Meijneken --------------------//

// Deze functie veranderd de speler x positie (opgeslagen in een struct).
// Of de speler naar links of rechts beweegt, is aangegeven door move_left. Dit is in deze definitie constant gemaakt.
// Deze functie returned een UART message in binair formaat.
// Gemaakt door Fabian Meijneken - 09/01/2026
void player_move(player_struct* player, char move_left)
{
    // Controleer of we naar links of rechts moeten bewegen, en verander player.x hiernaar.
    if (move_left == 1)
    {
        player->X_pos = (int16_t) (player->X_pos - player->speed);
        // Ik gebruik hier een cast om warnings te voorkomen. Beide x en speed zijn van het type int16_t.
        // Deze 16 bit integers worden voor de berekening beide omgezet naar een volledige integer (32 bit).
        // Als de berekening klaar is en ze gaan terug naar en int16_t dan "kan" je data verliezen, ook al is dat niet zo.
        // Om de compiler gerust te stellen gebruik ik dus een cast.

        // Note: dit probleem had ook voorkomen kunnen worden door gewoon integers te gebruiken,
        // maar ik zie dit als een mooie oefening tot het besparen van geheugen.

    } else
    {
        player->X_pos = (int16_t) (player->X_pos + player->speed);
    }

    // Versstuur een uart_bericht als "update".
    update_FPGA(player->obj_ID, player->X_pos, player->Y_pos, 0b1);
}


// Deze functie laat de speler, als dat kan, een kogel schieten.
// Als de speler boven een maximaal aantal kogels zit, kan deze niet schieten. Als dat wel kan, roep schiet functie op.
// Deze functie is void, aangezien de functie over de bullets updates verstuurd.
// Gemaakt door Fabian Meijneken - 09/01/2026
void player_shoot(player_struct* player, bullet_struct* bullets, sprite_struct* sprite)
{
    // Controleer of we naar links of rechts moeten bewegen, en verander player.x hiernaar.
    if (player->active_bullet_count < MAX_PLAYER_BULLET_COUNT)
    {
    	BulletBeheer(bullets, 1, 1, true, player, sprite);

        player-> active_bullet_count++;                 // Update de hoeveelheid kogels die de player momenteel heeft.
    }
}


// Deze functie bepaalt een handeling op basis van een frequentie.
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

		// Beweeg naar links
		case 1:
			player_move(player, 1);

		// Beweeg naar rechts
		case 2:
			player_move(player, 0);

		// Schiet
		case 3:
			player_shoot(player, bullets, sprite);

	}
}


//-------------------- Einde Fabian Meijneken --------------------//



//-------------------- Bram Laurens --------------------//

void run_dft(player_struct* player, bullet_struct* bullets, sprite_struct* sprites)
{
	for (uint8_t command = 0; command < (sizeof(command_freqs) - 1) ; command++)
	{
		float magnitude = DFT_compute_LUT(command_freqs[command]);

		if (magnitude >= COMMAND_THRESHOLD)
			command_handler(command, player, bullets, sprites);
	}
}

//-------------------- Einde Bram Laurens --------------------//


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	HAL_GPIO_TogglePin(LED_rood_GPIO_Port, LED_rood_Pin);
	new_ADC_value_recieved = true;
}


//-------------------- UART callback --------------------//
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uart2_DMA_index++;

	// Zorg dat de uart2_DMA_index niet buiten de buffer staat.
	if (uart2_DMA_index > (UART2_TX_BUFFER_SIZE - 1))
		uart2_DMA_index = 0;

	UART2_busy = false;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

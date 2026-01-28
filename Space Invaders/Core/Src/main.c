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
  * @author 		: Finn van Zijverden, Kim Lobstein, Fabian Meijneken, Bram Laurens
  ******************************************************************************
  */

// TODO At collision, send command for only that row. This saves uart size, maby that removes the stutter on colision?
// TODO fix sprites moving too far to the left
// TODO fix player being able to go offscreen on the right.
// TODO health takes 2 per hit.

// TODO low prio, UART can lose position when STM32 resets during UART command.


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
volatile bool update_tick_20hz = false;				// Globaal gedefinieerd omdat deze ook in de it.c wordt gebruikt.
enum gamestates game_status = GAME_RESET;
const bullet_struct bullet_empty = {
	.X_pos = 10,
	.Y_pos = 10,
	.actief = 0,
	.object_ID = 14,						// Dit moet een ongebruikt object_ID zijn, anders zal tijdens het renderen dit object ID steeds gaan "flikkeren"
	.richting = 0
};

uint8_t game_done_flicker_count = 0;
volatile bool button_command_override = false;


// Sprite variables
uint8_t links_rechts = 1; 													// Deze variable geeft aan of de sprites naar links of rechts bewegen. (0 voor links, 1 voor rechts)
int SPRITES_MOVE_FREQ = DEF_SPRITES_MOVE_FREQ;								// Deze deelt een 20 Hz klok. met een waar van 10 bewegen de sprites op 2 Hz.

int aantal_levende_sprites = SPRITES_PER_RIJ * AANTAL_RIJEN_SPRITES;		// Deze variable houdt de hoeveelheid levende sprites bij.

// UART variables
uint32_t tx_buffer[UART2_TX_BUFFER_SIZE];
uint8_t uart2_DMA_index, uart2_upload_index = 0;
bool UART2_busy;
uint8_t uart2_DMA_tx_bytes[4] = {0};


// DFT variables
// ADC DMA will continuously fill this block buffer; we copy into ADC_buffer in the DMA callbacks.
#define ADC_DMA_BLOCK_SAMPLES 64
static uint16_t adc_dma_block[ADC_DMA_BLOCK_SAMPLES];
static volatile bool adc_dma_block_ready = false; // Set true when a full 64-sample block has been captured.

int ADC_buffer[ADC_BUFFER_SIZE];
volatile bool buffer_full = false;
volatile uint32_t ADC_index = 0;



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
	char sprite_move_clock_counter = 1;
	bool sprite_move_clock = false;

	char game_done_clock_counter = 1;
	bool game_done_clock = false;

	// Player init:
	player_struct player;
	player.score = 0;

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
	// Capture ADC samples in blocks of 64; HAL will call Conv(Half)Cplt callbacks via DMA.
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_block, ADC_DMA_BLOCK_SAMPLES);

	init_LUTs();						// Initialiseer de LUTs voor de DFT.
	init_buttons();						// initialize the buttons for command override.

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		// Debug: only run/print when we have a full ADC buffer and UART2 is idle.
		// NOTE: USART2 is also used for DMA TX to the FPGA; mixing DMA + blocking TX can result in HAL_BUSY.
		

		#ifdef OUTPUT_DFT_DEBUG
		int msg_length;
		char msg[64];
		float peak;
		
		peak = DFT_range_peak(600.0f, 1100.0f);
		msg_length = snprintf(msg, sizeof(msg), "DFT Peak 600Hz-1.1kHz: %.2f	", peak);
		(void)HAL_UART_Transmit(&huart2, (uint8_t*)msg, (uint16_t)msg_length, 100);

		peak = DFT_range_peak(1100.0f, 1500.0f);
		msg_length = snprintf(msg, sizeof(msg), "DFT Peak 1.1kHz-1.5kHz: %.2f	", peak);
		(void)HAL_UART_Transmit(&huart2, (uint8_t*)msg, (uint16_t)msg_length, 100);

		peak = DFT_range_peak(1500.0f, 2000.0f);
		msg_length = snprintf(msg, sizeof(msg), "DFT Peak 1.5kHz-2kHz: %.2f		", peak);
		(void)HAL_UART_Transmit(&huart2, (uint8_t*)msg, (uint16_t)msg_length, 100);

		peak = DFT_range_peak(4000.0f, 4500.0f);
		msg_length = snprintf(msg, sizeof(msg), "DFT Peak 4kHz-4.5kHz: %.2f	\r\n", peak);
		(void)HAL_UART_Transmit(&huart2, (uint8_t*)msg, (uint16_t)msg_length, 100);
		#endif

		// Update het spel alleen als de game loopt.
		switch (game_status)
		{
			// Wordt gebruikt aan het begin van het spel, reset alle posities en waardes.
			case GAME_RESET:
				//----- Reset clock -----//
				sprite_move_clock_counter = 1;
				sprite_move_clock = false;

				//----- Reset sprite  variables -----//
				SPRITES_MOVE_FREQ = DEF_SPRITES_MOVE_FREQ;
				aantal_levende_sprites = SPRITES_PER_RIJ * AANTAL_RIJEN_SPRITES;


				//----- Reset player -----//
				player.obj_ID =					0;
				player.X_pos = 					PLAYER_X_START;
				player.Y_pos = 					PLAYER_Y_START;
				player.speed = 					PLAYER_MOVE_SPEED;
				player.active_bullet_count = 	0;
				player.lives = 					3;

				//----- Reset Sprites -----//
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

				//----- Reset Bullets -----//
				for (int i = 0; i < MAX_BULLET_AMOUNT; i++)
				{
					bullets[i] = bullet_empty;
				}


				//----- Verstuur kogel locaties -----//
				for (int i = 0; i < MAX_BULLET_AMOUNT; i++)
				{
					update_FPGA((6 + i),
								(bullets + i)->X_pos,
								(bullets + i)->Y_pos,
								((bullets + i)->richting << 1) | (bullets + i)->actief		// LSB is nu "richting", bitje links daarvan is "actief"
								);
				}

				//----- Verstuur sprite locaties -----//
				// Dit loopt vanaf 1 aangezien de player ID 1 heeft.
				for (int rij = 1; rij <= AANTAL_RIJEN_SPRITES; rij++)
				{
					uint8_t render_bits = 0b0;
					for (int i = 0; i < SPRITES_PER_RIJ; i++)
						render_bits |= ((sprites + ((rij-1) * 6) + i) -> alive) << (SPRITES_PER_RIJ - 1 - i);

					update_FPGA(rij, (sprites + ((rij-1) * 6))->X_pos, (sprites + ((rij-1) * 6))->Y_pos, render_bits);
				}


				game_status = GAME_STARTING;
				break;


			// Wordt gebruikt om na de GAME_RESET alle UART commando's weg te werken. Pas als alles is verstuurd gaan we door.
			case GAME_STARTING :
				if (uart2_DMA_index == uart2_upload_index)
					game_status = GAME_RUNNING;

				break;


			// Loopt tijdens het spel
			case GAME_RUNNING :
				// DFT, bullet en player update.
				if (update_tick_20hz)
				{
					HAL_GPIO_TogglePin(LED_oranje_GPIO_Port, LED_oranje_Pin);

					//----- DFT update -----//
					run_dft(&player, &bullets, &sprites);

					// Check voor collision
					collision_check_all_bullets(sprites, &player, bullets);

					// Update bullet locatie
					bulletUpdater(bullets, &player);


					// Verstuur kogel locaties
					for (int i = 0; i < MAX_BULLET_AMOUNT; i++)
					{
						update_FPGA((bullets + i)->object_ID,
									(bullets + i)->X_pos,
									(bullets + i)->Y_pos,
									((bullets + i)->richting << 1) | (bullets + i)->actief		// LSB is nu "richting", bitje links daarvan is "actief"
									);	
					}

					// Verstuur speler locatie
					update_FPGA(player.obj_ID, player.X_pos, player.Y_pos, (((player.lives & 0b11) << 1) | 0b1));

					// Verstuur de score
					// In de FPGA worden de bitjes van Y achter de bitjes van X geplakt om zo een 19 bits getal te genereren.
					// player.score is een uint16_t
					update_FPGA(SYSTEM_OBJ_ID, (player.score >> 9), (player.score), 0b0);


					// Genereer de sprite_move klok. Dit wordt gedaan met een instelbare frequentie.
					if (sprite_move_clock_counter++ >= (SPRITES_MOVE_FREQ / 3))
					{
					  sprite_move_clock_counter = 1;
					  sprite_move_clock = true;
					}

					update_tick_20hz = false;
				}

				// Sprite update
				if (sprite_move_clock)
				{
					HAL_GPIO_TogglePin(LED_groen_GPIO_Port, LED_groen_Pin);

					// Update sprite locatie
					move_sprites(sprites);

					// Verstuur sprite locaties
					// Dit loopt vanaf 1 aangezien de player ID 1 heeft.
					for (int rij = 1; rij <= AANTAL_RIJEN_SPRITES; rij++)
					{
						uint8_t render_bits = 0b0;
						for (int i = 0; i < SPRITES_PER_RIJ; i++)
							render_bits |= ((sprites + ((rij-1) * 6) + i) -> alive) << (SPRITES_PER_RIJ - 1 - i);

						update_FPGA(rij, (sprites + ((rij-1) * 6))->X_pos, (sprites + ((rij-1) * 6))->Y_pos, render_bits);
					}

					// Zet de "random seed" naar de runtime in miliseconds, deze is max 32 bits.
					// Dit geeft een meer random waarde.
					srand(HAL_GetTick());

					// Laat een random enemy een bullet schieten
					if ((rand() % BULLET_FREQUENCY) == 1)
					{
						
						// Maak een array met alle levende sprites daarin.
						int alive_sprites[aantal_levende_sprites];
						uint8_t count = 0;

						for (int i = 0; i < (AANTAL_RIJEN_SPRITES * SPRITES_PER_RIJ); i++)
						{
							if (sprites[i].alive)
								alive_sprites[count++] = sprites[i].id;
						}

						int random_sprite = alive_sprites[(rand() % aantal_levende_sprites)];

						// Schiet een bullet vanaf een random levende sprite.
						sprite_shoot(&player,
									bullets,
									sprites,
									random_sprite
									);
					}



					sprite_move_clock = false;
				}

				break;


			// Wordt gebruikt als de speler heeft gewonnen.
			case GAME_WON :
				if (update_tick_20hz)
				{
					if (game_done_clock_counter++ >= 10)
					{
						game_done_clock_counter = 1;
						game_done_clock = true;
					}

					update_tick_20hz = false;
				}

				if (game_done_clock)
				{
					// Flicker de game_won bit (render bit 0b100)
					if (game_done_flicker_count++ % 2)
						update_FPGA(SYSTEM_OBJ_ID, (player.score >> 9), (player.score), 0b100);
					else
						update_FPGA(SYSTEM_OBJ_ID, (player.score >> 9), (player.score), 0b0);

					if (game_done_flicker_count >= game_won_flicker_duration)
					{
						HAL_Delay(1000);
						game_done_flicker_count = 0;
						game_status = GAME_RESET;
					}

					game_done_clock = false;
				}
				break;

			case GAME_OVER :
				if (update_tick_20hz)
				{
					if (game_done_clock_counter++ >= 10)
					{
						game_done_clock_counter = 1;
						game_done_clock = true;
					}

					update_tick_20hz = false;
				}

				if (game_done_clock)
				{
					player.score = 0;

					// Flicker de game_over bit (render bit 0b10)
					if (game_done_flicker_count++ % 2)
						update_FPGA(SYSTEM_OBJ_ID, (player.score >> 9), (player.score), 0b10);
					else
						update_FPGA(SYSTEM_OBJ_ID, (player.score >> 9), (player.score), 0b0);

					if (game_done_flicker_count >= game_over_flicker_duration)
					{
						HAL_Delay(1000);
						game_done_flicker_count = 0;
						game_status = GAME_RESET;
					}

					game_done_clock = false;
				}
				break;

			case GAME_PAUSED :
				// TODO: GAME_PAUSED actie maken
				break;

		}


		if (!UART2_busy && !(uart2_DMA_index == uart2_upload_index))
		{
			// UART2 is vrij en er is een nieuw bericht, verstuur deze.

			UART2_busy = true;

			// Haal het te versturen commando op
			uint32_t current_command = tx_buffer[uart2_DMA_index];

			// Splits het commando (uint32_t) op in 4 uint8_t delen. Dit is nodig voor de UART transmision.

			uart2_DMA_tx_bytes[0] = (uint8_t) current_command;
			uart2_DMA_tx_bytes[1] = (uint8_t) (current_command >> 8);
			uart2_DMA_tx_bytes[2] = (uint8_t) (current_command >> 16);
			uart2_DMA_tx_bytes[3] = (uint8_t) (current_command >> 24);

			HAL_UART_Transmit_DMA(&huart2, uart2_DMA_tx_bytes, 4);
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, buttons_row_0_Pin|LED_groen_Pin|LED_oranje_Pin|LED_rood_Pin
                          |buttons_row_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : buttons_interrupt_Pin */
  GPIO_InitStruct.Pin = buttons_interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(buttons_interrupt_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : buttons_col_2_Pin buttons_col_0_Pin buttons_col_1_Pin */
  GPIO_InitStruct.Pin = buttons_col_2_Pin|buttons_col_0_Pin|buttons_col_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : buttons_row_0_Pin LED_groen_Pin LED_oranje_Pin LED_rood_Pin
                           buttons_row_1_Pin */
  GPIO_InitStruct.Pin = buttons_row_0_Pin|LED_groen_Pin|LED_oranje_Pin|LED_rood_Pin
                          |buttons_row_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
// Als actie 1 is, en shot_by_user is 1. Dan schiet de speler 1 bullet, en wordt sprite_num niet gebruikt. Als shot_by_user 0 is, dan wordt de bullet vanaf sprite[sprite_num] geschoten.


// de pointer naar de sprite point naar de sprite die schiet, dus NIET altijd naar de eerste waarde van de array.
void BulletBeheer(bullet_struct* bullets, char actie, int bulletIndex, bool shot_by_user, player_struct* player, sprite_struct* sprite, int sprite_num)
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
			(bullets + currentBullet) -> object_ID = 6 + currentBullet;

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

			// Variabelen schrijven naar de vrije bullet index
			(bullets + currentBullet) -> richting = 1;
			(bullets + currentBullet) -> X_pos = (sprite + sprite_num) -> X_pos;
			(bullets + currentBullet) -> Y_pos = (sprite + sprite_num) -> Y_pos;
			(bullets + currentBullet) -> object_ID = 6 + currentBullet;

			// Aangeven dat de bullet in gebruik is
			(bullets + currentBullet) -> actief = 1;

			// Er wordt hier geen update naar de FPGA gestuurd, omdat dat door de bulletUpdater wordt gedaan.
		}

		break;

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
void bulletUpdater(bullet_struct* bullets, player_struct* player)
{
	for (uint8_t i = 0; i < MAX_BULLET_AMOUNT; i++)
	{
		if ((bullets + i)->actief)
		{

			// Check of de bullet moet worden verwijderd
			if (((bullets + i)->Y_pos <= BULLET_Y_MIN)
					| ((bullets + i)->Y_pos >= BULLET_Y_MAX))
			{
				// Als de bullet is geschoten door de speler, verlaag de
				if ((bullets + i)->richting == 0)
					player->active_bullet_count--;

				(bullets + i)->actief = false;
				continue;
			}


			// Beweeg de kogel in de juiste richting
			if ((bullets + i)->richting)
				(bullets + i)->Y_pos += BULLET_MOVE_SPEED;
			else
				(bullets + i)->Y_pos -= BULLET_MOVE_SPEED;
		}
	}
}

//-------------------- Einde Finn van Zijverden --------------------//





//-------------------- Kim Lobstein --------------------//

void move_sprites(sprite_struct* sprites){
	bool sprite_raakt_rand = false;

	for (uint8_t i = 0; i < AANTAL_RIJEN_SPRITES * SPRITES_PER_RIJ; i++){
		if (
			(((sprites + i)->X_pos >= MAX_X_SPRITES)
			|| (((sprites + i)->X_pos + SPRITE_BREEDTE) <= MIN_X_SPRITES))
			&& ((sprites + i)->alive == 1)
			)
		{
			sprite_raakt_rand = true;
			break;
		}
	}

	if (sprite_raakt_rand)
	{
		links_rechts = !links_rechts;

		for (uint8_t k = 0; k <  AANTAL_RIJEN_SPRITES * SPRITES_PER_RIJ; k++)
		{
			if ((sprites + k)->alive
				&& (sprites + k)->Y_pos + SPRITE_Y_MOVE_SPEED >= MAX_Y_SPRITES)
			{
				// Game over
				game_status = GAME_OVER;

				return;
			}
			else
			{
				(sprites + k)->Y_pos = (sprites + k)->Y_pos + SPRITE_Y_MOVE_SPEED;
				(sprites + k)->X_pos += links_rechts ? SPRITE_X_MOVE_SPEED : -SPRITE_X_MOVE_SPEED;				// Ternary operator ;)
			}
		}
	}
	else
	{
		for (uint8_t k = 0; k <  AANTAL_RIJEN_SPRITES * SPRITES_PER_RIJ; k++)
		{
				(sprites + k)->X_pos += links_rechts ? SPRITE_X_MOVE_SPEED : -SPRITE_X_MOVE_SPEED;				// Ternary operator ;)
		}
	}
}



// Functie die voor alle actieve bullets een collision check uitvoert.
// Deze functie stopt as
void collision_check_all_bullets(sprite_struct* sprites, player_struct* player, bullet_struct* bullets)
{
	for (int i = 0; i < (MAX_BULLET_AMOUNT - 1); i++)
	{
		if ((bullets + i)->actief == 1)
			if (collision_per_bullet(sprites, player, bullets, i))
				continue;
	}
}

//-------------------- Einde Kim Lobstein --------------------//





//-------------------- Fabian Meijneken --------------------//


/**
 * @brief Deze functie veranderd de speler x positie (opgeslagen in een struct).
 * 
 * @author Fabian meijneken
 * @date 09/01/2026
 *
 * @param player - pointer naar de player struct
 * @param move_left - char die aangeeft of de speler naar links (1) of rechts (0) moet bewegen
 * @return void
 */
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
}

/**
 * @brief Deze functie laat de speler, als dat kan, een kogel schieten.
 * 
 * Als de speler boven een maximaal aantal kogels zit, kan deze niet schieten. Als dat wel kan, roep schiet functie op.
 * Deze functie is void, aangezien de functie over de bullets updates verstuurd.
 * @author Fabian meijneken
 * @date 09/01/2026
 *
 * @param player - pointer naar de player struct
 * @param bullets - pointer naar het eerste item in de bullets array
 * @param sprite - pointer naar het eerste item in de sprite array
 *
 * @return void
 */
void player_shoot(player_struct* player, bullet_struct* bullets, sprite_struct* sprite)
{
    if (player->active_bullet_count < MAX_PLAYER_BULLET_COUNT)
    {
    	// Het bulletIndex en sprite_num moeten worden meegestuurd, maar er wordt niets mee gedaan. Vandaar dat deze hier respectievelijk 1 en 0 zijn.
    	BulletBeheer(bullets, 1, 1, true, player, sprite, 0);

        player-> active_bullet_count++;                 // Update de hoeveelheid kogels die de player momenteel heeft.
    }
}


/**
 * @brief Deze functie laat een enemy, als dat kan, een kogel schieten.
 * 
 * @author Fabian meijneken
 * @date 27/01/2026
 *
 * @param player - pointer naar de player struct
 * @param bullets - pointer naar het eerste item in de bullets array
 * @param sprite - pointer naar het eerste item in de sprite array
 *
 * @return void
 */
void sprite_shoot(player_struct* player, bullet_struct* bullets, sprite_struct* sprite, int sprite_num)
{
    // shot by user is hier false. Er zal vanaf sprite x (sprite_num) een bullet worden geschoten.
	BulletBeheer(bullets, 1, 1, false, player, sprite, sprite_num);

}


/**
 * @brief Deze functie checkt of er collision is tussen één bullet en de speler / sprites.
 *
 * @author Fabian meijneken
 * @date 27/01/2026
 *
 * @param sprites - pointer naar het eerste item in de sprite array
 * @param player  - pointer naar de player struct
 * @param bullets - pointer naar het eerste item in de bullets array
 * @param bulletIndex - Geeft aan over welke bullet het momenteel gaat
 *
 * @return int - 0 als er geen collision is, 1 als er collision is.
 */
int collision_per_bullet(sprite_struct* sprites, player_struct* player, bullet_struct* bullets, uint8_t bulletIndex)				//BB 0 voor omhoog, 1 voor omlaag
{

	// Check of de kogel de speler raakt. de x en y positie van de kogel zijn de x en y positie van de linkerbovenhoek van de kogel sprite .
	// Belangrijk is dat de kogel zich in een 32x32 sprite bevindt. Als de kogel naar beneden gaat, zit deze tegen de onderkant van zijn sprite aan.
	// Gaat hij omhoog, zit de kogel tegen de bovenkant van zijn sprite aan.
	// De kogel zal ALTIJD horizontaal in het midden van zijn sprite zitten.

	// Ook de speler zit gecentreerd in zijn sprite.

	// Belangrijke constanten die deze functie gebruikt:
	// - BULLET_BREEDTE: De breedte van de kogel sprite in pixels
	// - BULLET_LENGTE: De lengte van de kogel sprite in pixels
	// - SPELER_BREEDTE: De breedte van de speler sprite in pixels
	// - SPELER_LENGTE: De lengte van de speler sprite in pixels

	if ((bullets + bulletIndex)->richting)		// De kogel beweegt omlaag, check of de kogel een speler raakt.
	{
		if ( ((bullets + bulletIndex)->X_pos + 16 + (BULLET_BREEDTE / 2)) >= (player->X_pos + 16 - (SPELER_BREEDTE / 2))		// Rechterkant van kogel (x locatie) moet groter zijn dan linkerkant van speler (x locatie)
			&& (bullets + bulletIndex)->X_pos + 16 - (BULLET_BREEDTE / 2) <= player->X_pos + 16 + (SPELER_BREEDTE / 2)			// Linkerkant van de kogel (x locatie) moet kleiner zijn dan rechterkant van speler (x locatie)
			&& (bullets + bulletIndex)->Y_pos + BULLET_LENGTE >= player->Y_pos
			&& (bullets + bulletIndex)->Y_pos + BULLET_LENGTE <= player->Y_pos + SPELER_LENGTE		// Check of de kogel y binnen de speler y valt
		   )
		{
			// Verwijder betreffende bullet
			BulletBeheer(bullets, 2, bulletIndex, true, player, sprites, 0);		// In deze call maken alleen de argumenten "bullets", "actie" (2) en "bulletIndex", uit. player en sprites worden bij actie=2 genegeerd.

			if (--(player->lives) == 0)
				game_status = GAME_OVER;

			return 1;
		}
	}
	else		// De kogel beweegt omhoog, check of de kogel een sprite raakt.
	{
		for (int i = 0; i < (AANTAL_RIJEN_SPRITES * SPRITES_PER_RIJ); i++)
		{
			if ((sprites + i)->alive == 1)		// Alleen checken als de sprite leeft
			{
				if ( ((bullets + bulletIndex)->X_pos + 16 + (BULLET_BREEDTE / 2)) >= ((sprites + i)->X_pos + 16 - (SPRITE_BREEDTE / 2))		// Rechterkant van kogel (x locatie) moet groter zijn dan linkerkant van sprite (x locatie)
					&& (bullets + bulletIndex)->X_pos + 16 - (BULLET_BREEDTE / 2) <= ((sprites + i)->X_pos + 16 + (SPRITE_BREEDTE / 2))		// Linkerkant van de kogel (x locatie) moet kleiner zijn dan rechterkant van sprite (x locatie)
					&& (bullets + bulletIndex)->Y_pos >= ((sprites + i)->Y_pos)
					&& (bullets + bulletIndex)->Y_pos <= ((sprites + i)->Y_pos + SPRITE_LENGTE)
				   )
				{
					// Controleer of alle sprites dood zijn
					if (--aantal_levende_sprites == 0)
					{
						game_status = GAME_WON;
						return 1;
					}


					// Er is een collision, maak de sprite dood.
					(sprites + i)->alive = 0;

					// Update score
					player->score += SCORE_PER_ENEMIE;

					// Versnel het spel
					SPRITES_MOVE_FREQ -= 1;

					// Verwijder betreffende bullet
					BulletBeheer(bullets, 2, bulletIndex, true, player, sprites, 0);		// In deze call maken alleen de argumenten "bullets", "actie" (2) en "bulletIndex", uit. player en sprites worden bij actie=2 genegeerd.
					player->active_bullet_count--;


					// Verstuur sprite locaties
					for (int rij = 1; rij <= AANTAL_RIJEN_SPRITES; rij++)
					{
						uint8_t render_bits = 0b0;
						for (int i = 0; i < SPRITES_PER_RIJ; i++)
							render_bits |= ((sprites + ((rij-1) * 6) + i) -> alive) << (SPRITES_PER_RIJ - 1 - i);

						update_FPGA(rij, (sprites + ((rij-1) * 6))->X_pos, (sprites + ((rij-1) * 6))->Y_pos, render_bits);
					}

					// return 1, zodat er niet meerdere items kunnen doodgaan aan 1 bullet.
					return 1;
				}
			}
		}
	}

	return 0;
}


//-------------------- Einde Fabian Meijneken --------------------//


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance != ADC1)
		return;

	HAL_GPIO_TogglePin(LED_rood_GPIO_Port, LED_rood_Pin);

	// Copy second half of the DMA block into the circular buffer.
	for (uint32_t i = ADC_DMA_BLOCK_SAMPLES / 2; i < ADC_DMA_BLOCK_SAMPLES; i++)
	{
		ADC_buffer[ADC_index] = (int)adc_dma_block[i];
		ADC_index++;
		if (ADC_index >= ADC_BUFFER_SIZE)
		{
			ADC_index = 0;
			buffer_full = true;
		}
	}

	// Signal main loop that a full 64-sample block has been captured.
	adc_dma_block_ready = true;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance != ADC1)
		return;

	// Copy first half of the DMA block into the circular buffer.
	for (uint32_t i = 0; i < ADC_DMA_BLOCK_SAMPLES / 2; i++)
	{
		ADC_buffer[ADC_index] = (int)adc_dma_block[i];
		ADC_index++;
		if (ADC_index >= ADC_BUFFER_SIZE)
		{
			ADC_index = 0;
			buffer_full = true;
		}
	}
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

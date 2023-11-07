/* USER CODE BEGIN Header */

/* Gömülü Sistemler Proje Ödevi
 *
 *  Ekrem Tarık Yetgin 	  - 05190000434
 *	Muhammed Ahmet Kartop - 05180000506
 *  Labirent Tank Oyunu Projesi
*/


/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "time.h"
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define TANK_RADIUS 10							// Tankların Yarıçapı
#define MAX_BULLET_COUNT 5						// Her tankın haritada bulundurabileceği maksimum mermi sayısı
#define MAX_BULLET_LIFETIME 55					// Mermilerin haritada kalma süresi (birimi yok, oyunun ilerleme hızına bağlı)
#define BULLET_RADIUS 1							// Mermi yarıçapı
#define LCD_X_SIZE 320							// Ekran yatay uzunluk
#define LCD_Y_SIZE 240							// Ekran dikey uzunluk
#define MAP_BLOCKS 5							// Haritanın boyutu, daha büyük değer daha dar yollar demek. 240'ı tam bölen değerler verilmeli
#define WALL_COLOR LCD_COLOR_BLACK				// Duvar rengi
#define BULLET_COLOR LCD_COLOR_DARKRED			// Mermi rengi
#define TURNS_TO_WIN 9							// Oyunu kazanmak için gereken puan

typedef enum bool{
	false,true
}bool;

typedef struct bullet{
	int xPos;
	int yPos;
	short xVelocity;
	short yVelocity;
	uint8_t lifeTime;
}bullet;

typedef struct Tank{
	uint8_t ID;
	char name[11];
	uint16_t xPos;
	uint16_t yPos;

	uint8_t radius;

	short xPosBarrel;					// Namlu'nun uç pozisyonunun x bileşeni.
	short yPosBarrel;					// Namlu'nun uç pozisyonunun y bileşeni

	bool turnClockwise;
	bool turnCounterClockwise;
	bool goForward;
	bool goBack;
	bool fire;

	uint32_t color;

	GPIO_TypeDef* gpioPort;
	uint16_t pins[6];

	bullet bullets[MAX_BULLET_COUNT];

	uint8_t score;
}Tank;

char playerName[11];
char uartInputData;
char startMessage[100];
bool playerOneNameSet = false;
bool playerTwoNameSet = false;

int map[MAP_BLOCKS][MAP_BLOCKS];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

static void Baslangic_Ayarlari(void);

void ButtonInput(Tank *tank);														// Ayarlanmış butonlardan giriş alır
void DrawTank(Tank *tank);															// Tankı mevcut pozisyona çizdirir
void EraseTank(Tank *tank);															// Tankın mevcut pozisyonuna arka plan rengini bastırır
void ActionTank(Tank *tank,Tank *tankSecodary,int map[MAP_BLOCKS][MAP_BLOCKS]);		// Basılan butonlara göre tankın hareketlerini gerçekleştirir

void MoveBullets(Tank *tank,Tank *tankSecondary);									// Mermi hareketini sağlar
void EraseBullets(Tank *tank);														// Mermilerin konumuna arka plan rengini bastırır
void DrawBullets(Tank *tank);														// Mermileri mevcut pozisyonda çizdirir
void CheckForHit(Tank *tank1,Tank *tank2);											// Mermilerin tanka çarpıp çarpmadığını kontrol eder

void GenerateMaze(int map[MAP_BLOCKS][MAP_BLOCKS]);									// Haritayı oluşturur
int NeighbourCount(int neighbours[4]);												// Haritanın mezcut konumundan gidilebilecek komşu blok sayısını döndürür
void GetNeighbours(int map[MAP_BLOCKS][MAP_BLOCKS], int negihbours[4], int mapxIndex, int mapyIndex);
// Haritanın mezcut konumundan gidilebilecek komşu blokları belirler


void DrawMap(int map[MAP_BLOCKS][MAP_BLOCKS]);										// Haritayı çizdirir

bool IsWallRight(int x, int y,int map[MAP_BLOCKS][MAP_BLOCKS]);						// Mevcut bloğun sağ tarafında duvar olmalı mı
bool IsWallLeft(int x, int y,int map[MAP_BLOCKS][MAP_BLOCKS]);						// Mevcut bloğun sol tarafında duvar olmalı mı
bool IsWallBottom(int x, int y,int map[MAP_BLOCKS][MAP_BLOCKS]);					// Mevcut bloğun alt tarafında duvar olmalı mı
bool IsWallUp(int x, int y,int map[MAP_BLOCKS][MAP_BLOCKS]);						// Mevcut bloğun üst tarafında duvar olmalı mı

void StartGame(Tank *tank1, Tank *tank2, int map[MAP_BLOCKS][MAP_BLOCKS]);			// Oyunun başlangıç ayarlarını yapar

void PrintScoreBoard(Tank *tank1, Tank *tank2);										// Puan durumunu ekrana yazar

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
	srand(time(NULL));
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
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	Baslangic_Ayarlari();
	//GenerateMaze();
	//Ekran_Mesaji();

	Tank tank1;
	Tank tank2;

	tank1.score = 0;
	tank2.score = 0;

	HAL_UART_Receive_IT(&huart2,(uint8_t*)&uartInputData , 1);


	// UART üzerinden oyuncuların adlarını alıp tank.name değişkenine kaydediyoruz

	BSP_LCD_SetFont(&Font16);

	strcpy(startMessage,"Please Enter The Name of the Player One Maximum of 10 characters\nPut a '.' at the end.\n\n");
	BSP_LCD_DisplayStringAtLine(1, (uint8_t*)"Please Enter The Name ");
	BSP_LCD_DisplayStringAtLine(2, (uint8_t*)"of the Player One");
	BSP_LCD_DisplayStringAtLine(3, (uint8_t*)"Maximum of 10 characters");
	BSP_LCD_DisplayStringAtLine(4, (uint8_t*)"Put a '.' at the end.");
	HAL_UART_Transmit(&huart2,(uint8_t*) startMessage, strlen(startMessage), 3000);
	while(!playerOneNameSet);
	strcpy(tank1.name,playerName);

	strcpy(playerName,"");
	BSP_LCD_ClearStringLine(1);
	BSP_LCD_ClearStringLine(2);
	BSP_LCD_ClearStringLine(3);
	BSP_LCD_ClearStringLine(4);

	strcpy(startMessage, "Please Enter The Name of the Player Two Maximum of 10 characters\nPut a '.' at the end.\n\n");
	BSP_LCD_DisplayStringAtLine(1, (uint8_t*)"Please Enter The Name ");
	BSP_LCD_DisplayStringAtLine(2, (uint8_t*)"of the Player Two");
	BSP_LCD_DisplayStringAtLine(3, (uint8_t*)"Maximum of 10 characters");
	BSP_LCD_DisplayStringAtLine(4, (uint8_t*)"Put a '.' at the end.");
	HAL_UART_Transmit(&huart2,(uint8_t*) startMessage, strlen(startMessage), 3000);
	while(!playerTwoNameSet);
	strcpy(tank2.name,playerName);

	StartGame(&tank1,&tank2,map);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

				EraseTank(&tank1);
				EraseTank(&tank2);
				HAL_Delay(10);
				ButtonInput(&tank1);
				ButtonInput(&tank2);
				ActionTank(&tank1,&tank2,map);
				ActionTank(&tank2,&tank1,map);
				DrawTank(&tank1);
				DrawTank(&tank2);
				HAL_Delay(40);



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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_1_Pin BUTTON_2_Pin BUTTON_3_Pin BUTTON_4_Pin
                           BUTTON_5_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin|BUTTON_2_Pin|BUTTON_3_Pin|BUTTON_4_Pin
                          |BUTTON_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void Baslangic_Ayarlari(void) {
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);

	BSP_LCD_Init();

	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font20);
}

// Oyun kolunda basılan butonlara göre tankın hareket değişkenleri true ya da false değeri alır.
void ButtonInput(Tank *tank)
{
	tank->goBack 				= HAL_GPIO_ReadPin(tank->gpioPort, tank->pins[0]);
	tank->goForward 			= HAL_GPIO_ReadPin(tank->gpioPort, tank->pins[1]);
	tank->turnClockwise 		= HAL_GPIO_ReadPin(tank->gpioPort, tank->pins[2]);
	tank->turnCounterClockwise	= HAL_GPIO_ReadPin(tank->gpioPort, tank->pins[3]);
	tank->fire					= HAL_GPIO_ReadPin(tank->gpioPort, tank->pins[4]);
}

// Tank ve tank namlusu bu fonksiyonla çizilir.
void DrawTank(Tank *tank)
{
	DrawBullets(tank);
	BSP_LCD_SetTextColor(tank->color);
	BSP_LCD_FillCircle(tank->xPos, tank->yPos, tank->radius);

	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_DrawLine(tank->xPos, tank->yPos, tank->xPos + tank->xPosBarrel, tank->yPos + tank->yPosBarrel);
}

// Tankın yerine arka plan rengini çizdirerek tankı silmiş oluyoruz.
void EraseTank(Tank *tank)
{
	EraseBullets(tank);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillCircle(tank->xPos, tank->yPos, tank->radius);
}

// ButtonInput() fonksiyonunda ayarlanan değişkenlerin değerleri kontrol edilerek
// bu fonksiyonda tankın hareketleri sağlanır.
void ActionTank(Tank *tankMain,Tank *tankSecondary,int map[MAP_BLOCKS][MAP_BLOCKS])
{
	// Önce mermileri hareket ettiriyoruz.
	MoveBullets(tankMain,tankSecondary);

	// Tank saat yönünde dönecekse
	if(tankMain->turnClockwise)
	{
		// Tankın namlusu üst yarı düzlemdeyse namlunun x değişkeni artar.
		if(tankMain->yPosBarrel > 0)
		{
			tankMain->xPosBarrel += tankMain->radius / 5;
			tankMain->yPosBarrel = (int)sqrt(tankMain->radius*tankMain->radius - tankMain->xPosBarrel*tankMain->xPosBarrel);
		}

		// Tankın namlusu üst yarı düzlemdeyse namlunun x değişkeni azalır.
		else if(tankMain->yPosBarrel < 0)
		{
			tankMain->xPosBarrel -= tankMain->radius / 5;
			tankMain->yPosBarrel = -(int)sqrt(tankMain->radius*tankMain->radius - tankMain->xPosBarrel* tankMain->xPosBarrel);
		}

		// Tankın namlusu tamamen sağ tarafa bakıyorsa namlunun x değişkeni azalır.
		else if(tankMain->yPosBarrel == 0 && tankMain->xPosBarrel == tankMain->radius)
		{
			tankMain->xPosBarrel -= tankMain->radius / 5;
			tankMain->yPosBarrel = -(int)sqrt(tankMain->radius*tankMain->radius - tankMain->xPosBarrel* tankMain->xPosBarrel);
		}

		// Tankın namlusu tamamen sol tarafa bakıyorsa namlunun x değişkeni artar.
		else if(tankMain->yPosBarrel == 0 && tankMain->xPosBarrel == -tankMain->radius)
		{
			tankMain->xPosBarrel += tankMain->radius / 5;
			tankMain->yPosBarrel = (int)sqrt(tankMain->radius*tankMain->radius - tankMain->xPosBarrel* tankMain->xPosBarrel);
		}
	}

	// Tank saat yönünün tersine dönecekse
	if(tankMain->turnCounterClockwise)
	{
		if(tankMain->yPosBarrel > 0)
		{
			tankMain->xPosBarrel -= tankMain->radius / 5;
			tankMain->yPosBarrel = (int)sqrt(tankMain->radius*tankMain->radius - tankMain->xPosBarrel*tankMain->xPosBarrel);
		}
		else if(tankMain->yPosBarrel < 0)
		{
			tankMain->xPosBarrel += tankMain->radius / 5;
			tankMain->yPosBarrel = -(int)sqrt(tankMain->radius*tankMain->radius - tankMain->xPosBarrel* tankMain->xPosBarrel);
		}
		else if(tankMain->yPosBarrel == 0 && tankMain->xPosBarrel == tankMain->radius)
		{
			tankMain->xPosBarrel -= tankMain->radius / 5;
			tankMain->yPosBarrel = +(int)sqrt(tankMain->radius*tankMain->radius - tankMain->xPosBarrel* tankMain->xPosBarrel);
		}
		else if(tankMain->yPosBarrel == 0 && tankMain->xPosBarrel == -tankMain->radius)
		{
			tankMain->xPosBarrel += tankMain->radius / 5;
			tankMain->yPosBarrel = -(int)sqrt(tankMain->radius*tankMain->radius - tankMain->xPosBarrel* tankMain->xPosBarrel);
		}
	}

	// tank ileri gidecekse
	if(tankMain->goForward)
	{
		// map[][] dizisindeki değişkenleri ekrana uyarlamak için gereken genişletme oranı.
		int block_size = LCD_Y_SIZE / MAP_BLOCKS;

		// Tankımızın ilerleme oranı namlu uzunluğunun yarısı kadardır.
		int xVel = tankMain->xPosBarrel / 2;

		// Tank sağ tarafa hareket ediyorsa
		if(xVel > 0)
		{
			// Hareket olduğu sürece
			while(xVel)
			{
				// Bloğun kenarında değilsek veya sağımızda duvar yoksa
				if((((((tankMain->yPos + tankMain->radius)/block_size) == (tankMain->yPos - tankMain->radius)/block_size))  && !IsWallRight(tankMain->yPos/block_size, tankMain->xPos / block_size , map)) || tankMain->xPos % block_size +2 < block_size - tankMain->radius  )
				{
					tankMain->xPos++;
					xVel--;
				}
				else
				{
					xVel = 0;
				}
			}
		}

		// Tank sol tarafa hareket ediyorsa
		else if(xVel < 0)
		{
			xVel = -xVel;
			while(xVel)
			{
				// Bloğun kenarında değilsek veya solumuzda duvar yoksa
				if((((((tankMain->yPos + tankMain->radius)/block_size) == (tankMain->yPos - tankMain->radius)/block_size))  && !IsWallLeft(tankMain->yPos/block_size, tankMain->xPos / block_size , map)) || tankMain->xPos % block_size -2 > tankMain->radius  )
				{
					tankMain->xPos--;
					xVel--;
				}
				else
				{
					xVel = 0;
				}
			}
		}

		int yVel = tankMain->yPosBarrel / 2;

		// Tank aşağı hareket ediyorsa
		if(yVel > 0)
		{
			while(yVel)
			{
				// Bloğun kenarında değilsek veya altımızda duvar yoksa
				if((((((tankMain->xPos + tankMain->radius)/block_size) == (tankMain->xPos - tankMain->radius)/block_size))  && !IsWallBottom(tankMain->yPos/block_size, tankMain->xPos / block_size , map)) || tankMain->yPos % block_size +3 < block_size - tankMain->radius  )
				{
					tankMain->yPos++;
					yVel--;
				}
				else
				{
					yVel = 0;
				}
			}
		}
		// Tank yukarı hareket ediyorsa
		else if(yVel < 0)
		{
			yVel = -yVel;
			while(yVel)
			{
				// Bloğun kenarında değilsek veya üstümüzde duvar yoksa
				if((((((tankMain->xPos + tankMain->radius)/block_size) == (tankMain->xPos - tankMain->radius)/block_size))  && !IsWallUp(tankMain->yPos/block_size, tankMain->xPos / block_size , map)) || tankMain->yPos % block_size -2 > tankMain->radius  )
				{
					tankMain->yPos--;
					yVel--;
				}
				else
				{
					yVel = 0;
				}
			}
		}
	}

	if(tankMain->goBack)
	{
		int block_size = LCD_Y_SIZE / MAP_BLOCKS;

		int xVel = -tankMain->xPosBarrel / 2;
		if(xVel > 0)
		{
			while(xVel)
			{
				if((((((tankMain->yPos + tankMain->radius)/block_size) == (tankMain->yPos - tankMain->radius)/block_size))  && !IsWallRight(tankMain->yPos/block_size, tankMain->xPos / block_size , map)) || tankMain->xPos % block_size +2 < block_size - tankMain->radius  )
				{
					tankMain->xPos++;
					xVel--;
				}
				else
				{
					xVel = 0;
				}
			}

		}
		else if(xVel < 0)
		{
			xVel = -xVel;
			while(xVel)
			{
				if((((((tankMain->yPos + tankMain->radius)/block_size) == (tankMain->yPos - tankMain->radius)/block_size))  && !IsWallLeft(tankMain->yPos/block_size, tankMain->xPos / block_size , map)) || tankMain->xPos % block_size -2 > tankMain->radius  )
				{
					tankMain->xPos--;
					xVel--;
				}
				else
				{
					xVel = 0;
				}
			}
		}

		int yVel = -tankMain->yPosBarrel / 2;
		if(yVel > 0)
		{
			while(yVel)
			{
				if((((((tankMain->xPos + tankMain->radius)/block_size) == (tankMain->xPos - tankMain->radius)/block_size))  && !IsWallBottom(tankMain->yPos/block_size, tankMain->xPos / block_size , map)) || tankMain->yPos % block_size +3 < block_size - tankMain->radius  )
				{
					tankMain->yPos++;
					yVel--;
				}
				else
				{
					yVel = 0;
				}
			}

		}
		else if(yVel < 0)
		{
			yVel = -yVel;
			while(yVel)
			{
				if((((((tankMain->xPos + tankMain->radius)/block_size) == (tankMain->xPos - tankMain->radius)/block_size))  && !IsWallUp(tankMain->yPos/block_size, tankMain->xPos / block_size , map)) || tankMain->yPos % block_size -2 > tankMain->radius  )
				{
					tankMain->yPos--;
					yVel--;
				}
				else
				{
					yVel = 0;
				}
			}
		}
}

	// tank.bullets.lifetime değeri 0'a eşit ise o mermi ateşlenmemiştir demektir.
	// Bu durumda bir mermi ateşlenir.
	// Merminin hareket hızı namlu uzunluğu ile aynıdır.
	// Tankların hızı namlu uzunluğunun yarısı kadar olduğu için mermiler tankların iki katı hızda hareket eder.
	if(tankMain->fire)
	{
		int i;
		for(i=0; i< MAX_BULLET_COUNT; i++)
		{
			if(tankMain->bullets[i].lifeTime == 0)
			{
				tankMain->bullets[i].lifeTime = MAX_BULLET_LIFETIME;
				tankMain->bullets[i].xPos = tankMain->xPos + tankMain->xPosBarrel;
				tankMain->bullets[i].yPos = tankMain->yPos + tankMain->yPosBarrel;
				tankMain->bullets[i].xVelocity = tankMain->xPosBarrel;
				tankMain->bullets[i].yVelocity = tankMain->yPosBarrel;
				break;
			}
		}
	}
}

// Mermilerin hareketi tankların hareketlerine benzer.
// En büyük fark mermiler tankların aksine duvara çarpınca durak yerine ters yönde hareketine devam eder.
// Böylece duvardan sekmiş olur.
void MoveBullets(Tank *tankMain, Tank *tankSecondary)
{
	int block_size = LCD_Y_SIZE / MAP_BLOCKS;
	int i;
	for(i=0; i< MAX_BULLET_COUNT; i++)
	{
		if(tankMain->bullets[i].lifeTime > 0)
		{
			int xvelocity = tankMain->bullets[i].xVelocity;
			if(xvelocity < 0)
			{
				xvelocity = -xvelocity;
			}
			while(xvelocity)
			{
				if(tankMain->bullets[i].xVelocity > 0)
				{
					if( !IsWallRight(tankMain->bullets[i].yPos / block_size, tankMain->bullets[i].xPos / block_size, map) || tankMain->bullets[i].xPos % block_size < block_size - 3)
					{
						tankMain->bullets[i].xPos++;
						if(tankMain->bullets[i].xPos % block_size == 0)
						{
							tankMain->bullets[i].xPos += 2;
						}
					}
					else
					{
						tankMain->bullets[i].xVelocity = -tankMain->bullets[i].xVelocity;
					}
				}
				else
				{
					if(!IsWallLeft(tankMain->bullets[i].yPos / block_size, tankMain->bullets[i].xPos / block_size, map) || tankMain->bullets[i].xPos % block_size > 3)
					{
						tankMain->bullets[i].xPos--;
						if(tankMain->bullets[i].xPos % block_size == 0)
						{
							tankMain->bullets[i].xPos -= 2;
						}
					}
					else
					{
						tankMain->bullets[i].xVelocity = -tankMain->bullets[i].xVelocity;
					}
				}
				xvelocity--;
			}

			int yvelocity = tankMain->bullets[i].yVelocity;
			if(yvelocity < 0)
			{
				yvelocity = -yvelocity;
			}
			while(yvelocity)
			{
				if(tankMain->bullets[i].yVelocity > 0)
				{
					if( !IsWallBottom(tankMain->bullets[i].yPos / block_size, tankMain->bullets[i].xPos / block_size, map) || tankMain->bullets[i].yPos % block_size < block_size - 3)
					{
						tankMain->bullets[i].yPos++;
					}
					else
					{
						tankMain->bullets[i].yVelocity = -tankMain->bullets[i].yVelocity;
					}
				}
				else
				{
					if( !IsWallUp(tankMain->bullets[i].yPos / block_size, tankMain->bullets[i].xPos / block_size, map) || tankMain->bullets[i].yPos % block_size > 3)
					{
						tankMain->bullets[i].yPos--;
					}
					else
					{
						tankMain->bullets[i].yVelocity = -tankMain->bullets[i].yVelocity;
					}
				}
				yvelocity--;
			}

			// Bu if-else bloğu mermilerin 2 blok sınırında ateşlenmesiyle ortaya çıkan bir hatayı düzeltmek için yazılmıştır.
			if(tankMain->bullets[i].xPos % block_size < 3)
			{
				tankMain->bullets[i].xPos += 2;
			}
			else if(tankMain->bullets[i].xPos % block_size > block_size - 3)
			{
				tankMain->bullets[i].xPos-= 2;
			}
			if(tankMain->bullets[i].yPos % block_size < 3)
			{
				tankMain->bullets[i].yPos += 2;
			}
			else if(tankMain->bullets[i].yPos % block_size > block_size - 3)
			{
				tankMain->bullets[i].yPos-= 2;
			}

			// Mermilerin her hareketinde ömürleri kısalır.
			tankMain->bullets[i].lifeTime--;
		}
	}
	// Mermilerin hareketi tamamlandığında tanklarla temas edip etmediği kontrol edilir.
	CheckForHit(tankMain,tankSecondary);
}

void CheckForHit(Tank *tank1,Tank *tank2)
{
	bool tank1Dead = false;
	bool tank2Dead = false;
	int i = 0;
	for(i = 0; i<MAX_BULLET_COUNT; i++)
	{
		if(tank1->bullets[i].lifeTime > 0)
		{
			int xDist1 = tank1->xPos - tank1->bullets[i].xPos;
			int yDist1 = tank1->yPos - tank1->bullets[i].yPos;
			if( sqrt( (xDist1)*(xDist1) + (yDist1)*(yDist1)) <= 9)
			{
				tank1Dead = true;
			}

			int xDist2 = tank2->xPos - tank1->bullets[i].xPos;
			int yDist2 = tank2->yPos - tank1->bullets[i].yPos;
			if( sqrt( (xDist2)*(xDist2) + (yDist2)*(yDist2)) <= 9)
			{
				tank2Dead = true;
			}
		}

		if(tank2->bullets[i].lifeTime > 0)
		{
			int xDist1 = tank1->xPos - tank2->bullets[i].xPos;
			int yDist1 = tank1->yPos - tank2->bullets[i].yPos;
			if( sqrt( (xDist1)*(xDist1) + (yDist1)*(yDist1)) <= 9)
			{
				tank1Dead = true;
			}

			int xDist2 = tank2->xPos - tank2->bullets[i].xPos;
			int yDist2 = tank2->yPos - tank2->bullets[i].yPos;
			if( sqrt( (xDist2)*(xDist2) + (yDist2)*(yDist2)) <= 9)
			{
				tank2Dead = true;
			}
		}
	}
	if(tank1Dead)
	{
		tank2->score++;
	}
	if(tank2Dead)
	{
		tank1->score++;
	}
	if(tank1Dead || tank2Dead)
	{
		if(tank1->ID == 1)
		{
			StartGame(tank1, tank2, map);
		}
		else
		{
			StartGame(tank2, tank1, map);
		}
	}
}

void EraseBullets(Tank *tank)
{
	int i;
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	for(i=0; i< MAX_BULLET_COUNT; i++)
	{
		if(tank->bullets[i].lifeTime != 0)
		{
			BSP_LCD_FillCircle(tank->bullets[i].xPos,tank->bullets[i].yPos , BULLET_RADIUS);
		}
	}
}

void DrawBullets(Tank *tank)
{
	int i;
	BSP_LCD_SetTextColor(BULLET_COLOR);
	for(i=0; i< MAX_BULLET_COUNT; i++)
	{
		if(tank->bullets[i].lifeTime != 0)
		{
			BSP_LCD_FillCircle(tank->bullets[i].xPos,tank->bullets[i].yPos , BULLET_RADIUS);
		}
	}
}

// Labirent üretme algoritması raporda görsel olarak anlatılmıştır.
void GenerateMaze(int map[MAP_BLOCKS][MAP_BLOCKS])
{

	int count = 0;
	int currentValue = 1;
	int mapxIndex = 0;
	int mapyIndex = 0;
	int randomValue;

	int neighbours[4] = {0,0,0,0};  // Üst,Sol,Alt,Sağ

	for(int x=0; x<MAP_BLOCKS; x++)
	{
		for(int y=0; y<MAP_BLOCKS; y++)
		{
			map[x][y] = 0;
		}
	}

	while(1)
	{
		map[mapxIndex][mapyIndex] = currentValue;
		currentValue++;
		count++;

		if(count == MAP_BLOCKS*MAP_BLOCKS)
		{
			break;
		}

		GetNeighbours(map, neighbours, mapxIndex, mapyIndex);

		if(!NeighbourCount(neighbours))
		{
			currentValue--;
			while(!NeighbourCount(neighbours))
			{
				if(mapxIndex != 0 && map[mapxIndex-1][mapyIndex] == currentValue-1)
				{
					mapxIndex--;
					currentValue--;
				}
				else if( mapxIndex < MAP_BLOCKS && map[mapxIndex+1][mapyIndex] == currentValue-1)
				{
					mapxIndex++;
					currentValue--;
				}
				else if(mapyIndex != 0 && map[mapxIndex][mapyIndex-1] == currentValue-1)
				{
					mapyIndex--;
					currentValue--;
				}
				else
				{
					mapyIndex++;
					currentValue--;
				}
				GetNeighbours(map, neighbours, mapxIndex, mapyIndex);
			}
			currentValue++;
		}

		if(NeighbourCount(neighbours))
		{
			do{
				randomValue = (rand() + 1) %4;
			}while(neighbours[randomValue] == 0);

			switch (randomValue)
			{
				case 0:
					mapxIndex--;
					break;
				case 1:
					mapyIndex--;
					break;
				case 2:
					mapxIndex++;
					break;
				case 3:
					mapyIndex++;
					break;
				default:
					break;
			}
		}
	}
	DrawMap(map);
}

int NeighbourCount(int neighbours[4])
{
	int count = 0;

	for(int i=0; i<4; i++)
	{
		if(neighbours[i] != 0)
		{
			count++;
		}
	}
	return count;
}

void GetNeighbours(int map[MAP_BLOCKS][MAP_BLOCKS], int neighbours[4], int mapxIndex, int mapyIndex)
{
	// Sol tarafa hareket edilebilir mi
	if(mapyIndex > 0 && map[mapxIndex][mapyIndex-1] == 0)
	{
		neighbours[1] = 1;
	}
	else
	{
		neighbours[1] = 0;
	}
	// Üst tarafa hareket edilebilir mi
	if(mapxIndex > 0 && map[mapxIndex-1][mapyIndex] == 0)
	{
		neighbours[0] = 1;
	}
	else
	{
		neighbours[0] = 0;
	}
	// Sağ tarafa hareket edilebilir mi
	if(mapyIndex < MAP_BLOCKS -1 && map[mapxIndex][mapyIndex+1] == 0)
	{
		neighbours[3] = 1;
	}
	else
	{
		neighbours[3] = 0;
	}
	// Alt tarafa hareket edilebilir mi
	if(mapxIndex < MAP_BLOCKS -1 && map[mapxIndex+1][mapyIndex] == 0)
	{
		neighbours[2] = 1;
	}
	else
	{
		neighbours[2] = 0;
	}
}

void DrawMap(int map[MAP_BLOCKS][MAP_BLOCKS])
{
	BSP_LCD_SetTextColor(WALL_COLOR);
	for(int x=0; x<MAP_BLOCKS; x++)
	{
		for(int y=0; y<MAP_BLOCKS; y++)
		{
			if( IsWallRight(x,y,map) )
			{
				BSP_LCD_DrawVLine( (y+1)* (LCD_Y_SIZE / MAP_BLOCKS) , (x)   * (LCD_Y_SIZE / MAP_BLOCKS) , (LCD_Y_SIZE / MAP_BLOCKS) );
			}
			if( IsWallBottom(x,y,map) )
			{
				BSP_LCD_DrawHLine( (y) * (LCD_Y_SIZE / MAP_BLOCKS) ,  (x+1) * (LCD_Y_SIZE / MAP_BLOCKS), (LCD_Y_SIZE / MAP_BLOCKS));
			}
			if( IsWallLeft(x, y, map))
			{
				BSP_LCD_DrawVLine( (y)* (LCD_Y_SIZE / MAP_BLOCKS) , (x)   * (LCD_Y_SIZE / MAP_BLOCKS) , (LCD_Y_SIZE / MAP_BLOCKS) );
			}
			if( IsWallUp(x, y, map))
			{
				BSP_LCD_DrawHLine( (y) * (LCD_Y_SIZE / MAP_BLOCKS) ,  (x) * (LCD_Y_SIZE / MAP_BLOCKS), (LCD_Y_SIZE / MAP_BLOCKS));
			}
		}
	}
}

bool IsWallRight(int x, int y,int map[MAP_BLOCKS][MAP_BLOCKS])
{
	if((map[x][y] - map[x][y+1] != 1 && map[x][y] - map[x][y+1] != -1) || y == MAP_BLOCKS-1)
	{
		return true;
	}
	return false;
}

bool IsWallBottom(int x, int y,int map[MAP_BLOCKS][MAP_BLOCKS])
{
	if((map[x][y] - map[x+1][y] != 1 && map[x][y] - map[x+1][y] != -1) || x == MAP_BLOCKS - 1)
	{
		return true;
	}
	return false;
}

bool IsWallLeft(int x, int y,int map[MAP_BLOCKS][MAP_BLOCKS])
{
	if((map[x][y] - map[x][y-1] != 1 && map[x][y] - map[x][y-1] != -1) || y == 0)
	{
		return true;
	}
	return false;
}
bool IsWallUp(int x, int y,int map[MAP_BLOCKS][MAP_BLOCKS])
{
	if((map[x][y] - map[x-1][y] != 1 && map[x][y] - map[x-1][y] != -1) || x == 0)
	{
		return true;
	}
	return false;
}

void StartGame(Tank *tank1, Tank *tank2,int map[MAP_BLOCKS][MAP_BLOCKS])
{

	if(tank1->score == TURNS_TO_WIN && tank2->score == TURNS_TO_WIN)
	{
		BSP_LCD_SetFont(&Font16);
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_DisplayStringAt(0, 120,(uint8_t*) "IT IS A DRAW!", CENTER_MODE);
		BSP_LCD_DisplayStringAt(0, 140,(uint8_t*)"PRESS FIRE BUTTON TO REMATCH!", CENTER_MODE);
		tank1->score = 0;
		tank2->score = 0;
		HAL_Delay(500);
		while(!HAL_GPIO_ReadPin(tank1->gpioPort, tank2->pins[4]) && !HAL_GPIO_ReadPin(tank2->gpioPort, tank2->pins[4]));
	}
	else if(tank1->score == TURNS_TO_WIN)
	{
		char endGameMessage[30];
		strcpy(endGameMessage,tank1->name);
		strcat(endGameMessage," WON THE GAME!");
		BSP_LCD_SetFont(&Font16);
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_DisplayStringAt(0, 120,(uint8_t*)endGameMessage, CENTER_MODE);
		BSP_LCD_DisplayStringAt(0, 140,(uint8_t*)"PRESS FIRE BUTTON TO REMATCH!", CENTER_MODE);
		tank1->score = 0;
		tank2->score = 0;
		HAL_Delay(500);
		while(!HAL_GPIO_ReadPin(tank1->gpioPort, tank2->pins[4]) && !HAL_GPIO_ReadPin(tank2->gpioPort, tank2->pins[4]));
	}
	else if(tank2->score == TURNS_TO_WIN)
	{
		char endGameMessage[30];
		strcpy(endGameMessage,tank2->name);
		strcat(endGameMessage," WON THE GAME!");
		BSP_LCD_SetFont(&Font16);
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_DisplayStringAt(0, 120,(uint8_t*)endGameMessage, CENTER_MODE);
		BSP_LCD_DisplayStringAt(0, 140,(uint8_t*)"PRESS FIRE BUTTON TO REMATCH!", CENTER_MODE);
		tank1->score = 0;
		tank2->score = 0;
		HAL_Delay(500);
		while(!HAL_GPIO_ReadPin(tank1->gpioPort, tank1->pins[4]) && !HAL_GPIO_ReadPin(tank2->gpioPort, tank2->pins[4]));
	}

	int i;

	tank1->ID = 1;
	tank1->radius = TANK_RADIUS;
	tank1->xPos = 15;
	tank1->yPos = 15;
	tank1->color = LCD_COLOR_BLUE;

	tank1->fire = false;
	tank1->goForward = false;
	tank1->goBack = false;
	tank1->turnClockwise = false;
	tank1->turnCounterClockwise = false;

	tank1->gpioPort = GPIOD;
	tank1->pins[0] = GPIO_PIN_9;
	tank1->pins[1] = GPIO_PIN_10;
	tank1->pins[2] = GPIO_PIN_11;
	tank1->pins[3] = GPIO_PIN_12;
	tank1->pins[4] = GPIO_PIN_13;
	tank1->xPosBarrel = tank1->radius;
	tank1->yPosBarrel = 0;
	for(i=0; i<MAX_BULLET_COUNT; i++)
	{
		tank1->bullets[i].lifeTime = 0;
		tank1->bullets[i].xPos = 0;
		tank1->bullets[i].yPos = 0;
		tank1->bullets[i].xVelocity = 0;
		tank1->bullets[i].yVelocity = 0;
	}

	tank2->ID = 2;
	tank2->radius = TANK_RADIUS;
	tank2->xPos = 225;
	tank2->yPos = 225;
	tank2->color = LCD_COLOR_RED;

	tank2->fire = false;
	tank2->goForward = false;
	tank2->goBack = false;
	tank2->turnClockwise = false;
	tank2->turnCounterClockwise = false;

	tank2->gpioPort = GPIOC;
	tank2->pins[0] = GPIO_PIN_0;
	tank2->pins[1] = GPIO_PIN_1;
	tank2->pins[2] = GPIO_PIN_2;
	tank2->pins[3] = GPIO_PIN_3;
	tank2->pins[4] = GPIO_PIN_13;
	tank2->xPosBarrel = -tank1->radius;
	tank2->yPosBarrel = 0;
	for(i=0; i<MAX_BULLET_COUNT; i++)
	{
		tank2->bullets[i].lifeTime = 0;
		tank2->bullets[i].xPos = 0;
		tank2->bullets[i].yPos = 0;
		tank2->bullets[i].xVelocity = 0;
		tank2->bullets[i].yVelocity = 0;
	}
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	GenerateMaze(map);
	PrintScoreBoard(tank1,tank2);
}

void PrintScoreBoard(Tank *tank1, Tank *tank2)
{
	BSP_LCD_SetTextColor(WALL_COLOR);
	BSP_LCD_DrawHLine(240, 120, 80);
	BSP_LCD_SetTextColor(tank1->color);
	BSP_LCD_FillCircle(280, 40, tank1->radius*2);

	BSP_LCD_SetTextColor(tank2->color);
	BSP_LCD_FillCircle(280, 160, tank2->radius*2);

	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_DrawHLine(280, 40, tank1->radius*2);
	BSP_LCD_DrawHLine(280, 160, tank2->radius*2);

	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayChar(272, 80, tank1->score+48);
	BSP_LCD_DisplayChar(272, 200, tank2->score+48);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if(!playerOneNameSet)
		{
			if(uartInputData == '.' || strlen(playerName) > 10)
			{
				playerOneNameSet = true;
			}
			else
			{
				strncat(playerName,&uartInputData,1);
			}
		}
		else if(!playerTwoNameSet)
		{
			if(uartInputData == '.' || strlen(playerName) > 10)
			{
				playerTwoNameSet = true;
			}
			else
			{
				strncat(playerName,&uartInputData,1);
			}
		}

		if(!playerOneNameSet || !playerTwoNameSet)
		{
			HAL_UART_Receive_IT(&huart2,(uint8_t*)&uartInputData , 1);
		}

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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - APRS/AFSK Audio Tone Generator
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "afsk.h"
#include "ax25.h"
#include <string.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMD_TYPE_FREQ_CHANGE  0x01
#define CMD_TYPE_TELEMETRY    0x02

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PKT_MAX_PAYLOAD     256
#define PKT_HEADER_SIZE     4
#define PKT_TOTAL_SIZE      260

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */
static const char SRC_CALL[]  = "VU3CDI";
static const uint8_t SRC_SSID = 5;
static const char DST_CALL[]  = "VU2CWN";
static const uint8_t DST_SSID = 0;
static const char PATH1_CALL[] = "WIDE";
static const uint8_t PATH1_SSID = 1;
static const char PATH2_CALL[] = "WIDE";
static const uint8_t PATH2_SSID = 2;

/* Frequency control */
#define FREQ_DEFAULT "435.2480"
#define FREQ_BACKUP  "435.2500"
static volatile uint8_t use_backup_freq = 0;

/* Packet structure */
typedef struct __attribute__((packed)) {
    uint8_t command_type;
    uint8_t payload_length;
    uint8_t padding[2];
    uint8_t payload[PKT_MAX_PAYLOAD];
} DataPacket_t;

/* Reception buffer - receives complete packet structure */
static DataPacket_t rxData;
static DataPacket_t processData;
static volatile uint8_t packet_ready = 0;

/* Statistics */
static uint32_t packets_valid = 0;
static uint32_t packets_rejected = 0;
static uint32_t uart_errors = 0;

/* Buffers - Reduced size for memory optimization */
#define AX25_BUF_SIZE 512  // Reduced from 4096
static uint8_t ax25_buffer[AX25_BUF_SIZE];
static uint16_t ax25_len = 0;

/* DAC pin masks for 4-bit R-2R ladder DAC */
static uint32_t dac_set_masks[16];
static uint32_t dac_reset_masks[16];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
static void DRA_Send(const char *s);
static void DRA_Init(void);
static void ProcessCommand(DataPacket_t *pkt);
static void TransmitTelemetry(const char *payload_text);
static inline int IsValidPacket(DataPacket_t *packet);
extern uint8_t afsk_isBusy(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief APRS/AFSK Audio Tone Generation Overview
 *
 * This code implements Bell 202 AFSK (Audio Frequency Shift Keying) for APRS:
 * - Mark (1):  1200 Hz
 * - Space (0): 2200 Hz
 * - Baud rate: 1200 baud
 *
 * Signal Flow:
 * 1. AX.25 frame encoding (ax25_encode) - Creates proper APRS packet
 * 2. AFSK generation (afsk_generate) - Converts bits to audio samples
 * 3. Timer interrupt (afsk_timer_tick) - Outputs samples at 9600 Hz via DAC
 * 4. DAC output (DAC_Write4) - 4-bit R-2R ladder creates analog audio
 * 5. Audio feeds DRA818U microphone input
 *
 * Timer3 Configuration for 9600 Hz:
 * - System clock: 84 MHz (APB1)
 * - Prescaler: 83 (84MHz / 84 = 1 MHz)
 * - Period: 103 (1MHz / 104 ≈ 9615 Hz ≈ 9600 Hz)
 *
 * At 9600 Hz sample rate:
 * - 1200 Hz tone = 8 samples per cycle (9600/1200)
 * - 2200 Hz tone = 4.36 samples per cycle (9600/2200)
 */

/* Check if payload contains printable ASCII */
static uint8_t IsValidPayload(const uint8_t *payload, uint8_t len)
{
    if (len == 0 || len > 256) return 0;

    for (uint8_t i = 0; i < len; i++) {
        if ((payload[i] >= 32 && payload[i] <= 126) ||
            payload[i] == '\r' || payload[i] == '\n' || payload[i] == '\t') {
            continue;
        }
        return 0;
    }
    return 1;
}

/* Validation function - CHECK COMMAND TYPE FIRST */
static inline int IsValidPacket(DataPacket_t *packet)
{
    if (packet->command_type < CMD_TYPE_FREQ_CHANGE ||
        packet->command_type > CMD_TYPE_TELEMETRY) {
        return 0;
    }

    if (packet->payload_length == 0 || packet->payload_length > 256) {
        return 0;
    }

    if (!IsValidPayload(packet->payload, packet->payload_length)) {
        return 0;
    }

    return 1;
}

/**
 * @brief Write 4-bit value to R-2R DAC
 * @param v: 4-bit value (0-15)
 *
 * DAC Pin Assignment:
 * - LSB (bit 0): PA4
 * - BIT_1 (bit 1): PA6
 * - BIT_2 (bit 2): PA7
 * - MSB (bit 3): PA0
 *
 * Uses precomputed masks for fast GPIO writes via BSRR register
 */
void DAC_Write4(uint8_t v)
{
    GPIOA->BSRR = dac_set_masks[v & 0x0F] | dac_reset_masks[v & 0x0F];
}

/**
 * @brief Precompute GPIO masks for all 16 DAC values
 *
 * BSRR register layout:
 * - Lower 16 bits: Set pins (write 1)
 * - Upper 16 bits: Reset pins (write 1)
 *
 * This optimization allows single-cycle GPIO writes
 */
void DAC_PrecomputeMasks(void)
{
    for (uint8_t v = 0; v < 16; v++) {
        uint32_t set_mask = 0;
        uint32_t reset_mask = 0;
        if (v & 0x01) set_mask |= LSB_Pin; else reset_mask |= (LSB_Pin << 16);
        if (v & 0x02) set_mask |= BIT_1_Pin; else reset_mask |= (BIT_1_Pin << 16);
        if (v & 0x04) set_mask |= BIT_2_Pin; else reset_mask |= (BIT_2_Pin << 16);
        if (v & 0x08) set_mask |= MSB_Pin; else reset_mask |= (MSB_Pin << 16);
        dac_set_masks[v] = set_mask;
        dac_reset_masks[v] = reset_mask;
    }
}

/**
 * @brief Timer3 interrupt callback - AFSK audio sample output
 *
 * Called at 9600 Hz to generate AFSK audio tones:
 * - afsk_timer_tick() fetches next audio sample
 * - Sample output via DAC_Write4()
 * - Creates 1200 Hz (mark) and 2200 Hz (space) tones
 *
 * At 9600 Hz sample rate:
 * - 1200 Hz: 8 samples per cycle
 * - 2200 Hz: ~4.36 samples per cycle
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        afsk_timer_tick();
    }
}

/* UART Receive Complete Callback - Receives COMPLETE PACKET */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (IsValidPacket(&rxData)) {
            packets_valid++;
            memcpy(&processData, &rxData, sizeof(DataPacket_t));
            packet_ready = 1;
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        } else {
            packets_rejected++;
        }

        HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxData, sizeof(DataPacket_t));
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        uart_errors++;

        HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxData, sizeof(DataPacket_t));
    }
}

/* Command Processing */
static void ProcessCommand(DataPacket_t *packet_cmd)
{
    HAL_IWDG_Refresh(&hiwdg);

    if (!IsValidPacket(packet_cmd)) {
        return;
    }

    if (packet_cmd->payload_length < PKT_MAX_PAYLOAD) {
        packet_cmd->payload[packet_cmd->payload_length] = '\0';
    } else {
        packet_cmd->payload[PKT_MAX_PAYLOAD - 1] = '\0';
    }

    switch (packet_cmd->command_type)
    {
        case CMD_TYPE_FREQ_CHANGE:
            TransmitTelemetry((char*)packet_cmd->payload);
            break;

        case CMD_TYPE_TELEMETRY:
            TransmitTelemetry((char*)packet_cmd->payload);
            break;

        default:
            break;
    }
}

/**
 * @brief Transmit APRS packet via AFSK
 *
 * APRS Packet Structure:
 * 1. Flag (0x7E) - Packet delimiter
 * 2. Destination Address (7 bytes) - VU2CWN-0
 * 3. Source Address (7 bytes) - VU3CDI-5
 * 4. Path (14 bytes) - WIDE1-1,WIDE2-1
 * 5. Control (0x03), PID (0xF0)
 * 6. Information Field - ">Hello World Somaiya OrbitRadio 73"
 * 7. FCS (2 bytes) - Frame Check Sequence (CRC)
 * 8. Flag (0x7E)
 *
 * The ax25_encode() function creates this properly formatted AX.25 frame
 * The afsk_generate() converts it to audio samples at 9600 Hz
 * Timer3 ISR outputs samples at 9600 Hz via DAC
 */
static void TransmitTelemetry(const char *payload_text)
{
    char temp[128];  // Reduced from 300
    uint8_t len = 0;

    // Manual string concatenation to save stack
    temp[len++] = '>';
    while (*payload_text && len < 100) {
        temp[len++] = *payload_text++;
    }
    const char *suffix = " | SVV-KJSIT, NEW LEAP LABS, Somaiya OrbitRadio 73";
    while (*suffix && len < 127) {
        temp[len++] = *suffix++;
    }
    temp[len] = '\0';

    HAL_IWDG_Refresh(&hiwdg);

    memset(ax25_buffer, 0, AX25_BUF_SIZE);
    ax25_len = 0;

    ax25_encode(ax25_buffer, &ax25_len,
                SRC_CALL, SRC_SSID,
                DST_CALL, DST_SSID,
                PATH1_CALL, PATH1_SSID,
                PATH2_CALL, PATH2_SSID,
                temp);

    if (ax25_len == 0) {
        return;
    }

    HAL_Delay(200);
    HAL_IWDG_Refresh(&hiwdg);

    afsk_generate(ax25_buffer, ax25_len);

    HAL_GPIO_WritePin(PTT_UHF_GPIO_Port, PTT_UHF_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);

    afsk_start();

    uint32_t start = HAL_GetTick();
    while (afsk_isBusy() && (HAL_GetTick() - start) < 20000) {
        HAL_Delay(10);
        HAL_IWDG_Refresh(&hiwdg);
    }

    HAL_Delay(200);
    afsk_stop();

    HAL_GPIO_WritePin(PTT_UHF_GPIO_Port, PTT_UHF_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
}

void DRA_Send(const char *s)
{
    HAL_UART_Transmit(&huart6, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

void DRA_Init(void)
{
    const char *freq = use_backup_freq ? FREQ_BACKUP : FREQ_DEFAULT;
    HAL_Delay(500);
    DRA_Send("AT+DMOCONNECT");
    HAL_Delay(300);

    char cmd[64];  // Reduced from 100
    uint8_t idx = 0;
    const char *prefix = "AT+DMOSETGROUP=0,";
    while (*prefix) cmd[idx++] = *prefix++;
    const char *f = freq;
    while (*f) cmd[idx++] = *f++;
    cmd[idx++] = ',';
    f = freq;
    while (*f) cmd[idx++] = *f++;
    const char *suffix = ",0000,0,0000";
    while (*suffix) cmd[idx++] = *suffix++;
    cmd[idx] = '\0';

    DRA_Send(cmd);
    HAL_Delay(300);
    DRA_Send("AT+DMOSETVOLUME=6");
    HAL_Delay(200);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  DAC_PrecomputeMasks();
  HAL_Delay(100);

  afsk_Init();

  DRA_Init();

  memset(&rxData, 0, sizeof(DataPacket_t));
  memset(&processData, 0, sizeof(DataPacket_t));
  packet_ready = 0;

  HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxData, sizeof(DataPacket_t));

  uint32_t last_status = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_IWDG_Refresh(&hiwdg);

    if (packet_ready)
    {
        packet_ready = 0;
        ProcessCommand(&processData);
    }

    uint32_t now = HAL_GetTick();

    if ((now - last_status) >= 10000) {
        last_status = now;
    }

    HAL_Delay(10);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 103;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RS485_RE_Pin|RS485_DE_Pin|PTT_UHF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BIT_1_Pin|BIT_2_Pin|LD2_Pin|MSB_Pin
                          |LSB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_RE_Pin RS485_DE_Pin PTT_UHF_Pin */
  GPIO_InitStruct.Pin = RS485_RE_Pin|RS485_DE_Pin|PTT_UHF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BIT_1_Pin BIT_2_Pin LD2_Pin MSB_Pin LSB_Pin */
  GPIO_InitStruct.Pin = BIT_1_Pin|BIT_2_Pin|LD2_Pin|MSB_Pin|LSB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

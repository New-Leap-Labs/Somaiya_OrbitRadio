/* main.c
 * UART (Binary Protocol) -> APRS (AX.25) -> AFSK1200 -> DRA818U
 * VERSION 4 - Binary Protocol with Sync Header
 * FIXED: Properly handles OrbitAid_Command structure
 */

#include "main.h"
#include "afsk.h"
#include "ax25.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Hardware handles */
UART_HandleTypeDef huart1; /* UART1 async (USART1) PA9/PA10 */
UART_HandleTypeDef huart2; /* Debug (USART2) PA2/PA3 */
UART_HandleTypeDef huart6; /* DRA818U (USART6) PC6/PC7 */
TIM_HandleTypeDef  htim3;  /* sample timer */

/* APRS config */
static const char SRC_CALL[]  = "VU3CDI";
static const uint8_t SRC_SSID = 5;
static const char DST_CALL[]  = "VU2CWN";
static const uint8_t DST_SSID = 0;
static const char PATH1_CALL[] = "WIDE1";
static const uint8_t PATH1_SSID = 1;
static const char PATH2_CALL[] = "WIDE2";
static const uint8_t PATH2_SSID = 1;

/* Frequency control */
#define FREQ_DEFAULT "435.2480"
#define FREQ_BACKUP  "435.2500"
static volatile uint8_t use_backup_freq = 0;  /* 0 = default, 1 = backup */

/* Command definitions */
#define CMD_TYPE_FREQ_CHANGE  0x01
#define CMD_TYPE_TELEMETRY    0x02

typedef struct OrbitAid_Command_t
{
    uint8_t command_type;
    uint8_t command_payload_length;
    uint8_t padding[2];
    uint8_t command_payload[256];
} OrbitAid_Command;

/* Binary protocol reception state machine */
typedef enum {
    WAIT_SYNC1,      /* Waiting for 0xAA */
    WAIT_SYNC2,      /* Waiting for 0x55 */
    READ_HEADER,     /* Reading 4-byte header */
    READ_PAYLOAD     /* Reading payload */
} RxState_t;

static volatile RxState_t rx_state = WAIT_SYNC1;
static volatile OrbitAid_Command rx_command;
static volatile uint16_t rx_header_index = 0;
static volatile uint16_t rx_payload_index = 0;
static volatile uint8_t command_ready = 0;  /* Flag: complete command received */

/* buffers */
#define AX25_BUF_SIZE 4096
static uint8_t ax25_buffer[AX25_BUF_SIZE];
static uint16_t ax25_len = 0;

/* Async UART reception buffer */
static uint8_t uart_rx_byte;

/* DAC pin masks - precomputed for fast atomic writes */
static uint32_t dac_set_masks[16];
static uint32_t dac_reset_masks[16];

/* forward declarations */
void SystemClock_Config(void);
void GPIO_Init(void);
void USART2_Init(void);
void USART1_Init(void);
void USART6_Init(void);
void TIM3_Init(void);
void DAC_PrecomputeMasks(void);

static void Debug_Print(const char *s);
static void DRA_Send(const char *s);
static void DRA_Init(void);
void Debug_PrintClocks(void);
static void ProcessCommand(OrbitAid_Command *cmd);
static void TransmitTelemetry(const char *payload_text);

/* External function to check if AFSK is still transmitting */
extern uint8_t afsk_isBusy(void);
extern uint32_t afsk_getBitsRemaining(void);

/* Optimized DAC write function using precomputed BSRR masks */
void DAC_Write4(uint8_t v)
{
#if 1
    /* FAST VERSION: Atomic write using BSRR (requires all pins on same port) */
    GPIOA->BSRR = dac_set_masks[v & 0x0F] | dac_reset_masks[v & 0x0F];
#else
    /* SLOW VERSION: Individual GPIO writes (use if pins on different ports) */
    HAL_GPIO_WritePin(LSB_GPIO_Port, LSB_Pin,   (v>>0) & 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIT_1_GPIO_Port, BIT_1_Pin, (v>>1) & 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIT_2_GPIO_Port, BIT_2_Pin, (v>>2) & 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MSB_GPIO_Port, MSB_Pin,   (v>>3) & 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

/* Precompute BSRR masks for all 16 possible DAC values */
void DAC_PrecomputeMasks(void)
{
    for (uint8_t v = 0; v < 16; v++) {
        uint32_t set_mask = 0;
        uint32_t reset_mask = 0;

        /* LSB (bit 0) - PA15 */
        if (v & 0x01) set_mask |= LSB_Pin; else reset_mask |= (LSB_Pin << 16);
        /* BIT_1 (bit 1) - PA1 */
        if (v & 0x02) set_mask |= BIT_1_Pin; else reset_mask |= (BIT_1_Pin << 16);
        /* BIT_2 (bit 2) - PA4 */
        if (v & 0x04) set_mask |= BIT_2_Pin; else reset_mask |= (BIT_2_Pin << 16);
        /* MSB (bit 3) - PA6 */
        if (v & 0x08) set_mask |= MSB_Pin; else reset_mask |= (MSB_Pin << 16);

        dac_set_masks[v] = set_mask;
        dac_reset_masks[v] = reset_mask;
    }
}

/* HAL timer callback - calls afsk tick */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        afsk_timer_tick();
    }
}

/* UART Receive Complete Callback - Binary Protocol State Machine */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* Toggle LED to show activity */
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

        /* State machine for binary protocol with sync header */
        switch (rx_state)
        {
            case WAIT_SYNC1:
                if (uart_rx_byte == 0xAA) {
                    rx_state = WAIT_SYNC2;
                }
                break;

            case WAIT_SYNC2:
                if (uart_rx_byte == 0x55) {
                    /* Sync header complete, read command header */
                    rx_state = READ_HEADER;
                    rx_header_index = 0;
                    rx_payload_index = 0;
                } else {
                    /* Not a valid sync sequence, restart */
                    rx_state = WAIT_SYNC1;
                }
                break;

            case READ_HEADER:
                /* Read 4-byte header: type, length, padding[2] */
                ((uint8_t*)&rx_command)[rx_header_index++] = uart_rx_byte;

                if (rx_header_index >= 4) {
                    /* Header complete, check if there's payload */
                    if (rx_command.command_payload_length > 0) {
                        if (rx_command.command_payload_length > 256) {
                            /* Invalid length, reset */
                            rx_state = WAIT_SYNC1;
                        } else {
                            /* Read payload */
                            rx_state = READ_PAYLOAD;
                            rx_payload_index = 0;
                        }
                    } else {
                        /* No payload (e.g., FREQ_CHANGE command) */
                        command_ready = 1;
                        rx_state = WAIT_SYNC1;
                    }
                }
                break;

            case READ_PAYLOAD:
                /* Read payload bytes */
                rx_command.command_payload[rx_payload_index++] = uart_rx_byte;

                if (rx_payload_index >= rx_command.command_payload_length) {
                    /* Complete command received */
                    command_ready = 1;
                    rx_state = WAIT_SYNC1;
                }
                break;

            default:
                rx_state = WAIT_SYNC1;
                break;
        }

        /* Re-enable async reception for next byte */
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    GPIO_Init();
    DAC_PrecomputeMasks();  /* Precompute DAC masks for fast writes */

    USART2_Init(); /* debug */
    USART6_Init(); /* DRA */
    USART1_Init(); /* UART async mode */
    TIM3_Init();   /* sample timer */

    /* init afsk */
    afsk_Init();

    Debug_Print("\r\n=== BeliefSat OrbitRadio-5 APRS MODEM v4 ===\r\n");
    Debug_Print("Binary Protocol with Sync Header (0xAA 0x55)\r\n");

    /* Print clock info for debugging */
    Debug_PrintClocks();

    DRA_Init();

    /* Start asynchronous UART reception */
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);

    Debug_Print("UART listening for binary commands (PA9/PA10)...\r\n");

    /* main loop: check for command flag, process and transmit */
    for (;;)
    {
        /* Check if complete command is ready */
        if (command_ready == 1)
        {
            command_ready = 0;  /* Clear flag immediately */

            /* Make a local copy of the command for processing */
            OrbitAid_Command cmd_copy;
            memcpy(&cmd_copy, (void*)&rx_command, sizeof(OrbitAid_Command));

            /* Process the received command */
            ProcessCommand(&cmd_copy);
        }

        /* Main loop can do other tasks here if needed */
        HAL_Delay(10);  /* Small delay to prevent CPU hogging */
    }
}

/* Process received command based on type */
static void ProcessCommand(OrbitAid_Command *cmd)
{
    char dbg[100];

    snprintf(dbg, sizeof(dbg), "\r\n>> Command RX: Type=0x%02X, Length=%u\r\n",
             cmd->command_type, cmd->command_payload_length);
    Debug_Print(dbg);

    switch (cmd->command_type)
    {
        case CMD_TYPE_FREQ_CHANGE:
            Debug_Print(">> FREQ_CHANGE command received\r\n");

            /* Toggle frequency */
            use_backup_freq = !use_backup_freq;

            /* Reconfigure DRA818U immediately */
            DRA_Init();

            snprintf(dbg, sizeof(dbg), ">> Frequency changed to: %s MHz\r\n\r\n",
                     use_backup_freq ? FREQ_BACKUP : FREQ_DEFAULT);
            Debug_Print(dbg);
            break;

        case CMD_TYPE_TELEMETRY:
            Debug_Print(">> TELEMETRY command received\r\n");

            /* Null-terminate payload for safety */
            cmd->command_payload[cmd->command_payload_length] = '\0';

            snprintf(dbg, sizeof(dbg), ">> Payload: %s\r\n", cmd->command_payload);
            Debug_Print(dbg);

            /* Transmit via APRS */
            TransmitTelemetry((char*)cmd->command_payload);
            break;

        default:
            snprintf(dbg, sizeof(dbg), ">> Unknown command type: 0x%02X\r\n\r\n",
                     cmd->command_type);
            Debug_Print(dbg);
            break;
    }
}

/* Transmit telemetry data via APRS */
static void TransmitTelemetry(const char *payload_text)
{
    char dbg[80];
    char aprs_payload[256];

    /* Build APRS payload with Data Type Identifier
     * '>' = Status message (most appropriate for telemetry)
     */
    snprintf(aprs_payload, sizeof(aprs_payload), ">%s | Somaiya OrbitRadio-5 73", payload_text);

    /* prepare AX.25 frame */
    ax25_len = 0;
    ax25_encode(ax25_buffer, &ax25_len,
                SRC_CALL, SRC_SSID,
                DST_CALL, DST_SSID,
                PATH1_CALL, PATH1_SSID,
                PATH2_CALL, PATH2_SSID,
                aprs_payload);

    snprintf(dbg, sizeof(dbg), "AX.25 frame: %u bytes (payload: %zu chars)\r\n",
             ax25_len, strlen(aprs_payload));
    Debug_Print(dbg);

    /* Pre-TX delay - system stabilization */
    HAL_Delay(200);

    /* Enable PTT */
    HAL_GPIO_WritePin(PTT_UHF_GPIO_Port, PTT_UHF_Pin, GPIO_PIN_SET);
    Debug_Print("PTT ON\r\n");

    /* TX Delay (TXD) - wait for radio to key up
     * DRA818U typically needs 300-500ms
     */
    HAL_Delay(500);

    /* Generate AFSK bit stream from AX.25 frame */
    afsk_generate(ax25_buffer, ax25_len);

    /* Debug: show bit count */
    snprintf(dbg, sizeof(dbg), "AFSK bits queued: %lu\r\n", afsk_getBitsRemaining());
    Debug_Print(dbg);

    /* Start transmission */
    afsk_start();
    Debug_Print("TX started...\r\n");

    /* Wait for transmission to complete */
    uint32_t start_time = HAL_GetTick();
    uint32_t timeout = start_time + 15000;  /* 15 second max */

    while (afsk_isBusy()) {
        if (HAL_GetTick() > timeout) {
            Debug_Print("TX timeout!\r\n");
            break;
        }
    }

    uint32_t tx_time = HAL_GetTick() - start_time;
    snprintf(dbg, sizeof(dbg), "TX complete: %lu ms\r\n", tx_time);
    Debug_Print(dbg);

    /* Post-TX delay before releasing PTT */
    HAL_Delay(100);

    /* Stop AFSK and release PTT */
    afsk_stop();
    HAL_GPIO_WritePin(PTT_UHF_GPIO_Port, PTT_UHF_Pin, GPIO_PIN_RESET);
    Debug_Print("PTT OFF\r\n\r\n");
}

/* System Clock config: use HSI 16 MHz, no PLL */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
}

void Debug_PrintClocks(void)
{
    char buf[100];
    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

    uint32_t tim_clk = pclk1;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        tim_clk = pclk1 * 2;
    }

    snprintf(buf, sizeof(buf), "SYSCLK: %lu Hz\r\n", sysclk);
    Debug_Print(buf);
    snprintf(buf, sizeof(buf), "HCLK: %lu Hz\r\n", hclk);
    Debug_Print(buf);
    snprintf(buf, sizeof(buf), "PCLK1: %lu Hz, TIM3 clk: %lu Hz\r\n", pclk1, tim_clk);
    Debug_Print(buf);

    uint32_t period = TIM3->ARR + 1;
    uint32_t actual_rate = tim_clk / period;
    snprintf(buf, sizeof(buf), "TIM3 ARR: %lu, Sample rate: %lu Hz\r\n", TIM3->ARR, actual_rate);
    Debug_Print(buf);
}

/* GPIO init */
void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    /* DAC bits - Configure with HIGH speed for clean waveforms
     * PA15 = LSB (bit 0)
     * PA1  = BIT_1 (bit 1)
     * PA4  = BIT_2 (bit 2)
     * PA6  = MSB (bit 3)
     */
    g.Pin = LSB_Pin | BIT_1_Pin | BIT_2_Pin | MSB_Pin;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    /* Initialize DAC to mid-level (7 = 0b0111) */
    HAL_GPIO_WritePin(LSB_GPIO_Port, LSB_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIT_1_GPIO_Port, BIT_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIT_2_GPIO_Port, BIT_2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MSB_GPIO_Port, MSB_Pin, GPIO_PIN_RESET);

    /* PTT (PC9) */
    g.Pin = PTT_UHF_Pin;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PTT_UHF_GPIO_Port, &g);
    HAL_GPIO_WritePin(PTT_UHF_GPIO_Port, PTT_UHF_Pin, GPIO_PIN_RESET);

    /* LD2 (PA5) */
    g.Pin = LD2_Pin;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &g);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

/* USART2 debug init (PA2 TX, PA3 RX) */
void USART2_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin = USART_TX_Pin | USART_RX_Pin;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(USART_TX_GPIO_Port, &g);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

/* USART1 async mode (PA9 TX, PA10 RX) - INTERRUPT DRIVEN */
void USART1_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    /* Configure PA9 (TX) and PA10 (RX) for USART1 */
    g.Pin = USART1_TX_Pin | USART1_RX_Pin;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &g);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    /* Enable UART1 interrupt in NVIC */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* USART6 for DRA818U (PC6 TX, PC7 RX) */
void USART6_Init(void)
{
    __HAL_RCC_USART6_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin = USART6_TX_Pin | USART6_RX_Pin;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(USART6_TX_GPIO_Port, &g);

    huart6.Instance = USART6;
    huart6.Init.BaudRate = 9600;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart6);
}

/* TIM3 init: 9600 Hz sample rate for AFSK1200 */
void TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance = TIM3;

    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t tim_clk = pclk1;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        tim_clk = pclk1 * 2;
    }

    /* Calculate period for 9600 Hz */
    uint32_t period = tim_clk / 9600;
    if (period < 1) period = 1;

    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = period - 1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim3);

    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

    HAL_TIM_Base_Start_IT(&htim3);
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);  /* Highest priority */
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* Debug print */
static void Debug_Print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

/* DRA818U helpers */
void DRA_Send(const char *s)
{
    HAL_UART_Transmit(&huart6, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

void DRA_Init(void)
{
    char cmd_buf[80];
    const char *freq = use_backup_freq ? FREQ_BACKUP : FREQ_DEFAULT;

    Debug_Print("Configuring DRA818U...\r\n");
    HAL_Delay(500);

    DRA_Send("AT+DMOCONNECT");
    HAL_Delay(300);

    /* Set frequency based on flag */
    snprintf(cmd_buf, sizeof(cmd_buf), "AT+DMOSETGROUP=0,%s,%s,0000,0,0000", freq, freq);
    DRA_Send(cmd_buf);
    HAL_Delay(300);

    DRA_Send("AT+DMOSETVOLUME=6");
    HAL_Delay(200);

    /* Print current frequency */
    snprintf(cmd_buf, sizeof(cmd_buf), "DRA818U @ %s MHz ready (Mode: %s)\r\n",
             freq, use_backup_freq ? "BACKUP" : "DEFAULT");
    Debug_Print(cmd_buf);
}

/* Error handler */
void Error_Handler(void)
{
    while (1) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(200);
    }
}

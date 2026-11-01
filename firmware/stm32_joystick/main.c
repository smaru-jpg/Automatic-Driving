/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Dual Joystick Controller (Drive + Camera) with Menu
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>  // abs() 함수 사용을 위해 필수
#include <stdbool.h> // true, false 사용
#include "i2c_lcd.h" // I2C LCD 라이브러리
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 동작 모드 정의 (상태 머신)
typedef enum {
    MODE_MENU,   // 메뉴 선택 중
    MODE_DRIVE   // 주행 및 카메라 제어 중
} SystemMode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_I2C_ADDRESS (0x27) // LCD 주소 (0x27 또는0xF)

// 조이스틱 임계값 (메뉴 선택용 단순 비교)
#define JOYSTICK_HIGH_THRESHOLD 3200
#define JOYSTICK_LOW_THRESHOLD  800

// 🚨 [중요] 스위치 핀 정의 (본인 회로에 맞게 수정 필요)
// 예: 외부 버튼을 PA4에 연결했다면 GPIOA, GPIO_PIN_4로 변경
#define BTN_PORT B1_GPIO_Port
#define BTN_PIN  B1_Pin
/* USER CODE END PD

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t huart2_rx_data;
uint8_t huart5_rx_data;

// [중요] ADC 값 저장 배열 (4채널: 주행X, 주행Y, 캠X, 캠Y)
uint32_t ADC_VAL[4];

I2C_LCD_HandleTypeDef lcd;

char lcd_buf[20];
char last_drv_msg[30] = ""; // 주행 메시지 중복 방지
char last_cam_msg[30] = ""; // 카메라 메시지 중복 방지

// 상태 관리 변수들
SystemMode current_mode = MODE_MENU; // 초기 상태: 메뉴
int menu_cursor = 0;                 // 0: 1번 메뉴, 1: 2번 메뉴
bool btn_pressed_flag = false;       // 버튼 디바운싱용 플래그
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 🚗 1. 주행 조이스틱 처리 함수 (비례 제어)
void Process_Drive_Mode(uint32_t x, uint32_t y) {
    char send_protocol[30];
    char lcd_line[20];

    // 1. 값 변환 (0~4095 -> -100~+100)
    // Y축: 속도 (Speed), X축: 조향 (Turn)
    int speed = ((int)y - 2048) / 20;
    int turn  = ((int)x - 2048) / 20;

    // 2. 값 제한 (Clamping)
    if (speed > 100) speed = 100; if (speed < -100) speed = -100;
    if (turn > 100) turn = 100;   if (turn < -100) turn = -100;

    // 3. 데드존(Deadzone) 처리 (중앙 떨림 방지)
    if (abs(speed) < 10) speed = 0;
    if (abs(turn)  < 10) turn = 0;

    // 4. 프로토콜 생성 (예: "DRV:0,50")
    sprintf(send_protocol, "DRV:%d,%d", turn, speed);

    // 5. LCD 표시용 문자열
    sprintf(lcd_line, "D T:%-3d S:%-3d", turn, speed);

    // 6. 데이터 전송 (값이 변했을 때만)
    if (strcmp(send_protocol, last_drv_msg) != 0) {
        // UART 전송
        HAL_UART_Transmit(&huart2, (uint8_t*)send_protocol, strlen(send_protocol), 100);
        HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100); // 줄바꿈
        HAL_UART_Transmit(&huart5, (uint8_t*)send_protocol, strlen(send_protocol), 100);
        HAL_UART_Transmit(&huart5, (uint8_t*)"\r\n", 2, 100);

        // LCD 갱신 (2번째 줄)
        lcd_gotoxy(&lcd, 0, 1);
        lcd_puts(&lcd, lcd_line);

        strcpy(last_drv_msg, send_protocol);
    }

/ 📷 2. 카메라 조이스틱 처리 함수 (비례 제어)
void Process_Camera_Mode(uint32_t x, uint32_t y) {
    char send_protocol[30];

    // 1. 값 변환 (0~4095 -> -100~+100)
    int pan  = ((int)x - 2048) / 20; // 좌우
    int tilt = ((int)y - 2048) / 20; // 상하

    // 2. 값 제한
    if (pan > 100) pan = 100;   if (pan < -100) pan = -100;
    if (tilt > 100) tilt = 100; if (tilt < -100) tilt = -100;

    // 3. 데드존 처리
    if (abs(pan) < 10) pan = 0;
    if (abs(tilt) < 10) tilt = 0;

    // 4. 프로토콜 생성 (예: "CAM:10,-20")
    sprintf(send_protocol, "CAM:%d,%d", pan, tilt);

    // 5. 데이터 전송 (값이 변했을 때만)
    if (strcmp(send_protocol, last_cam_msg) != 0) {
        HAL_UART_Transmit(&huart2, (uint8_t)send_protocol, strlen(send_protocol), 100);
        HAL_UART_Transmit(&huart2, (uint8_t)"\r\n", 2, 100);
        HAL_UART_Transmit(&huart5, (uint8_t*)send_protocol, strlen(send_protocol), 100);
        HAL_UART_Transmit(&huart5, (uint8_t*)"\r\n", 2, 100);

        // 카메라는 LCD에 표시 안 함 (공간 부족)
        strcpy(last_cam_msg, send_protocol);
    }
}

// 📋 3. 메뉴 처리 함수
void Process_Menu_Mode(uint32_t y) {
    static int last_cursor = -1;

    // 조이스틱 위/아래로 커서 이동
    if (y > JOYSTICK_HIGH_THRESHOLD) menu_cursor = 0; // 위
    else if (y < JOYSTICK_LOW_THRESHOLD) menu_cursor = 1; // 아래

    if (menu_cursor != last_cursor) {
        lcd_clear(&lcd);
        if (menu_cursor == 0) {
            lcd_gotoxy(&lcd, 0, 0); lcd_puts(&lcd, "> 1. DRIVE & CAM");
            lcd_gotoxy(&lcd, 0, 1); lcd_puts(&lcd, "  2. SETTING");
        } else {
            lcd_gotoxy(&lcd, 0, 0); lcd_puts(&lcd, "  1. DRIVE & CAM");
            lcd_gotoxy(&lcd, 0, 1); lcd_puts(&lcd, "> 2. SETTING");
        }
        last_cursor = menu_cursor;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  MX_I2C1_Init();
  MX_ADC2_Init();

  /* USER CODE BEGIN 2 */
  // 1. ADC DMA 시작 (4개 채널 값을 ADC_VAL 배열에 계속 저장)
  HAL_ADC_Start_DMA(&hadc2, ADC_VAL, 4);

  // 2. UART 수신 인터럽트 활성화
  HAL_UART_Receive_IT(&huart2, &huart2_rx_data, 1);
  HAL_UART_Receive_IT(&huart5, &huart5_rx_data, 1);

  // 3. LCD 초기화
  lcd.hi2c = &hi2c1;
  lcd.address = (LCD_I2C_ADDRESS << 1);
  lcd_init(&lcd);

  lcd_gotoxy(&lcd, 0, 0);
  lcd_puts(&lcd, "System Init...");
  HAL_Delay(1000);

  // 초기 상태 설정
  current_mode = MODE_MENU;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // --- 1. 버튼 입력 처리 (모드 전환) ---
    if (HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == GPIO_PIN_RESET) {
        if (btn_pressed_flag == false) { // 눌린 순간
            btn_pressed_flag = true;

            if (current_mode == MODE_MENU) {
                if (menu_cursor == 0) { // 1번 메뉴 선택 시 주행 모드로
                    current_mode = MODE_DRIVE;
                    lcd_clear(&lcd);
                    lcd_gotoxy(&lcd, 0, 0);
                    lcd_puts(&lcd, "[DRIVE MODE]");
                    // 상태 초기화 (바로 전송되도록)
                    strcpy(last_drv_msg, "");
                    strcpy(last_cam_msg, "");
                }
            }
            else if (current_mode == MODE_DRIVE) {
                // 주행 모드에서 누르면 메뉴로 복귀
                current_mode = MODE_MENU;
                lcd_clear(&lcd);
            }
        }
    } else {
        btn_pressed_flag = false; // 버튼 떼면 리셋
    }

    // --- 2. 모드별 동작 ---
    switch (current_mode) {
        case MODE_MENU:
            // 메뉴에서는 주행용 조이스틱 Y축(Index 1)만 사용하여 이동
            Process_Menu_Mode(ADC_VAL[1]);
            break;

        case MODE_DRIVE:
            // 주행 조이스틱 (Index 0, 1)
            Process_Drive_Mode(ADC_VAL[0], ADC_VAL[1]);

            // 카메라 조이스틱 (Index 2, 3)
            Process_Camera_Mode(ADC_VAL[2], ADC_VAL[3]);
            break;
    }

    HAL_Delay(50); // 루프 주기
  }
  /* USER CODE END 3 */
}

/**



























































































































































 3 */}
/**  * @brief System Clock Configuration  * @retval None  */void SystemClock_Config(void){  RCC_OscInitTypeDef RCC_OscInitStruct = {0};  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;  RCC_OscInitStruct.HSIState = RCC_HSI_ON;  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;  RCC_OscInitStruct.PLL.PLLM = 16;  RCC_OscInitStruct.PLL.PLLN = 336;  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;  RCC_OscInitStruct.PLL.PLLQ = 2;  RCC_OscInitStruct.PLL.PLLR = 2;  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)  {    Error_Handler();  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)  {    Error_Handler();  }}
static void MX_ADC2_Init(void){  ADC_ChannelConfTypeDef sConfig = {0};
  hadc2.Instance = ADC2;  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;  hadc2.Init.Resolution = ADC_RESOLUTION_12B;  hadc2.Init.ScanConvMode = ENABLE;  hadc2.Init.ContinuousConvMode = ENABLE;  hadc2.Init.DiscontinuousConvMode = DISABLE;  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;  hadc2.Init.NbrOfConversion = 4; // 🚨 여기 4 확인!  hadc2.Init.DMAContinuousRequests = ENABLE;  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;  if (HAL_ADC_Init(&hadc2) != HAL_OK)  {    Error_Handler();  }
  // Rank 1: 주행 X  sConfig.Channel = ADC_CHANNEL_0;  sConfig.Rank = 1;  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();
  // Rank 2: 주행 Y  sConfig.Channel = ADC_CHANNEL_1;  sConfig.Rank = 2;  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();
  // Rank 3: 카메라 X  sConfig.Channel = ADC_CHANNEL_6; // 🚨 채널 번호 확인 필요  sConfig.Rank = 3;  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();
  // Rank 4: 카메라 Y  sConfig.Channel = ADC_CHANNEL_7; // 🚨 채널 번호 확인 필요  sConfig.Rank = 4;  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();}
static void MX_I2C1_Init(void){  hi2c1.Instance = I2C1;  hi2c1.Init.ClockSpeed = 100000;  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;  hi2c1.Init.OwnAddress1 = 0;  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;  hi2c1.Init.OwnAddress2 = 0;  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();}
static void MX_UART5_Init(void){  huart5.Instance = UART5;  huart5.Init.BaudRate = 38400;  huart5.Init.WordLength = UART_WORDLENGTH_8B;  huart5.Init.StopBits = UART_STOPBITS_1;  huart5.Init.Parity = UART_PARITY_NONE;  huart5.Init.Mode = UART_MODE_TX_RX;  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;  huart5.Init.OverSampling = UART_OVERSAMPLING_16;  if (HAL_UART_Init(&huart5) != HAL_OK) Error_Handler();}
static void MX_USART2_UART_Init(void){  huart2.Instance = USART2;  huart2.Init.BaudRate = 38400; // PC 통신용  huart2.Init.WordLength = UART_WORDLENGTH_8B;  huart2.Init.StopBits = UART_STOPBITS_1;  huart2.Init.Parity = UART_PARITY_NONE;  huart2.Init.Mode = UART_MODE_TX_RX;  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;  huart2.Init.OverSampling = UART_OVERSAMPLING_16;  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();}
static void MX_DMA_Init(void){  __HAL_RCC_DMA2_CLK_ENABLE();  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);}
static void MX_GPIO_Init(void){  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();  __HAL_RCC_GPIOH_CLK_ENABLE();  __HAL_RCC_GPIOA_CLK_ENABLE();  __HAL_RCC_GPIOD_CLK_ENABLE();  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  // 버튼 핀 설정 (외부 버튼 사용시 적절히 수정)  GPIO_InitStruct.Pin = B1_Pin;  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  GPIO_InitStruct.Pull = GPIO_PULLUP;  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LD2_Pin;  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  GPIO_InitStruct.Pull = GPIO_PULLUP;  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);}
/* USER CODE BEGIN 4 */// UART 중계 기능 (필요시 활성화)void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){  if (huart->Instance == USART2) {    HAL_UART_Transmit(&huart5, &huart2_rx_data, 1, 10);    HAL_UART_Receive_IT(&huart2, &huart2_rx_data, 1);  }  else if (huart->Instance == UART5) {    HAL_UART_Transmit(&huart2, &huart5_rx_data, 1, 10);    HAL_UART_Receive_IT(&huart5, &huart5_rx_data, 1);  }}/* USER CODE END 4 */
void Error_Handler(void){  __disable_irq();  while (1) {}}
#ifdef USE_FULL_ASSERTvoid assert_failed(uint8_t *file, uint32_t line) {}#endif


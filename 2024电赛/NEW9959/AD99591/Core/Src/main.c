/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author         : Jinhao Zhang
  * @date           : 2025-07-08
  * @version        : 2.0(增加同频显示功能)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ad9959.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // For abs()
#include "AD9834.h"
#include "arm_math.h"
// AD9959的SPI通信是通过ad9959.c/h库处理的，无需直接访问SPI句柄
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_SAMPLES         2048    // FFT sample points, must be power of 2
#define SAMPLING_RATE       8000000  // ADC sampling rate (Hz), based on TIM2 config: 128MHz/(8*1) = 16MHz/2 = 8MHz
#define ADC_BUF_SIZE        FFT_SAMPLES

// Frequency search range
#define MIN_FREQ_HZ         5000    // Min frequency 5kHz
#define MAX_FREQ_HZ         150000  // Max frequency 150kHz

// 5kHz step (as required)
#define FREQ_STEP_5K        5000    // 5kHz step

// Amplitude control
#define MIN_OUTPUT_VPP      1.0f    // Min output peak-to-peak 1V
#define DDS_FULL_SCALE_VPP  0.53f   // AD9959 full scale output ~530mVpp

// Frequency detection precision control
#define PEAK_DISTANCE       5       // Min distance between peaks (FFT points)
#define PI                  3.14159265358979f

// Wave type definitions - using different names to avoid conflict with AD9834.h
#define WAVE_SINE           0       // Sine wave
#define WAVE_TRIANGLE       1       // Triangle wave

// TJC screen command definitions
#define TJC_END_CMD         0xFF    // Command end marker (3x 0xFF)
#define TJC_PAGE_MAIN       0       // Main page
#define TJC_MAX_BUFFER      100     // Command buffer size

// AD9959 PLL和寄存器定义
#define AD9959_PLL_MULTIPLIER    20      // PLL乘法因子 (4-20)
#define AD9959_REG_CFR           0x00    // 控制功能寄存器
#define AD9959_REG_FR1           0x01    // 功能寄存器1
#define AD9959_REG_FR2           0x02    // 功能寄存器2
#define AD9959_REG_CPOW0         0x0A    // 通道0相位偏移字
#define AD9959_REG_CPOW1         0x0B    // 通道1相位偏移字

// 相位控制定义
#define PHASE_ADJUST_FACTOR      100.0f  // 软件PLL的相位调整系数
#define MAX_PHASE_ERROR          0.05f   // 最大允许相位误差 (5%)
#define PHASE_CHECK_INTERVAL     100     // 相位检查间隔(ms)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// FFT related variables
arm_rfft_fast_instance_f32 fft_inst;
uint16_t adc_buf[ADC_BUF_SIZE];
float32_t fft_input[FFT_SAMPLES];
float32_t fft_output[FFT_SAMPLES*2]; // Complex output needs 2x space
float32_t hanning_window[FFT_SAMPLES];
float32_t magnitude[FFT_SAMPLES/2];  // Magnitude spectrum

// Status flags
volatile uint8_t adc_conv_complete_flag = 0;
volatile uint8_t process_flag = 0;

// Detected frequencies and amplitudes
float32_t detected_freqs[2] = {0.0f, 0.0f};
float32_t detected_mags[2] = {0.0f, 0.0f};
uint8_t wave_types[2] = {WAVE_SINE, WAVE_SINE}; // Default to sine wave

// UART control
uint8_t uart_rx_buffer[1];

// TJC screen communication
UART_HandleTypeDef* tjc_uart = &huart3;  // TJC screen connected to USART3
uint8_t tjc_buffer[TJC_MAX_BUFFER];      // TJC command buffer

// 相位跟踪变量
float32_t current_phase_error = 0.0f;
uint16_t phase_adjustment = 0;
volatile uint8_t pll_locked = 0;
uint32_t last_phase_check_time = 0;

// 定时器句柄
TIM_HandleTypeDef htim3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Generate_Hanning_Window(void);
void Process_ADC_Data(void);
void Detect_Wave_Type(void);
void Set_Output_Frequencies(float32_t freq1, float32_t freq2, uint8_t type1, uint8_t type2);
void Set_Output_Amplitudes(void);
float32_t Find_Nearest_Multiple(float32_t freq, float32_t step);

// TJC screen functions
void TJC_SendCmd(const char* cmd);
void TJC_Init(void);
void TJC_UpdateSignalInfo(float32_t freq1, float32_t freq2, uint8_t type1, uint8_t type2);
void TJC_UpdateFFTData(void);

// AD9959 PLL和同步函数
void AD9959_ConfigurePLL(uint8_t multiplier);
void AD9959_WriteRegister(uint8_t reg, uint8_t *data, uint8_t length);
void AD9959_ResetAllPhases(void);
void AD9959_SynchronizeChannels(void);
void AD9959_EnableSyncMode(void);

// STM32软件PLL函数
void ConfigureSoftwarePLL(void);
/**
  * @brief  测量通道间相位误差 - 使用零交叉检测方法
  * @return 测量的相位误差值 (-0.5 ~ +0.5 表示 -180° ~ +180°)
  */
float32_t MeasurePhaseError(void);
void AdjustPhaseBasedOnPLL(float32_t phaseError);
void StartPhaseTracking(void);
void CheckAndAdjustPhase(void);
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
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // Initialize AD9959
  ad9959_init();
  
  // 配置AD9959内部PLL
  AD9959_ConfigurePLL(AD9959_PLL_MULTIPLIER);
  
  // 启用AD9959同步模式以获得稳定的相位关系
  AD9959_EnableSyncMode();
  
  // 重置所有相位以保持一致
  AD9959_ResetAllPhases();
  
  // Set initial amplitude
  ad9959_write_amplitude(AD9959_CHANNEL_0, 1023); // Set max amplitude
  ad9959_write_amplitude(AD9959_CHANNEL_1, 1023);

  // Generate Hanning window
  Generate_Hanning_Window();

  // Initialize FFT
  arm_rfft_fast_init_f32(&fft_inst, FFT_SAMPLES);

  // 配置软件PLL
  ConfigureSoftwarePLL();

  // Enable UART receive interrupt
  HAL_UART_Receive_IT(&huart3, uart_rx_buffer, 1);

  // Start TIM2 to trigger ADC sampling
  HAL_TIM_Base_Start(&htim2);
  
  // Start ADC1 DMA transfer
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_SIZE);

  // Initialize TJC screen
  TJC_Init();

  // 存储当前时间用于相位检查
  last_phase_check_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (adc_conv_complete_flag && process_flag) {
        adc_conv_complete_flag = 0; // Clear flag
        Process_ADC_Data();         // Process collected data
        Detect_Wave_Type();         // Detect wave type
        
        // Output 5kHz multiple frequencies
        float32_t output_freq1 = Find_Nearest_Multiple(detected_freqs[0], FREQ_STEP_5K);
        float32_t output_freq2 = Find_Nearest_Multiple(detected_freqs[1], FREQ_STEP_5K);
        
        // Set output frequencies and amplitudes
        Set_Output_Frequencies(output_freq1, output_freq2, wave_types[0], wave_types[1]);
        Set_Output_Amplitudes();
        
        // Update TJC screen display
        TJC_UpdateSignalInfo(output_freq1, output_freq2, wave_types[0], wave_types[1]);
        TJC_UpdateFFTData();
        
        process_flag = 0;  // Processing complete, wait for next trigger
    }
    
    if (adc_conv_complete_flag && !process_flag) {
        // If no processing flag, continue data collection
        adc_conv_complete_flag = 0;
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_SIZE);
    }
    
    // 定期检查并调整相位
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_phase_check_time >= PHASE_CHECK_INTERVAL) {
        CheckAndAdjustPhase();
        last_phase_check_time = current_time;
    }
    
    HAL_Delay(10); // Short delay to avoid high CPU usage
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
  RCC_OscInitStruct.PLL.PLLN = 128;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Generate Hanning window coefficients
  */
void Generate_Hanning_Window(void)
{
    for (int i = 0; i < FFT_SAMPLES; i++) {
        // Hanning window formula: w(n) = 0.5 * (1 - cos(2π*n/(N-1)))
        hanning_window[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (FFT_SAMPLES - 1)));
    }
}

/**
  * @brief  Process ADC data: window, FFT, find peaks
  */
void Process_ADC_Data(void)
{
    // 1. Data preprocessing and windowing
    for (int i = 0; i < FFT_SAMPLES; i++) {
        // Convert 12-bit ADC data (0-4095) to -1.0 to 1.0 float range, apply Hanning window
        fft_input[i] = ((float32_t)adc_buf[i] - 2048.0f) / 2048.0f * hanning_window[i];
    }

    // 2. Perform FFT
    arm_rfft_fast_f32(&fft_inst, fft_input, fft_output, 0);

    // 3. Calculate FFT magnitude spectrum
    arm_cmplx_mag_f32(fft_output, magnitude, FFT_SAMPLES/2);

    // 4. Find two largest peaks in specified frequency range
    float32_t max_val1 = 0.0f, max_val2 = 0.0f;
    uint32_t max_idx1 = 0, max_idx2 = 0;
    
    // Calculate FFT indices corresponding to frequency range
    uint32_t min_idx = (uint32_t)(MIN_FREQ_HZ / (SAMPLING_RATE / (float32_t)FFT_SAMPLES));
    uint32_t max_idx = (uint32_t)(MAX_FREQ_HZ / (SAMPLING_RATE / (float32_t)FFT_SAMPLES));

    // First pass to find largest peak
    for (uint32_t i = min_idx; i <= max_idx; i++) {
        if (magnitude[i] > max_val1) {
            max_val1 = magnitude[i];
            max_idx1 = i;
        }
    }

    // Second pass to find second largest peak
    // Ignore points near the first peak to avoid detecting the same wide peak as two frequencies
    for (uint32_t i = min_idx; i <= max_idx; i++) {
        if (abs((int)i - (int)max_idx1) > PEAK_DISTANCE) { // Avoid searching near main peak
            if (magnitude[i] > max_val2) {
                max_val2 = magnitude[i];
                max_idx2 = i;
            }
        }
    }

    // 5. Calculate frequencies and amplitudes
    detected_freqs[0] = max_idx1 * (SAMPLING_RATE / (float32_t)FFT_SAMPLES);
    detected_freqs[1] = max_idx2 * (SAMPLING_RATE / (float32_t)FFT_SAMPLES);
    detected_mags[0] = max_val1;
    detected_mags[1] = max_val2;

    // Ensure freq1 is the lower frequency
    if (detected_freqs[0] > detected_freqs[1]) {
        float32_t temp_freq = detected_freqs[0];
        detected_freqs[0] = detected_freqs[1];
        detected_freqs[1] = temp_freq;
        
        float32_t temp_mag = detected_mags[0];
        detected_mags[0] = detected_mags[1];
        detected_mags[1] = temp_mag;
    }
}

/**
  * @brief  Detect wave type (by analyzing FFT results)
  */
void Detect_Wave_Type(void)
{
    // High precision wave detection algorithm: analyze harmonic ratios and phase relationships
    // Triangle wave: 1) Only odd harmonics 2) Harmonic amplitude decreases as 1/n² 3) Odd harmonics have alternating phases
    // Sine wave: Only fundamental, almost no harmonics
    
    // Detect first frequency wave type
    uint32_t fund_idx1 = (uint32_t)(detected_freqs[0] / (SAMPLING_RATE / (float32_t)FFT_SAMPLES));
    uint32_t harm3_idx1 = fund_idx1 * 3; // 3rd harmonic
    uint32_t harm5_idx1 = fund_idx1 * 5; // 5th harmonic
    uint32_t harm2_idx1 = fund_idx1 * 2; // 2nd harmonic (should be small for triangle)
    
    // Calculate harmonic ratios
    float32_t fund_mag1 = magnitude[fund_idx1];
    float32_t harm3_ratio1 = 0.0f;
    float32_t harm5_ratio1 = 0.0f;
    float32_t harm2_ratio1 = 0.0f;
    
    if (harm3_idx1 < FFT_SAMPLES/2) harm3_ratio1 = magnitude[harm3_idx1] / fund_mag1;
    if (harm5_idx1 < FFT_SAMPLES/2) harm5_ratio1 = magnitude[harm5_idx1] / fund_mag1;
    if (harm2_idx1 < FFT_SAMPLES/2) harm2_ratio1 = magnitude[harm2_idx1] / fund_mag1;
    
    // Theoretically, triangle wave 3rd harmonic is ~1/9 of fundamental, 5th is ~1/25
    // Also, triangle wave has no even harmonics
    if (harm3_ratio1 > 0.08f && harm3_ratio1 < 0.15f && 
        harm5_ratio1 > 0.03f && harm5_ratio1 < 0.06f &&
        harm2_ratio1 < 0.03f) {
        wave_types[0] = WAVE_TRIANGLE;
    } else {
        wave_types[0] = WAVE_SINE;
    }
    
    // Detect second frequency wave type
    uint32_t fund_idx2 = (uint32_t)(detected_freqs[1] / (SAMPLING_RATE / (float32_t)FFT_SAMPLES));
    uint32_t harm3_idx2 = fund_idx2 * 3; // 3rd harmonic
    uint32_t harm5_idx2 = fund_idx2 * 5; // 5th harmonic
    uint32_t harm2_idx2 = fund_idx2 * 2; // 2nd harmonic (should be small for triangle)
    
    // Calculate harmonic ratios
    float32_t fund_mag2 = magnitude[fund_idx2];
    float32_t harm3_ratio2 = 0.0f;
    float32_t harm5_ratio2 = 0.0f;
    float32_t harm2_ratio2 = 0.0f;
    
    if (harm3_idx2 < FFT_SAMPLES/2) harm3_ratio2 = magnitude[harm3_idx2] / fund_mag2;
    if (harm5_idx2 < FFT_SAMPLES/2) harm5_ratio2 = magnitude[harm5_idx2] / fund_mag2;
    if (harm2_idx2 < FFT_SAMPLES/2) harm2_ratio2 = magnitude[harm2_idx2] / fund_mag2;
    
    // Use same criteria
    if (harm3_ratio2 > 0.08f && harm3_ratio2 < 0.15f && 
        harm5_ratio2 > 0.03f && harm5_ratio2 < 0.06f &&
        harm2_ratio2 < 0.03f) {
        wave_types[1] = WAVE_TRIANGLE;
    } else {
        wave_types[1] = WAVE_SINE;
    }
    
    // Additional wave confirmation: check time domain characteristics
    // Time domain analysis code could be added here, e.g., zero-crossing rate, crest factor
}

/**
  * @brief  Set output frequencies, select different chips based on wave type
  */
void Set_Output_Frequencies(float32_t freq1, float32_t freq2, uint8_t type1, uint8_t type2)
{
    // 输出第一个信号(信号A - 用作触发源)
    if (type1 == WAVE_TRIANGLE) {
        // 使用AD9834生成三角波
        AD9834_Init();
        AD9834_SetFrequency(FREQ_REG_0, (uint32_t)freq1, TRIANGLE_WAVE);
        // 重置相位确保稳定触发
        AD9834_Reset();
        AD9834_EnableOutput();
    } else {
        // 使用AD9959生成正弦波 - 通道0作为主控通道
        ad9959_write_frequency(AD9959_CHANNEL_0, (uint32_t)freq1);
        // 重置通道0相位以确保稳定触发
        ad9959_reset_phase(AD9959_CHANNEL_0);
    }
    
    // 短暂延时用于同步
    HAL_Delay(2);
    
    // 输出第二个信号(信号B)
    if (type2 == WAVE_TRIANGLE) {
        // 使用AD9834生成三角波
        AD9834_Init();
        AD9834_SetFrequency(FREQ_REG_0, (uint32_t)freq2, TRIANGLE_WAVE);
    } else {
        // 使用AD9959生成正弦波
        ad9959_write_frequency(AD9959_CHANNEL_1, (uint32_t)freq2);
        // 为同步显示维持与通道0的相位关系
        ad9959_reset_phase(AD9959_CHANNEL_1);
    }
    
    // 同步所有通道 - 确保相位对齐
    AD9959_SynchronizeChannels();
    
    // 如果两个信号都使用AD9959，启动相位跟踪
    if (type1 == WAVE_SINE && type2 == WAVE_SINE) {
        StartPhaseTracking();
    }
}

/**
  * @brief  Set AD9959 output amplitude, ensure peak-to-peak value >= 1V
  */
void Set_Output_Amplitudes(void)
{
    // Calculate amplitude value to ensure output >= 1V peak-to-peak
    // DDS_FULL_SCALE_VPP is AD9959 full scale output
    // 1023 is AD9959 amplitude register max value
    
    uint16_t amp_value = (uint16_t)(MIN_OUTPUT_VPP / DDS_FULL_SCALE_VPP * 1023.0f);
    
    // Limit to valid range
    if (amp_value > 1023) amp_value = 1023;
    
    ad9959_write_amplitude(AD9959_CHANNEL_0, amp_value);
    ad9959_write_amplitude(AD9959_CHANNEL_1, amp_value);
    
}

/**
  * @brief  Find nearest multiple of step frequency
  * @param  freq: Input frequency
  * @param  step: Step value
  * @return Nearest multiple of step frequency
  */
float32_t Find_Nearest_Multiple(float32_t freq, float32_t step)
{
    int32_t multiple = (int32_t)roundf(freq / step);
    if (multiple < 1) multiple = 1; // At least 1x
    return multiple * step;
}

/**
  * @brief  UART receive callback
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3) {
        // Set processing flag on any character received
        process_flag = 1;
        
        // Re-enable receive interrupt
        HAL_UART_Receive_IT(&huart3, uart_rx_buffer, 1);
    }
}

/**
  * @brief  ADC conversion complete callback (DMA mode)
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        adc_conv_complete_flag = 1;
        // Stop ADC until data processing is complete
        HAL_ADC_Stop_DMA(&hadc1);
    }
}

/**
  * @brief  Send command to TJC screen
  * @param  cmd: Command string
  */
void TJC_SendCmd(const char* cmd)
{
    uint16_t len = strlen(cmd);
    HAL_UART_Transmit(tjc_uart, (uint8_t*)cmd, len, 100);
    
    // Send end markers (3x 0xFF)
    uint8_t end_bytes[3] = {TJC_END_CMD, TJC_END_CMD, TJC_END_CMD};
    HAL_UART_Transmit(tjc_uart, end_bytes, 3, 100);
}

/**
  * @brief  Initialize TJC screen
  */
void TJC_Init(void)
{
    HAL_Delay(100); // Wait for screen to start
    
    // Switch to main page
    TJC_SendCmd("page main");
    HAL_Delay(50);
    
    // Set initial values for display areas
    TJC_SendCmd("t0.txt=\"Waiting for signal...\"");
    TJC_SendCmd("t1.txt=\"Waiting for signal...\"");
    
    // Clear waveform display
    TJC_SendCmd("cle 1,0");  // Clear waveform area 1
    TJC_SendCmd("cle 2,0");  // Clear waveform area 2
    
    // Add status area for PLL lock indication
    TJC_SendCmd("t3.txt=\"PLL: Unlocked\"");
    
    printf("TJC screen initialized\r\n");
}

/**
  * @brief  Update signal info to TJC screen
  */
void TJC_UpdateSignalInfo(float32_t freq1, float32_t freq2, uint8_t type1, uint8_t type2)
{
    char cmd_buffer[TJC_MAX_BUFFER];
    
    // Update signal A info
    if (type1 == WAVE_TRIANGLE) {
        sprintf(cmd_buffer, "t0.txt=\"Signal A: %.1f Hz (Triangle)\"", freq1);
    } else {
        sprintf(cmd_buffer, "t0.txt=\"Signal A: %.1f Hz (Sine)\"", freq1);
    }
    TJC_SendCmd(cmd_buffer);
    
    // Update signal B info
    if (type2 == WAVE_TRIANGLE) {
        sprintf(cmd_buffer, "t1.txt=\"Signal B: %.1f Hz (Triangle)\"", freq2);
    } else {
        sprintf(cmd_buffer, "t1.txt=\"Signal B: %.1f Hz (Sine)\"", freq2);
    }
    TJC_SendCmd(cmd_buffer);
    
    // Update output status info
    sprintf(cmd_buffer, "t2.txt=\"Output: Normal (Vpp>=1V)\"");
    TJC_SendCmd(cmd_buffer);
    
    // Update PLL status
    if (pll_locked && type1 == WAVE_SINE && type2 == WAVE_SINE) {
        sprintf(cmd_buffer, "t3.txt=\"PLL: Locked (%.3f)\"", current_phase_error);
    } else {
        sprintf(cmd_buffer, "t3.txt=\"PLL: Adjusting\"");
    }
    TJC_SendCmd(cmd_buffer);
}

/**
  * @brief  Update FFT data to TJC screen
  */
void TJC_UpdateFFTData(void)
{
    char cmd_buffer[TJC_MAX_BUFFER];
    int i, j;
    int display_points = 100; // Screen display points
    int step = FFT_SAMPLES / (2 * display_points);
    
    // Clear existing waveform
    TJC_SendCmd("cle 1,0");
    
    // Draw FFT spectrum
    // Find max value for normalization
    float32_t max_mag = 0.1f; // Avoid division by zero
    for (i = 0; i < FFT_SAMPLES/2; i++) {
        if (magnitude[i] > max_mag) {
            max_mag = magnitude[i];
        }
    }
    
    // Add waveform points using addt command
    for (i = 0, j = 0; i < FFT_SAMPLES/2 && j < display_points; i += step, j++) {
        // Normalize to 0-255 range
        int y_val = (int)(magnitude[i] / max_mag * 200.0f);
        if (y_val > 255) y_val = 255;
        
        // Add point to waveform
        sprintf(cmd_buffer, "add 1,0,%d,%d", j, 255 - y_val); // Invert Y axis
        TJC_SendCmd(cmd_buffer);
    }
    
    // Mark detected frequency positions
    uint32_t freq1_idx = (uint32_t)(detected_freqs[0] / (SAMPLING_RATE / (float32_t)FFT_SAMPLES));
    uint32_t freq2_idx = (uint32_t)(detected_freqs[1] / (SAMPLING_RATE / (float32_t)FFT_SAMPLES));
    
    int x1 = freq1_idx / step;
    int x2 = freq2_idx / step;
    
    if (x1 < display_points) {
      sprintf(cmd_buffer, "line %d,50,%d,250,RED", x1, x1);
      TJC_SendCmd(cmd_buffer);
    }
    if (x2 < display_points) {
      sprintf(cmd_buffer, "line %d,50,%d,250,BLUE", x2, x2);
      TJC_SendCmd(cmd_buffer);
    }
}

/**
  * @brief  配置AD9959内部PLL
  * @param  multiplier: PLL乘法因子 (4-20)
  */
void AD9959_ConfigurePLL(uint8_t multiplier)
{
    // 将乘法因子限制在有效范围内 (4-20)
    if (multiplier < 4) multiplier = 4;
    if (multiplier > 20) multiplier = 20;
    
    // 读取当前CFR寄存器值
    uint8_t cfr_data[3] = {0}; // CFR为3字节
    
    // 设置PLL乘法因子并使能PLL
    // CFR[2]的位7-3为乘法因子值
    // CFR[2]的位2为PLL使能位
    cfr_data[2] = (multiplier << 3) | (1 << 2);
    
    // 写入AD9959 CFR寄存器
    AD9959_WriteRegister(AD9959_REG_CFR, cfr_data, 3);
    
    // 等待PLL锁定
    HAL_Delay(10);
    
    // 重置DDS核心以确保使用PLL
    ad9959_reset();
}

/**
  * @brief  写入AD9959寄存器
  * @param  reg: 寄存器地址
  * @param  data: 要写入的数据指针
  * @param  length: 数据长度
  */
void AD9959_WriteRegister(uint8_t reg, uint8_t *data, uint8_t length)
{
    // 使用ad9959库中的函数进行寄存器操作
    // 直接调用ad9959_write_data，无需构造buffer
    ad9959_write_data((AD9959_REG_ADDR)reg, length, data, 1);
    
    // 注：参数1=寄存器地址，参数2=数据长度，参数3=数据指针，参数4=是否更新IO(1=是)
}

/**
  * @brief  重置所有AD9959通道的相位
  */
void AD9959_ResetAllPhases(void)
{
    // 重置所有通道的相位
    ad9959_reset_phase(AD9959_CHANNEL_0);
    ad9959_reset_phase(AD9959_CHANNEL_1);
    ad9959_reset_phase(AD9959_CHANNEL_2);
    ad9959_reset_phase(AD9959_CHANNEL_3);
}

/**
  * @brief  同步所有AD9959通道
  */
void AD9959_SynchronizeChannels(void)
{
    // 先重置所有通道相位
    AD9959_ResetAllPhases();
    
    // 同时重置DDS核心以同步
    ad9959_reset();
    
    // 等待同步稳定
    HAL_Delay(1);
}

/**
  * @brief  启用AD9959同步模式
  */
void AD9959_EnableSyncMode(void)
{
    // 配置FR1寄存器启用同步功能
    uint8_t fr1_data[3] = {0};
    
    // 设置同步时钟使能位
    fr1_data[0] = 0x80;  // 同步时钟使能(MSyncEn)位在FR1[0]的bit7
    
    // 写入FR1寄存器
    AD9959_WriteRegister(AD9959_REG_FR1, fr1_data, 3);
    
    // 应用设置
    ad9959_io_update(); // 而不是ad9959_update();
}

/**
  * @brief  配置STM32定时器用于软件PLL
  */
void ConfigureSoftwarePLL(void)
{
    // 启用TIM3时钟
    __HAL_RCC_TIM3_CLK_ENABLE();
    
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 1599; // 64MHz / 1600 = 40kHz
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 3999;    // 40kHz / 4000 = 10Hz定时中断
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim3);
    
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
    
    // 使能TIM3中断
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    
    // 启动定时器
    HAL_TIM_Base_Start_IT(&htim3);
}

/**
  * @brief  测量通道间相位误差 - 使用零交叉检测方法
  * @return 测量的相位误差值 (-0.5 ~ +0.5 表示 -180° ~ +180°)
  */
float32_t MeasurePhaseError(void)
{
    #define ZC_BUF_SIZE 64      // 零交叉检测缓冲区大小
    #define ZC_THRESHOLD 100    // 零交叉阈值
    
    static uint16_t signal_a[ZC_BUF_SIZE]; // 信号A采样缓冲
    static uint16_t signal_b[ZC_BUF_SIZE]; // 信号B采样缓冲
    static uint32_t adc_values[2];         // ADC转换值[0]=通道A, [1]=通道B
    
    // 1. 采集两个通道的信号数据
    for(uint8_t i = 0; i < ZC_BUF_SIZE; i++) {
        // 配置ADC2采样通道0（连接到信号A）
        ADC_ChannelConfTypeDef sConfig = {0};
        sConfig.Channel = ADC_CHANNEL_0;
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
        HAL_ADC_ConfigChannel(&hadc2, &sConfig);
        
        // 启动ADC2转换并等待完成
        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2, 10);
        signal_a[i] = HAL_ADC_GetValue(&hadc2);
        HAL_ADC_Stop(&hadc2);
        
        // 配置ADC2采样通道1（连接到信号B）
        sConfig.Channel = ADC_CHANNEL_1;
        HAL_ADC_ConfigChannel(&hadc2, &sConfig);
        
        // 启动ADC2转换并等待完成
        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2, 10);
        signal_b[i] = HAL_ADC_GetValue(&hadc2);
        HAL_ADC_Stop(&hadc2);
        
        // 延时确保采样率一致
        __NOP(); __NOP(); __NOP(); __NOP();
    }
    
    // 2. 查找信号A的上升过零点
    int32_t zc_index_a = -1;
    for(uint8_t i = 1; i < ZC_BUF_SIZE; i++) {
        // 检测上升过零（使用中点2048作为零点参考）
        if(signal_a[i-1] < 2048 && signal_a[i] >= 2048) {
            zc_index_a = i;
            break;
        }
    }
    
    // 3. 在信号A过零点附近查找信号B的上升过零点
    int32_t zc_index_b = -1;
    int32_t search_start = (zc_index_a > ZC_BUF_SIZE/4) ? (zc_index_a - ZC_BUF_SIZE/4) : 0;
    int32_t search_end = (zc_index_a + ZC_BUF_SIZE/4 < ZC_BUF_SIZE) ? (zc_index_a + ZC_BUF_SIZE/4) : ZC_BUF_SIZE-1;
    
    for(int32_t i = search_start+1; i <= search_end; i++) {
        if(signal_b[i-1] < 2048 && signal_b[i] >= 2048) {
            zc_index_b = i;
            break;
        }
    }
    
    // 4. 如果找到两个有效的过零点，计算相位差
    if(zc_index_a >= 0 && zc_index_b >= 0) {
        // 使用线性插值提高过零点精度
        float32_t precise_zc_a = zc_index_a - 1.0f + 
                                (2048.0f - (float32_t)signal_a[zc_index_a-1]) / 
                                ((float32_t)signal_a[zc_index_a] - (float32_t)signal_a[zc_index_a-1]);
        
        float32_t precise_zc_b = zc_index_b - 1.0f + 
                                (2048.0f - (float32_t)signal_b[zc_index_b-1]) / 
                                ((float32_t)signal_b[zc_index_b] - (float32_t)signal_b[zc_index_b-1]);
        
        // 计算相位差（归一化到-0.5~+0.5，相当于-180°~+180°）
        float32_t phase_diff = (precise_zc_b - precise_zc_a) / (float32_t)ZC_BUF_SIZE;
        
        // 处理相位环绕（如果相位差大于半个周期，则减去一个周期）
        if(phase_diff > 0.5f) phase_diff -= 1.0f;
        if(phase_diff < -0.5f) phase_diff += 1.0f;
        
        return phase_diff;
    }
    
    // 如果没有找到有效的过零点，使用相关性方法
    if(zc_index_a < 0 || zc_index_b < 0) {
        // 计算信号的相关性来确定相位
        float32_t correlation = 0.0f;
        float32_t max_correlation = 0.0f;
        int32_t max_shift = 0;
        
        // 去除DC偏置
        float32_t signal_a_float[ZC_BUF_SIZE];
        float32_t signal_b_float[ZC_BUF_SIZE];
        for(uint8_t i = 0; i < ZC_BUF_SIZE; i++) {
            signal_a_float[i] = (float32_t)signal_a[i] - 2048.0f;
            signal_b_float[i] = (float32_t)signal_b[i] - 2048.0f;
        }
        
        // 对于每个可能的移位，计算相关性
        for(int32_t shift = -ZC_BUF_SIZE/4; shift <= ZC_BUF_SIZE/4; shift++) {
            correlation = 0.0f;
            for(uint8_t i = 0; i < ZC_BUF_SIZE; i++) {
                int32_t j = i + shift;
                if(j >= 0 && j < ZC_BUF_SIZE) {
                    correlation += signal_a_float[i] * signal_b_float[j];
                }
            }
            
            if(correlation > max_correlation) {
                max_correlation = correlation;
                max_shift = shift;
            }
        }
        
        // 计算相位差
        float32_t phase_diff = (float32_t)max_shift / (float32_t)ZC_BUF_SIZE;
        
        // 返回相位误差
        return phase_diff;
    }
    
    // 如果所有方法都失败，返回上次的相位误差
    return current_phase_error;
}

/**
  * @brief  基于测量的误差调整相位
  * @param  phaseError: 测量的相位误差
  */
void AdjustPhaseBasedOnPLL(float32_t phaseError)
{
    // 实现PI控制器进行相位调整
    static float32_t integral_error = 0.0f;
    float32_t proportional = 500.0f; // 比例增益
    float32_t integral = 50.0f;      // 积分增益
    
    // 更新积分项并限制防止积分饱和
    integral_error += phaseError;
    if (integral_error > 0.2f) integral_error = 0.2f;
    if (integral_error < -0.2f) integral_error = -0.2f;
    
    // 计算相位调整量
    phase_adjustment = (uint16_t)((phaseError * proportional + integral_error * integral) * PHASE_ADJUST_FACTOR);
    
    // 将相位调整应用到通道1
    ad9959_write_phase(AD9959_CHANNEL_1, phase_adjustment);
    
    // 检查PLL是否锁定(相位误差较小)
    if (fabsf(phaseError) < MAX_PHASE_ERROR) {
        pll_locked = 1;
    } else {
        pll_locked = 0;
    }
}

/**
  * @brief  开始相位跟踪过程
  */
void StartPhaseTracking(void)
{
    // 重置相位跟踪变量
    current_phase_error = 0.0f;
    phase_adjustment = 0;
    pll_locked = 0;
    
    // 初始相位测量
    current_phase_error = MeasurePhaseError();
    
    // 初始相位调整
    AdjustPhaseBasedOnPLL(current_phase_error);
}

/**
  * @brief  检查并调整相位
  */
void CheckAndAdjustPhase(void)
{
    // 仅在两个通道都有效时执行调整
    if (detected_freqs[0] > 0 && detected_freqs[1] > 0 &&
        wave_types[0] == WAVE_SINE && wave_types[1] == WAVE_SINE) {
        
        // 测量当前相位误差
        current_phase_error = MeasurePhaseError();
        
        // 基于相位误差调整
        AdjustPhaseBasedOnPLL(current_phase_error);
        
        // 可以在TJC屏幕上更新锁定状态
        char cmd_buffer[TJC_MAX_BUFFER];
        if (pll_locked) {
            sprintf(cmd_buffer, "t3.txt=\"PLL: Locked (%.3f)\"", current_phase_error);
        } else {
            sprintf(cmd_buffer, "t3.txt=\"PLL: Adjusting (%.3f)\"", current_phase_error);
        }
        TJC_SendCmd(cmd_buffer);
    }
}

/**
  * @brief  定时器中断处理函数
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        // 以定时器中断频率进行相位检查
        if (detected_freqs[0] > 0 && detected_freqs[1] > 0 && 
            wave_types[0] == WAVE_SINE && wave_types[1] == WAVE_SINE) {
            // 仅在需要时执行快速相位调整
            if (!pll_locked) {
                current_phase_error = MeasurePhaseError();
                AdjustPhaseBasedOnPLL(current_phase_error);
            }
        }
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
  while (1)
  {
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

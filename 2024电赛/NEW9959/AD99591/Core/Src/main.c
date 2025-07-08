/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author         : Jinhao Zhang
  * @date           : 2025-07-08
  * @version        : 1.0(没有同频显示)
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
  ad9959_write_amplitude(AD9959_CHANNEL_0, 1023); // Set max amplitude
  ad9959_write_amplitude(AD9959_CHANNEL_1, 1023);

  // Generate Hanning window
  Generate_Hanning_Window();

  // Initialize FFT
  arm_rfft_fast_init_f32(&fft_inst, FFT_SAMPLES);

  // Enable UART receive interrupt
  HAL_UART_Receive_IT(&huart3, uart_rx_buffer, 1);

  // Start TIM2 to trigger ADC sampling
  HAL_TIM_Base_Start(&htim2);
  
  // Start ADC1 DMA transfer
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_SIZE);

  // Initialize TJC screen
  TJC_Init();

  // Send initialization complete message
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
        
        process_flag = 0;  // Processing complete, wait for next trigger=
    }
    
    if (adc_conv_complete_flag && !process_flag) {
        // If no processing flag, continue data collection
        adc_conv_complete_flag = 0;
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_SIZE);
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
    // Output first signal
    if (type1 == WAVE_TRIANGLE) {
        // Use AD9834 for triangle wave
        AD9834_Init();
        AD9834_SetFrequency(FREQ_REG_0, (uint32_t)freq1, TRIANGLE_WAVE);
    } else {
        // Use AD9959 for sine wave
        ad9959_write_frequency(AD9959_CHANNEL_0, (uint32_t)freq1);
    }
    
    // Output second signal
    if (type2 == WAVE_TRIANGLE) {
        // Use AD9834 for triangle wave
        AD9834_Init();
        AD9834_SetFrequency(FREQ_REG_0, (uint32_t)freq2, TRIANGLE_WAVE);
    } else {
        // Use AD9959 for sine wave
        ad9959_write_frequency(AD9959_CHANNEL_1, (uint32_t)freq2);
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

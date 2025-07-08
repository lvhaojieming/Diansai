/* USER CODE BEGIN Includes */
#include "ad9959.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // For abs()
#include "AD9834.h"
#include "arm_math.h"
// AD9959的SPI通信是通过ad9959.c/h库处理的，无需直接访问SPI句柄

// 自定义的相位重置函数，因为ad9959库中没有
void ad9959_reset_phase(uint8_t channel)
{
    // 直接调用写相位函数，将相位设为0
    ad9959_write_phase((AD9959_CHANNEL)channel, 0);
}

// 声明函数，但实现放到下面
void ad9959_reset_phase(uint8_t channel);
void AD9959_WriteRegister(uint8_t reg, uint8_t *data, uint8_t length);
/* USER CODE END Includes */ 

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
 * @brief 重置AD9959通道相位为0
 * @param channel 目标通道
 */
void ad9959_reset_phase(uint8_t channel)
{
    // 直接调用写相位函数，将相位设为0
    ad9959_write_phase((AD9959_CHANNEL)channel, 0);
}

/**
 * @brief 写入AD9959寄存器
 * @param reg 寄存器地址
 * @param data 要写入的数据指针
 * @param length 数据长度
 */
void AD9959_WriteRegister(uint8_t reg, uint8_t *data, uint8_t length)
{
    ad9959_select(); // 选择AD9959芯片
    
    // 发送寄存器地址
    uint8_t instruction = reg;
    HAL_SPI_Transmit(&hspi1, &instruction, 1, 100);
    
    // 发送数据
    HAL_SPI_Transmit(&hspi1, data, length, 100);
    
    ad9959_deselect(); // 取消选择AD9959芯片
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
    
    // 使用IO更新函数
    ad9959_io_update();
}

/* USER CODE END 4 */ 
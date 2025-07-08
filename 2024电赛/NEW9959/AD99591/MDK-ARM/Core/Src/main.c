/* USER CODE BEGIN Includes */
#include "ad9959.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // For abs()
#include "AD9834.h"
#include "arm_math.h"
// AD9959的SPI通信是通过ad9959.c/h库处理的，无需直接访问SPI句柄

// 声明函数，但实现放到下面
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
void ad9959_reset_phase(AD9959_CHANNEL channel)
{
    // 直接调用写相位函数，将相位设为0
    ad9959_write_phase(channel, 0);
}

/**
 * @brief 写入AD9959寄存器
 * @param reg 寄存器地址
 * @param data 要写入的数据指针
 * @param length 数据长度
 */
void AD9959_WriteRegister(uint8_t reg, uint8_t *data, uint8_t length)
{
    // 使用AD9959库提供的写数据函数，而不是直接使用SPI
    ad9959_write_data((AD9959_REG_ADDR)reg, length, data, 1);
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
void AD9959_ResetUsedPhases(void)
{
    // 只重置实际使用的通道的相位
    ad9959_reset_phase(AD9959_CHANNEL_0);
    ad9959_reset_phase(AD9959_CHANNEL_1);
}

/**
  * @brief  同步所有AD9959通道
  */
void AD9959_SynchronizeChannels(void)
{
    // 先重置使用的通道相位
    AD9959_ResetUsedPhases();
    
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

/**
  * @brief  Send multiple commands to TJC screen in one transmission
  * @param  cmds: Array of command strings
  * @param  count: Number of commands
  */
void TJC_SendBatchCmd(const char* cmds[], uint8_t count)
{
    static uint8_t buffer[TJC_MAX_BUFFER]; // 静态缓冲区避免频繁栈分配
    uint16_t offset = 0;
    
    // 合并所有指令
    for (uint8_t i = 0; i < count; i++) {
        uint16_t len = strlen(cmds[i]);
        if (offset + len + 1 >= TJC_MAX_BUFFER) break; // 防止缓冲区溢出
        
        // 复制命令到缓冲区
        memcpy(&buffer[offset], cmds[i], len);
        offset += len;
        
        // 添加命令分隔符
        buffer[offset++] = ';';
    }
    
    // 发送合并后的指令
    HAL_UART_Transmit(tjc_uart, buffer, offset, 100);
    
    // 只在最后发送一次结束符
    static const uint8_t end_bytes[3] = {TJC_END_CMD, TJC_END_CMD, TJC_END_CMD};
    HAL_UART_Transmit(tjc_uart, end_bytes, 3, 100);
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
    static const uint8_t end_bytes[3] = {TJC_END_CMD, TJC_END_CMD, TJC_END_CMD};
    HAL_UART_Transmit(tjc_uart, end_bytes, 3, 100);
}

/**
  * @brief  Initialize TJC screen
  */
void TJC_Init(void)
{
    HAL_Delay(100); // Wait for screen to start
    
    // 使用批量发送命令，减少串口通信次数
    const char* cmds[3] = {
        "page main",
        "t0.txt=\"A: Wait...\"",
        "t1.txt=\"B: Wait...\"",
    };
    TJC_SendBatchCmd(cmds, 3);
    
    HAL_Delay(50);
    TJC_SendCmd("t3.txt=\"PLL: --\"");
}

/**
  * @brief  Update signal info to TJC screen
  */
void TJC_UpdateSignalInfo(float32_t freq1, float32_t freq2, uint8_t type1, uint8_t type2)
{
    static char cmd_buffer[TJC_MAX_BUFFER]; // 静态缓冲区避免频繁栈分配
    
    // 使用静态变量缓存上次的值，只在数据变化时更新
    static float32_t last_freq1 = -1.0f;
    static float32_t last_freq2 = -1.0f;
    static uint8_t last_type1 = 0xFF;
    static uint8_t last_type2 = 0xFF;
    static uint8_t last_pll_status = 0xFF;
    
    // 检查频率和波形类型是否变化
    bool update_signal_a = (fabsf(freq1 - last_freq1) > 0.1f) || (type1 != last_type1);
    bool update_signal_b = (fabsf(freq2 - last_freq2) > 0.1f) || (type2 != last_type2);
    bool update_pll = (last_pll_status != pll_locked);
    
    // 准备要发送的命令数组
    const char* cmds[3];
    uint8_t cmd_count = 0;
    
    // 只在需要时更新信号A信息
    if (update_signal_a) {
        sprintf(cmd_buffer, "t0.txt=\"A: %.1fkHz (%s)\"", 
                freq1/1000.0f, 
                (type1 == WAVE_TRIANGLE) ? "Tri" : "Sin");
        cmds[cmd_count++] = cmd_buffer;
        
        // 更新缓存
        last_freq1 = freq1;
        last_type1 = type1;
    }
    
    // 只在需要时更新信号B信息
    if (update_signal_b) {
        // 使用第二个静态缓冲区，避免覆盖第一个命令
        static char cmd_buffer2[TJC_MAX_BUFFER];
        sprintf(cmd_buffer2, "t1.txt=\"B: %.1fkHz (%s)\"", 
                freq2/1000.0f, 
                (type2 == WAVE_TRIANGLE) ? "Tri" : "Sin");
        cmds[cmd_count++] = cmd_buffer2;
        
        // 更新缓存
        last_freq2 = freq2;
        last_type2 = type2;
    }
    
    // 只在PLL状态变化时更新
    if (update_pll) {
        // 使用第三个静态缓冲区
        static char cmd_buffer3[TJC_MAX_BUFFER];
        if (pll_locked && type1 == WAVE_SINE && type2 == WAVE_SINE) {
            sprintf(cmd_buffer3, "t3.txt=\"PLL: OK\"");
        } else {
            sprintf(cmd_buffer3, "t3.txt=\"PLL: --\"");
        }
        cmds[cmd_count++] = cmd_buffer3;
        
        // 更新缓存
        last_pll_status = pll_locked;
    }
    
    // 如果有命令需要发送，则批量发送
    if (cmd_count > 0) {
        TJC_SendBatchCmd(cmds, cmd_count);
    }
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
        
        // PLL状态更新会在TJC_UpdateSignalInfo函数中处理
        // 不需要在这里直接更新屏幕，减少串口通信
    }
}

/* USER CODE END 4 */ 
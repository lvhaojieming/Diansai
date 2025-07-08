/*
 * @Author: Jinhao Zhang 2023213315@bupt.cn
 * @Date: 2025-07-07 19:59:44
 * @LastEditors: Jinhao Zhang 2023213315@bupt.cn
 * @LastEditTime: 2025-07-08 11:46:02
 * @FilePath: \MDK-ARM\AD9834.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __AD9834_H 
#define __AD9834_H 
 
//	#include "stm32f4xx_it.h"
//	#include "stdio.h"
	#include "gpio.h"
 
	//TRIANGLE_WAVE��SINE_WAVE��SQUARE_WAVEĬ��"λ"����
	
	#define TRIANGLE_WAVE	0x2002	//���ǲ�
 
	#define SINE_WAVE		0x2008	//���Ҳ�
 
	#define SQUARE_WAVE		0x2028	//����
 
	#define PIN_SW			0x200	//���ſ���ʹ��λ
	
	/* AD9834����Ƶ��75MHz */ 
	#define AD9834_SYSTEM_CLOCK     75000000UL 
 
	/*
		�����ĳ��Լ����ư�����š����罫AD9834_FSYNC�Ÿĳ�PC2���ƣ�����"#define AD9834_FSYNC GPIO_Pin_2"
		ע�⣡����
		������ɺ����ڡ�void AD9834_Init(void)����ʼ����Ӧ������
	*/
 
	/* AD9834 �������� */ 
	#define AD9834_Control_Port  GPIOA
	#define AD9834_FSYNC				GPIO_PIN_1		//A1	�������ݵ�֡ͬ���ź� �͵�ƽ��Ч��������
	#define AD9834_SCLK 				GPIO_PIN_2		//A2
	#define AD9834_SDATA				GPIO_PIN_3 		//A3
	#define AD9834_RESET				GPIO_PIN_4		//A4
	#define AD9834_FS					GPIO_PIN_5		//A5	Ƶ��ѡ������ ѡ��Ƶ�ʼĴ���FREQ0 �� FREQ1
	#define AD9834_PS					GPIO_PIN_6		//A6    ��λѡ�� ��λ�Ĵ��� PHASE0 �� PHASE1 ���ӵ���λ�ۼ���
 
 
	#define AD9834_FSYNC_SET   HAL_GPIO_WritePin(AD9834_Control_Port ,AD9834_FSYNC ,GPIO_PIN_SET) 
 
	#define AD9834_FSYNC_CLR   HAL_GPIO_WritePin(AD9834_Control_Port ,AD9834_FSYNC ,GPIO_PIN_RESET) 
 
	#define AD9834_SCLK_SET   HAL_GPIO_WritePin(AD9834_Control_Port ,AD9834_SCLK ,GPIO_PIN_SET) 
 
	#define AD9834_SCLK_CLR   HAL_GPIO_WritePin(AD9834_Control_Port ,AD9834_SCLK ,GPIO_PIN_RESET) 
 
	#define AD9834_SDATA_SET   HAL_GPIO_WritePin(AD9834_Control_Port ,AD9834_SDATA ,GPIO_PIN_SET) 
 
	#define AD9834_SDATA_CLR   HAL_GPIO_WritePin(AD9834_Control_Port ,AD9834_SDATA ,GPIO_PIN_RESET) 
 
	#define AD9834_RESET_SET   HAL_GPIO_WritePin(AD9834_Control_Port ,AD9834_RESET ,GPIO_PIN_SET) 
 
	#define AD9834_RESET_CLR   HAL_GPIO_WritePin(AD9834_Control_Port ,AD9834_RESET ,GPIO_PIN_RESET) 
 
	#define AD9834_PS_SET   HAL_GPIO_WritePin(GPIOA ,AD9834_PS ,GPIO_PIN_SET) 
 
	#define AD9834_PS_CLR   HAL_GPIO_WritePin(GPIOA ,AD9834_PS ,GPIO_PIN_RESET) 
	
	#define AD9834_FS_SET   HAL_GPIO_WritePin(GPIOA ,AD9834_FS ,GPIO_PIN_SET) 
 
	#define AD9834_FS_CLR   HAL_GPIO_WritePin(GPIOA ,AD9834_FS ,GPIO_PIN_RESET) 
 
	
	
	
	
	
	#define FREQ_REG_0		0x4000 	//Ƶ�ʼĴ���0
 
	#define FREQ_REG_1		0x8000	//Ƶ�ʼĴ���1
 
	#define PHASE_REG_0		0xC000	//��λ�Ĵ���0
	
	#define PHASE_REG_1		0xE000  //��λ�Ĵ���1
	
	
	
/* AD9834相关函数 */ 
void AD9834_Init(void);//AD9834引脚初始化
 
void AD9834_Write_16Bits(uint32_t data);//向AD9834写入16位数据
 
void AD9834_SetFrequency(uint16_t reg, float fre, uint16_t type);//设置频率值
 
void AD9834_SetPhase(uint16_t reg, uint16_t val);//设置相位值

void AD9834_Reset(void);//重置AD9834芯片

void AD9834_EnableOutput(void);//使能AD9834输出
 
#endif /* AD9834_H */ 

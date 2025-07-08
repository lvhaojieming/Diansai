#include "ad9959.h"


uint8_t FR2_DATA[2] = {0x20, 0x00};//default Value = 0x0000
uint8_t CFR_DATA[3] = {0x00, 0x03, 0x02};//default Value = 0x000302

uint8_t CPOW0_DATA[2] = {0x00, 0x00};//default Value = 0x0000   @ = POW/2^14*360

uint8_t LSRR_DATA[2] = {0x00, 0x00};//default Value = 0x----

uint8_t RDW_DATA[4] = {0x00, 0x00, 0x00, 0x00};//default Value = 0x--------

uint8_t FDW_DATA[4] = {0x00, 0x00, 0x00, 0x00};//default Value = 0x--------

/**
* @brief AD9959��΢��ʱ
* */
static void ad9959_delay(uint32_t length) {
    length = length * 12;
    while (length--);
}

/**
* @brief AD9959��ʼ��
* */
void ad9959_init(void) {
    uint8_t FR1_DATA[3] = {0xD0, 0x00,
                           0x00};//20��Ƶ Charge pump control = 75uA FR1<23> -- VCO gain control =0ʱ system clock below 160 MHz;

    ad9959_io_init();
    ad9959_reset();

    ad9959_write_data(AD9959_REG_FR1, 3, FR1_DATA, 1);//д���ܼĴ���1
    ad9959_write_data(AD9959_REG_FR2, 2, FR2_DATA, 1);

    //д���ʼƵ��
    ad9959_write_frequency(AD9959_CHANNEL_0, 1000);
    ad9959_write_frequency(AD9959_CHANNEL_1, 1000);
    ad9959_write_frequency(AD9959_CHANNEL_2, 1000);
    ad9959_write_frequency(AD9959_CHANNEL_3, 1000);

    ad9959_write_phase(AD9959_CHANNEL_0, 0);
    ad9959_write_phase(AD9959_CHANNEL_1, 0);
    ad9959_write_phase(AD9959_CHANNEL_2, 0);
    ad9959_write_phase(AD9959_CHANNEL_3, 0);

    ad9959_write_amplitude(AD9959_CHANNEL_0, 0xFF);
    ad9959_write_amplitude(AD9959_CHANNEL_1, 0xFF);
    ad9959_write_amplitude(AD9959_CHANNEL_2, 0xFF);
    ad9959_write_amplitude(AD9959_CHANNEL_3, 0xFF);
}

/**
* @brief AD9959��λ
* */
void ad9959_reset(void) {
    AD9959_RESET_0;
    ad9959_delay(1);
    AD9959_RESET_1;
    ad9959_delay(30);
    AD9959_RESET_0;
}

/**
* @brief AD9959IO�ڳ�ʼ��
* */
void ad9959_io_init(void) {
   
    AD9959_PDC_0;
    AD9959_CS_1;
    AD9959_SCLK_0;
    AD9959_UPDATE_0;
    AD9959_PS0_0;
    AD9959_PS1_0;
    AD9959_PS2_0;
   AD9959_PS3_0;
    AD9959_SDIO0_0;
    AD9959_SDIO1_0;
    AD9959_SDIO2_0;
    AD9959_SDIO3_0;
}

/**
 * @brief AD9959����IO�Ĵ���
 * */
void ad9959_io_update(void) {
    AD9959_UPDATE_0;
    ad9959_delay(2);
    AD9959_UPDATE_1;
    ad9959_delay(4);
    AD9959_UPDATE_0;
}

/**
 * @brief ͨ��SPI��AD9959д����
 * @param register_address �Ĵ�����ַ
 * @param number_of_registers �����ֽ���
 * @param register_data ������ʼ��ַ
 * @param update �Ƿ����IO�Ĵ���
 * */
void ad9959_write_data(AD9959_REG_ADDR register_address, uint8_t number_of_registers, const uint8_t *register_data,
                       bool update) {
    uint8_t ControlValue = 0;
    uint8_t ValueToWrite = 0;
    uint8_t RegisterIndex = 0;
    uint8_t i = 0;

    assert_param(IS_AD9959_REG_ADDR(register_address));

    ControlValue = register_address;
//д���ַ
AD9959_SCLK_0;
    AD9959_CS_0;
    for (i = 0; i < 8; i++) {
        AD9959_SCLK_0;
        if (0x80 == (ControlValue & 0x80))
            AD9959_SDIO0_1;
        else
            AD9959_SDIO0_0;
            AD9959_SCLK_1;
        ControlValue <<= 1;
    }
    AD9959_SCLK_0;
//д������
    for (RegisterIndex = 0; RegisterIndex < number_of_registers; RegisterIndex++) {
        ValueToWrite = register_data[RegisterIndex];
        for (i = 0; i < 8; i++) {
            AD9959_SCLK_0;
            if (0x80 == (ValueToWrite & 0x80))
                AD9959_SDIO0_1;
            else
                AD9959_SDIO0_0;
            AD9959_SCLK_1;
            ValueToWrite <<= 1;
        }
        AD9959_SCLK_0;
    }
    if (update) ad9959_io_update();
    AD9959_CS_1;
}

/**
 * @brief ����ͨ�������λ
 * @param channel ���ͨ��
 * @param phase �����λ 14bit �����λ��Χ��0~16383(��Ӧ�Ƕȣ�0��~360��)
 * */
void ad9959_write_phase(AD9959_CHANNEL channel, uint16_t phase) {
    uint8_t cs_data = channel;
    assert_param(IS_AD9959_CHANNEL(channel));
    CPOW0_DATA[1] = (uint8_t) phase;
    CPOW0_DATA[0] = (uint8_t) (phase >> 8);
    ad9959_write_data(AD9959_REG_CSR, 1, &cs_data, 1);
    ad9959_write_data(AD9959_REG_CPOW0, 2, CPOW0_DATA, 1);

}

/**
 * @brief ����ͨ�����Ƶ��
 * @param channel ���ͨ��
 * @param amplitude ���Ƶ�� (Ƶ�ʷ�Χ 1 ~ 200000000Hz)
 * */
void ad9959_write_frequency(AD9959_CHANNEL channel, uint32_t Freq) {
    uint8_t CFTW0_DATA[4] = {0x00, 0x00, 0x00, 0x00};    //�м����
    uint32_t frequency;
    uint8_t cs_data = channel;

    assert_param(IS_AD9959_CHANNEL(channel));

    frequency = (uint32_t) Freq * 8.589934592;       //������Ƶ�����ӷ�Ϊ�ĸ��ֽ�  8.589934592=(2^32)/500000000 ����500M=25M*20(��Ƶ���ɱ��)
    CFTW0_DATA[3] = (uint8_t) frequency;
    CFTW0_DATA[2] = (uint8_t) (frequency >> 8);
    CFTW0_DATA[1] = (uint8_t) (frequency >> 16);
    CFTW0_DATA[0] = (uint8_t) (frequency >> 24);

    ad9959_write_data(AD9959_REG_CSR, 1, &cs_data, 1);
    ad9959_write_data(AD9959_REG_CFTW0, 4, CFTW0_DATA, 1);


}

/**
 * @brief ����ͨ���������
 * @param channel ���ͨ��
 * @param amplitude ������� 10bit �����λ��Χ��0~1023(��Ӧ���ȣ�0 ~ 530mV)
 * */
void ad9959_write_amplitude(AD9959_CHANNEL channel, uint16_t amplitude) {
    uint8_t ACR_DATA[3] = {0x00, 0x00, 0x00};//default Value = 0x--0000 Rest = 18.91/Iout
    uint8_t cs_data = channel;

    assert_param(IS_AD9959_CHANNEL(channel));

    amplitude = amplitude | 0x1000;
    ACR_DATA[2] = (uint8_t) amplitude;
    ACR_DATA[1] = (uint8_t) (amplitude >> 8);

    ad9959_write_data(AD9959_REG_CSR, 1, &cs_data, 1);
    ad9959_write_data(AD9959_REG_ACR, 3, ACR_DATA, 1);

}
/*  if(SWEEP == 1)
    {
      for(uint32_t freq_sw = 1000000; freq_sw < 40000000; freq_sw += 1000000)
    {
        ad9959_write_frequency(AD9959_CHANNEL_3,freq_sw);
        LCD_ShowFloatNum1(20,20,(float)freq_sw/1000000,4,BLACK,WHITE,24);
        HAL_Delay(200);
    }
    SWEEP = 0;
    }
    else
    {
      ;
       //LCD_ShowFloatNum1(10,10,(float)freq/1000000,4,BLACK,WHITE,24);
    }*/
    /* void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_4)
  {
    delay_x();
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == RESET)
    {
      freq = freq + 1000000;
       ad9959_write_frequency(AD9959_CHANNEL_3,(uint32_t)freq);
      LCD_ShowFloatNum1(20,20,(float)freq/1000000,4,BLACK,WHITE,24);
    }
  }
  if(GPIO_Pin == GPIO_PIN_5)
  {
     delay_x();
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5) == RESET)
    {
      freq = freq - 1000000;
      ad9959_write_frequency(AD9959_CHANNEL_3,(uint32_t)freq);
      LCD_ShowFloatNum1(20,20,(float)freq/1000000,4,BLACK,WHITE,24);
    }
  }
  if(GPIO_Pin == GPIO_PIN_6)
  { delay_x();
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6) == RESET)
    {
      SWEEP = 1-SWEEP;
    }
    delay_x();
  }
  
}
void delay_x(void)
{
  for(int i = 0;i<3000000;i++)
  {
    ;
  }
} */

/**
 * @brief 重置通道相位为0
 * @param channel 目标通道
 */
void ad9959_reset_phase(AD9959_CHANNEL channel)
{
    // 直接调用写相位函数，将相位设为0
    ad9959_write_phase(channel, 0);
}

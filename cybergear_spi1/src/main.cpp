/**
 * CYBERGEAR SPI1 (BROKEN)
 * ----------------------
 * SPI1 is connected to a single device, the Infineon 6EDL7141
 * This device has a 24bit word length, and is tricky to use with a gd32 which
 * is more comfortable with 8bit and 16bit words.
 *
 * This example tries to setup SPI using gd32 libs - unfortuantely
 * the second byte reads the same as the first byte - so something is wrong.
 *
 * The sample iterates through the first 10 registers trying to read 2 bytes.
 */

#include <Arduino.h>
#include "SimpleCAN.h"

void setup()
{
  pinMode(PB4, OUTPUT);
  digitalWrite(PB4, HIGH); // this might enable CAN

  RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;
  rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
  spi_parameter_struct spi_init_struct;

  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_GPIOC);
  rcu_periph_clock_enable(RCU_GPIOD);
  rcu_periph_clock_enable(RCU_AF);

  rcu_periph_clock_enable(RCU_SPI1);
  rcu_periph_clock_enable(RCU_SPI2);

  CAN.begin(1000000);

  GPIO_CTL0(GPIOA) = 0x44244422;
  GPIO_CTL1(GPIOA) = 0x38842BBB;
  GPIO_OCTL(GPIOA) = 0x00002003;

  GPIO_CTL0(GPIOB) = 0x22222220;
  GPIO_CTL1(GPIOB) = 0x949342B8;
  GPIO_OCTL(GPIOB) = 0x000015FE;

  GPIO_CTL0(GPIOC) = 0x22440000;
  GPIO_CTL1(GPIOC) = 0x224B4B22;
  GPIO_OCTL(GPIOC) = 0x0000C3C0;

  GPIO_CTL0(GPIOD) = 0x44444244;
  GPIO_CTL1(GPIOD) = 0x44444444;
  GPIO_OCTL(GPIOD) = 0x00000004;

  AFIO_PCF0 = 0x12004000;

  gpio_bit_reset(GPIOB, GPIO_PIN_12); // Start Transaction CS LOW

  spi_struct_para_init(&spi_init_struct);

  spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
  spi_init_struct.device_mode = SPI_MASTER;
  spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
  spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE; // MODE 0

  spi_init_struct.nss = SPI_NSS_SOFT;
  spi_init_struct.prescale = SPI_PSC_16;
  spi_init_struct.endian = SPI_ENDIAN_MSB;
  spi_init(SPI1, &spi_init_struct);
  spi_crc_off(SPI1);
  spi_enable(SPI1);
}

static uint8_t data[8];

uint8_t reg = 0x01;

void loop()
{

  gpio_bit_reset(GPIOB, GPIO_PIN_12); // Start Transaction CS LOW

  while (RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_TBE))
    ;
  spi_i2s_data_transmit(SPI1, reg);

  int count = 0;
  while ((RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_RBNE)) && (count++ < 1000))
    ;

  uint16_t receive1 = spi_i2s_data_receive(SPI1);
  while ((RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_RBNE)) && (count++ < 1000))
    ;

  uint16_t receive2 = spi_i2s_data_receive(SPI1);
  while ((RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_RBNE)) && (count++ < 1000))
    ;

  gpio_bit_set(GPIOB, GPIO_PIN_12); // END Transaction CS HIGH

  data[0] = reg;
  data[1] = receive1;
  data[2] = receive2;

  bool isRtr = false;

  CanMsg txMsg = CanMsg(
      CanExtendedId(0b111111111111110, isRtr),
      3,
      data);
  gpio_bit_set(GPIOB, GPIO_PIN_12); // END Transaction CS HIGH

  CAN.write(txMsg);

  reg += 1;
  if (reg >= 0xA)
  {
    reg = 0x0;
  }

  delay(900);
}

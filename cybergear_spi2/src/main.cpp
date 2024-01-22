#include <Arduino.h>

void setHigh(uint32_t gpio_periph, uint32_t pin) {
  gpio_init(gpio_periph, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, pin); // LED PA5
  gpio_bit_set(GPIOA, pin);
}

void setup()
{

  rcu_periph_clock_enable(RCU_SPI2);
  rcu_periph_clock_enable(RCU_AF);
  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_GPIOC);

  setHigh(GPIOA, GPIO_PIN_0);
  setHigh(GPIOA, GPIO_PIN_1);
  setHigh(GPIOA, GPIO_PIN_11);
  setHigh(GPIOB, GPIO_PIN_1);
  setHigh(GPIOB, GPIO_PIN_2);
  setHigh(GPIOB, GPIO_PIN_3);
  setHigh(GPIOB, GPIO_PIN_4);
  setHigh(GPIOB, GPIO_PIN_5);
  setHigh(GPIOB, GPIO_PIN_6);
  setHigh(GPIOB, GPIO_PIN_7);
  setHigh(GPIOB, GPIO_PIN_10);
  setHigh(GPIOC, GPIO_PIN_6);
  setHigh(GPIOC, GPIO_PIN_7);
  setHigh(GPIOC, GPIO_PIN_8);
  setHigh(GPIOC, GPIO_PIN_9);
  setHigh(GPIOC, GPIO_PIN_14);
  setHigh(GPIOC, GPIO_PIN_15);


  // gpio_pin_remap_config(AFIO_PCF0_SPI2_REMAP)
  // gpio_pin_remap_config(GPIO_SPI2_REMAP, ENABLE);
  
  
  gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5); // LED PA5
  gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15); // CS PA15
  
  gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10); // SCK PC10
  gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11); // MISO PC11
  gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12); // MOSI PC12

  spi_parameter_struct spi_config;
  
  spi_struct_para_init(&spi_config);

  spi_config.device_mode = SPI_MASTER;
  spi_config.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
  spi_config.frame_size = SPI_FRAMESIZE_8BIT;
  spi_config.nss = SPI_NSS_SOFT;
  spi_config.endian = SPI_ENDIAN_MSB;
  // spi_config.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE; // MODE 0
  spi_config.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE; //MODE 1

  spi_config.prescale = SPI_PSC_128;

  // set the CS pin high whilst we configure
  gpio_bit_set(GPIOA, GPIO_PIN_15);

  spi_init(SPI2, &spi_config);

  spi_crc_off(SPI2);
  spi_enable(SPI2);

}

void blinkManyTimes(uint8_t count) {

  for (int i = 1; i <= count; i++) {
    gpio_bit_reset(GPIOA, GPIO_PIN_5);
    delay(200);
    gpio_bit_set(GPIOA, GPIO_PIN_5);
    delay(200);
  }

}
  
uint16_t angle_register = 0x3FFD;
uint16_t angle_register1 = 0x3F;
// uint16_t angle_register1 = 0x01;
uint16_t angle_register2 = 0xFF;


void loop()
{

  gpio_bit_reset(GPIOA, GPIO_PIN_15); // Start Transaction CS LOW
  
  while (RESET == spi_i2s_flag_get(SPI2, SPI_FLAG_TBE));
  spi_i2s_data_transmit(SPI2, angle_register1);
  while (RESET == spi_i2s_flag_get(SPI2, SPI_FLAG_TBE));

  spi_i2s_data_transmit(SPI2, angle_register2);
  uint16_t receive1;
  uint16_t receive2;
   int count = 0;
  while ((RESET == spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE)) && (count++ < 1000));
  
  if (count >= 1000) {
    blinkManyTimes(1);
  }
  receive1 = spi_i2s_data_receive(SPI2);
  while ((RESET == spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE)) && (count++ < 1000));
  receive2 = spi_i2s_data_receive(SPI2);

  if (receive1 == 0 || receive2 == 0) {
    blinkManyTimes(2);
  } else if (receive1 ==1) {
    blinkManyTimes(3);
  } else {
    blinkManyTimes(5);
  }

  delay(10);
  gpio_bit_set(GPIOA, GPIO_PIN_15); // END Transaction CS HIGH

  delay(1000);

}

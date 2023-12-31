#include <Arduino.h>
#include "gd32f30x.h"
#include "gd32f30x_can.h"
#include "gd32_it.h"

#define LED_PIN PB2
#define CAN0_USED

#define USART1_TX PA2
#define USART1_RX PA3

#ifdef CAN0_USED
#define CANX CAN0
#else
#define CANX CAN1
#endif

volatile ErrStatus test_flag;
volatile ErrStatus test_flag_interrupt = ERROR;

void nvic_config(void);
void led_config(void);
// void can_loopback_init(can_parameter_struct can_parameter, can_filter_parameter_struct can_filter);

ErrStatus can_loopback_transmit(bool useInterrupt)
{
  can_trasnmit_message_struct transmit_message;
  uint32_t timeout = 0x0000FFFF;

  if (useInterrupt)
  {
    /* enable CAN receive FIFO0 not empty interrupt  */
    can_interrupt_enable(CANX, CAN_INTEN_RFNEIE0);
  }

  /* initialize transmit message */
  transmit_message.tx_sfid = 0;
  transmit_message.tx_efid = 0x1234;
  transmit_message.tx_ff = CAN_FF_EXTENDED;
  transmit_message.tx_ft = CAN_FT_DATA;
  transmit_message.tx_dlen = 2;
  transmit_message.tx_data[0] = 0xDE;
  transmit_message.tx_data[1] = 0xCA;
  /* transmit a message */
  Serial.println("transmit");
  delay(10);
  can_message_transmit(CANX, &transmit_message);

  // Serial.println("trans done");
  delay(100);
  /* waiting for receive completed */
  while ((test_flag_interrupt != SUCCESS) && (timeout != 0))
  {
    timeout--;
  }
  if (0 == timeout)
  {
    test_flag_interrupt = ERROR;
  }

  if (useInterrupt)
  {
    can_interrupt_disable(CANX, CAN_INTEN_RFNEIE0);
  }
  return test_flag_interrupt;
}

void can_loopback_init(can_parameter_struct can_parameter, can_filter_parameter_struct can_filter)
{
  Serial.println("deinit");

  /* initialize CAN register */
  can_deinit(CANX);

  Serial.println("deinit done");

  delay(10);

  /* initialize CAN */
  can_parameter.time_triggered = DISABLE;
  can_parameter.auto_bus_off_recovery = DISABLE;
  can_parameter.auto_wake_up = DISABLE;
  can_parameter.auto_retrans = ENABLE;
  can_parameter.rec_fifo_overwrite = DISABLE;
  can_parameter.trans_fifo_order = DISABLE;
  can_parameter.working_mode = CAN_LOOPBACK_MODE;
  /* configure baudrate to 125kbps */
  can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
  can_parameter.time_segment_1 = CAN_BT_BS1_3TQ;
  can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;
  can_parameter.prescaler = 96;
  can_init(CANX, &can_parameter);

  Serial.println("init done");
  delay(10);
  /* initialize filter */
#ifdef CAN0_USED
  /* CAN0 filter number */
  can_filter.filter_number = 0;
#else
  /* CAN1 filter number */
  can_filter.filter_number = 15;
#endif
  /* initialize filter */
  can_filter.filter_mode = CAN_FILTERMODE_MASK;
  can_filter.filter_bits = CAN_FILTERBITS_32BIT;
  can_filter.filter_list_high = 0x0000;
  can_filter.filter_list_low = 0x0000;
  can_filter.filter_mask_high = 0x0000;
  can_filter.filter_mask_low = 0x0000;
  can_filter.filter_fifo_number = CAN_FIFO0;
  can_filter.filter_enable = ENABLE;
  can_filter_init(&can_filter);
  Serial.println("filter init done");
  delay(10);
}

void nvic_config(void)
{
#ifdef CAN0_USED
  /* configure CAN0 NVIC */
  nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 0, 0);
#else
  /* configure CAN1 NVIC */
  nvic_irq_enable(CAN1_RX0_IRQn, 0, 0);
#endif
}

void setup()
{
#ifdef CAN0_USED
  rcu_periph_clock_enable(RCU_CAN0);
#else
  rcu_periph_clock_enable(RCU_CAN1);
#endif
  nvic_config();

  Serial.begin(115200);
  delay(4000);
  Serial.println("Starting CAN");
  delay(10);

  pinMode(LED_PIN, OUTPUT);

  can_parameter_struct can_init_parameter;
  can_filter_parameter_struct can_filter_parameter;
  can_loopback_init(can_init_parameter, can_filter_parameter);

  delay(10);
}

void loop()
{
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  Serial.println("loop");
  test_flag_interrupt = can_loopback_transmit(false);

  if (SUCCESS == test_flag_interrupt)
  {
    Serial.println("Success");
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    Serial.println("Fail");

    can_receive_message_struct receive_message;

    can_message_receive(CANX, CAN_FIFO0, &receive_message);
    Serial.println(receive_message.rx_efid);
    Serial.println(receive_message.rx_data[0], HEX);
    digitalWrite(LED_PIN, LOW);
  }

  digitalWrite(LED_PIN, LOW);
  delay(800);
}

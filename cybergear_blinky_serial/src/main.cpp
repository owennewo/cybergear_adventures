#include <Arduino.h>

#define LED_PIN PA5

// static void system_clock_120m_hxtal_16Mhz(void)
// {
//     uint32_t timeout = 0U;
//     uint32_t stab_flag = 0U;

//     /* enable HXTAL */
//     RCU_CTL |= RCU_CTL_HXTALEN;

//     /* wait until HXTAL is stable or the startup time is longer than HXTAL_STARTUP_TIMEOUT */
//     do{
//         timeout++;
//         stab_flag = (RCU_CTL & RCU_CTL_HXTALSTB);
//     }while((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

//     /* if fail */
//     if(0U == (RCU_CTL & RCU_CTL_HXTALSTB)){
//         while(1){
//         }
//     }

//     RCU_APB1EN |= RCU_APB1EN_PMUEN;
//     PMU_CTL |= PMU_CTL_LDOVS;

//     /* HXTAL is stable */
//     /* AHB = SYSCLK */
//     RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
//     /* APB2 = AHB/1 */
//     RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
//     /* APB1 = AHB/2 */
//     RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

//     /* select HXTAL/2 as clock source */
//     RCU_CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PREDV0);
//     RCU_CFG0 |= (RCU_PLLSRC_HXTAL_IRC48M | RCU_CFG0_PREDV0);

//     /* CK_PLL = (CK_HXTAL/2) * 30 = 120 MHz */
//     RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4 | RCU_CFG0_PLLMF_5);
//     // RCU_CFG0 |= RCU_PLL_MUL30;
//     RCU_CFG0 |= RCU_PLL_MUL15;

//     /* enable PLL */
//     RCU_CTL |= RCU_CTL_PLLEN;

//     /* wait until PLL is stable */
//     while(0U == (RCU_CTL & RCU_CTL_PLLSTB)){
//     }

//     /* enable the high-drive to extend the clock frequency to 120 MHz */
//     PMU_CTL |= PMU_CTL_HDEN;
//     while(0U == (PMU_CS & PMU_CS_HDRF)){
//     }

//     /* select the high-drive mode */
//     PMU_CTL |= PMU_CTL_HDS;
//     while(0U == (PMU_CS & PMU_CS_HDSRF)){
//     }

//     /* select PLL as system clock */
//     RCU_CFG0 &= ~RCU_CFG0_SCS;
//     RCU_CFG0 |= RCU_CKSYSSRC_PLL;

//     /* wait until PLL is selected as system clock */
//     while(0U == (RCU_CFG0 & RCU_SCSS_PLL)){
//     }
// }

// HXTAL_VALUE
void setup()
{
  // system_clock_120m_hxtal_16Mhz();
  // SystemCoreClockUpdate();

  Serial.begin(115200);
  delay(3000);
  Serial.print("SysClock Speed: ");
  Serial.print(SystemCoreClock);
  Serial.println(" Hz");
  delay(10);

  pinMode(LED_PIN, OUTPUT);
}

void loop()
{
  // if (SystemCoreClock > 490000000) {
digitalWrite(LED_PIN, LOW);
  delay(200);
  Serial.print("+");
  digitalWrite(LED_PIN, HIGH);
  delay(800);
  Serial.println("-");
  // }
  
}

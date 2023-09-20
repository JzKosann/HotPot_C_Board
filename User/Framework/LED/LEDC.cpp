//
// Created by ShiF on 2023/9/12.
//

#include "LEDC.h"
/* Private includes ----------------------------------------------------------*/
#include "ledio.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

void (*setLedLevel)(LedC led, uint8_t level);

/* Private user code ---------------------------------------------------------*/
void LED_SetLedOn(LedC &led) {
    led.state = LED_ON;
    setLedLevel(led, led.on_level);
}

void LED_SetLedOff(LedC &led) {
    led.state = LED_OFF;
    setLedLevel(led, !led.on_level);
}

void LED_SetLedToggle(LedC &led) {
    led.state = LED_TOGGLE;
    setLedLevel(led, 3);
}

/**
  * @brief  设置led的状态
  * @param  status: LED_OFF, LED_ON, LED_TOGGLE
  * @retval None
  */
void LedC::setStatus(LED_State_e status) {
    this->state = status;
    switch (status) {
        case LED_ON:
            LED_SetLedOn(*this);
            break;
        case LED_OFF:
            LED_SetLedOff(*this);
            break;
        case LED_TOGGLE:
            LED_SetLedToggle(*this);
            break;
    }
}

void LED_Init(void) {
    LEDIO_ConfigInit();
    LedC led(GPIOE, GPIO_PIN_11, LED_ON);   //LED_R PE11
    led.setStatus(LED_TOGGLE);
}


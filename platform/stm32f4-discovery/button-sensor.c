/*
 * Copyright (c) 2013, Jens Nielsen
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL JENS NIELSEN BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "stm32f4_discovery.h"
#include "dev/button-sensor.h"

//#include BOARD_HEADER

//#define DEBOUNCE 1

#if DEBOUNCE
static struct timer debouncetimer;
#endif

#define FALSE 0
#define TRUE  1

uint8_t button_flags = 0;

#define BUTTON_ACTIVE_FLG 0x01
#define BUTTON_PRESSED_FLG 0x02

#define BUTTON_HAS_BEEN_PRESSED() (button_flags & BUTTON_PRESSED_FLG)
#define BUTTON_HAS_BEEN_RELEASED() (!(button_flags & BUTTON_PRESSED_FLG))
#define BUTTON_SET_PRESSED() (button_flags |= BUTTON_PRESSED_FLG)
#define BUTTON_SET_RELEASED() (button_flags &= ~BUTTON_PRESSED_FLG)


GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT };

const uint16_t BUTTON_PIN[BUTTONn] = {USER_BUTTON_PIN };

const uint32_t BUTTON_CLK[BUTTONn] = {USER_BUTTON_GPIO_CLK };

const uint16_t BUTTON_EXTI_LINE[BUTTONn] = {USER_BUTTON_EXTI_LINE };

const uint8_t BUTTON_PORT_SOURCE[BUTTONn] = {USER_BUTTON_EXTI_PORT_SOURCE};

const uint8_t BUTTON_PIN_SOURCE[BUTTONn] = {USER_BUTTON_EXTI_PIN_SOURCE };
const uint8_t BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn };


/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_USER
  * @param  Button_Mode: Specifies Button mode.
  *   This parameter can be one of following parameters:
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability
  * @retval None
  */
void STM_EVAL_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the BUTTON Clock */
  RCC_AHB1PeriphClockCmd(BUTTON_CLK[Button], ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure Button pin as input */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = BUTTON_PIN[Button];
  GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStructure);

  if (Button_Mode == BUTTON_MODE_EXTI)
  {
    /* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(BUTTON_PORT_SOURCE[Button], BUTTON_PIN_SOURCE[Button]);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = BUTTON_EXTI_LINE[Button];
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = BUTTON_IRQn[Button];
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
  }
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_USER
  * @retval The Button GPIO pin value.
  */
uint32_t STM_EVAL_PBGetState(Button_TypeDef Button)
{
  return GPIO_ReadInputDataBit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/*---------------------------------------------------------------------------*/
static void
init(void)
{
  #if DEBOUNCE
  timer_set(&debouncetimer, 0);
  #endif
  
  STM_EVAL_PBInit( BUTTON_USER, BUTTON_MODE_EXTI );
  
}
/*---------------------------------------------------------------------------*/
static void activate(void)
{
  button_flags |= BUTTON_ACTIVE_FLG;
}
/*---------------------------------------------------------------------------*/
static void deactivate(void)
{
  button_flags &= ~BUTTON_ACTIVE_FLG;
}
/*---------------------------------------------------------------------------*/
static int active(void)
{
  return (button_flags & BUTTON_ACTIVE_FLG)? 1 : 0;
}
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  if(!active()){
    return 0;
  }


#if DEBOUNCE
  if(timer_expired(&debouncetimer))
  {
    if(STM_EVAL_PBGetState(BUTTON_USER))
    {
      timer_set(&debouncetimer, CLOCK_SECOND / 10);
      if(BUTTON_HAS_BEEN_RELEASED()){ // Button has been previously released.
        sensors_changed(&button_sensor);
      }
      BUTTON_SET_PRESSED();

      return 1;
    }
    else {
      BUTTON_SET_RELEASED();
      return 0;
    }
  }
  else {
    return 0;
  }
#else
  if ( STM_EVAL_PBGetState(BUTTON_USER) )
  {
    return 1;
  }
  else
  {
    return 0;
  }
#endif

}
/*---------------------------------------------------------------------------*/
static int configure(int type, int value)
{
  switch(type){
    case SENSORS_HW_INIT:
      init();
      return 1;
    case SENSORS_ACTIVE:
      if(value)        
        activate();
      else
        deactivate();
      return 1;
  }
       
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch(type) {
    
    case SENSORS_READY:
      return active();
  }
  
  return 0;
}
/*---------------------------------------------------------------------------*/
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line0);

#if DEBOUNCE
        if(timer_expired(&debouncetimer))
        {
            timer_set(&debouncetimer, CLOCK_SECOND / 5);
            sensors_changed(&button_sensor);
        }
#else
        sensors_changed(&button_sensor);
#endif
    }
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(button_sensor, BUTTON_SENSOR,
	       value, configure, status);


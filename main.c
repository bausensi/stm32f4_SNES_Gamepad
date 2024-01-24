/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usbd_hid_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"

#define CLOCK GPIO_Pin_13
#define LATCH GPIO_Pin_14
#define DATA  GPIO_Pin_15

static void initGPIO(void);
static void initTimer(void);
static void appInit(void);
static uint32_t configUSB(void);

static uint32_t delayTime;

static volatile uint8_t should_poll = 0;

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END; // The USB device
                                                            // 

uint16_t snes_poll(void) {
    uint16_t mask = 0;

    GPIO_SetBits(GPIOC, LATCH);
    USB_OTG_BSP_uDelay(12);
    GPIO_ResetBits(GPIOC, LATCH);

    for(uint32_t i = 0; i < 16; i++) {
        USB_OTG_BSP_uDelay(6);
        GPIO_ResetBits(GPIOC, CLOCK);
    
        mask |= (uint16_t)GPIO_ReadInputDataBit(GPIOC, DATA) << i;

        USB_OTG_BSP_uDelay(6);
        GPIO_SetBits(GPIOC, CLOCK);
    
    }
    return ~mask;
}

int main(void) {
    delayTime = 10;
    appInit();

    while (1) {
        USB_OTG_BSP_uDelay(1000);
        uint16_t mask = snes_poll();

        uint8_t  report[2] = { (uint8_t)( mask & 0xFF ), (uint8_t)( mask >> 8 ) };
        USBD_HID_SendReport (&USB_OTG_dev, &report, 2);
    }

    return 0;
}

void appInit(void){
    initGPIO();
    initTimer();
    configUSB();
}

/******* The interrupt handler ***/
// Read the GPIO in to an 8 bit integer, send the report aka the 8 bit integer

void initGPIO(void){
    // Init the GPIO for the SNES controller
    // First configure the inputs
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
    GPIO_InitTypeDef gpio_snes;
    // Configure the output pin
    gpio_snes.GPIO_Mode = GPIO_Mode_OUT;
    gpio_snes.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    gpio_snes.GPIO_OType = GPIO_OType_PP;
    gpio_snes.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_snes.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &gpio_snes);
    
    gpio_snes.GPIO_Mode = GPIO_Mode_IN;
    gpio_snes.GPIO_Pin = GPIO_Pin_15;
    gpio_snes.GPIO_OType = GPIO_OType_PP;
    gpio_snes.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_snes.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &gpio_snes);
}


// This timer is used when polling input from the GPIO

void initTimer(void){
    /* power on the TIM7 timer 
       (the microcontroller powers off every peripheral by default) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    TIM_TimeBaseInitTypeDef timInitStruct;
    timInitStruct.TIM_CounterMode   = TIM_CounterMode_Up;
    
    /* Don't modify clock frequency, 
     * we do this by setting the prescaler value*/
    timInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    
    /* According to the datasheet of the STM32F407xx
       (DM00037051.pdf, page 29, Table 3. Timer feature comparison)
       TIM7 runs at 84 MHz, that is it increments its value
       84000000 times in a second.
       By setting its prescaler to 1000 it increments only 
       84000 times in a second (1000 times slower).
       By setting its period (auto reload value)
       to 84, we make it increment 1000 times from 0 to 84 in a second.
       That is, it counts every millisecond from 0 to 84, then it interrupts.
       This way we get an interrupt every millisecond.
     */
    timInitStruct.TIM_Prescaler     = 1000;
    timInitStruct.TIM_Period        = 84 * 2;

    /* store this data into memory */
    TIM_TimeBaseInit(TIM7, &timInitStruct);
    /* enable the TIM7 interrupt source 
       this is not the same as enabling the actual interrupt routine,
       which we setup in init_interrupt(),
       it just enables TIM7 as interrupt source,
       that is it makes TIM7 emit interrupt signals
       when it overflows the TIM_Period value
     */
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    
    /* actually start the timer */
    TIM_Cmd(TIM7, ENABLE);
    
    /* From this point on, TIM7 runs and emits and interrupt signal
       every millisecond. Next step is to setup NVIC to actually
       catch the interrupt signal and branch to TIM7_IRQHandler. */
}


// This function enbles the interrupt generated from the timer..

void initInterrupt(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* which interupt number */
    NVIC_InitStructure.NVIC_IRQChannel    = TIM7_IRQn;
    /* enable or disable? */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* lowest priority (highest is 0, lowest if 0x0F) */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0F;
    
    /* Store this data into NVIC memory map
       This function, which is declared in misc.h,
       computes a few values and stores them at the address NVIC, 
       which is #defined in core_cm4.h.
       NVIC is just a memory address casted to an NVIC
       configurations structure (NVIC_Type). */
    NVIC_Init(&NVIC_InitStructure);
    
    /* From this point on and having setup TIM7 in init_timer, 
       the interrupt handler TIM7_IRQHandler is being called 
       every millisecond. */
}

// The set up method for USB!
static uint32_t configUSB(void)
{
  USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc, 
            &USBD_HID_cb, 
            &USR_cb);  
  return 0;
}



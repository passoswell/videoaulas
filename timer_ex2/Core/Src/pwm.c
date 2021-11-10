#include <stddef.h>
#include "pwm.h"
//#include "sys_cfg_stm32f407.h"
#include "macros.h"
#include "stm32f4xx.h"



/*****************************************************************************************************************/
/*                                               TIMER MACROS                                                    */
/*****************************************************************************************************************/
/** @defgroup CLK Enable
 * @{
 */
#define TIMER1_CLK_ENABLE()   SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN)
#define TIMER8_CLK_ENABLE()   SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN)

/* These Timers have 4 channels */
#define TIMER2_CLK_ENABLE()   SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN)
#define TIMER3_CLK_ENABLE()   SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN)
#define TIMER4_CLK_ENABLE()   SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN)
#define TIMER5_CLK_ENABLE()   SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN)

/* These Timers have 2 independent 16 bit channels for PWM */
#define TIMER9_CLK_ENABLE()   SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM9EN)
#define TIMER12_CLK_ENABLE()  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM12EN)

#define TIMER10_CLK_ENABLE()  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM10EN)
#define TIMER11_CLK_ENABLE()  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM11EN)
#define TIMER13_CLK_ENABLE()  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM13EN)
#define TIMER14_CLK_ENABLE()  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM14EN)

/*****************************************************************************************************************/
/*                                                GPIO MACROS                                                    */
/*****************************************************************************************************************/
/** @defgroup GPIO CLK Enable
 * @{
 */
#define GPIOA_CLK_ENABLE()    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN)
#define GPIOB_CLK_ENABLE()    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN)
#define GPIOC_CLK_ENABLE()    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN)
#define GPIOD_CLK_ENABLE()    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN)
#define GPIOE_CLK_ENABLE()    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN)

/*****************************************************************************************************************/
/*                                             GPIO MODE MACROS                                                  */
/*****************************************************************************************************************/
#define GPIO_MODER_MODE_INPUT     0x00
#define GPIO_MODER_MODE_OUTPUT    0x01
#define GPIO_MODER_MODE_ALT_FUNC  0x02
#define GPIO_MODER_MODE_ANALOG    0x03

#define GPIO_PUSHPULL     GPIO_OTYPER_OT_0

#define SET_GPIO_MODE(GPIO, GPIO_PIN, MODE)        (GPIO->MODER)  |=  ((MODE) << ((GPIO_PIN) * 2U))
#define CLR_GPIO_MODE(GPIO, GPIO_NUMBER)           ((GPIO->MODER) &= ~((GPIO_MODER_MODER0) << (GPIO_NUMBER * 2U)))

/*****************************************************************************************************************/
/*                                             GPIO ALTERNATIVE FUNCTIONS                                        */
/*****************************************************************************************************************/
#define SET_GPIO_AFR(GPIO, GPIO_NUMBER, ALT_FUNC)  ((GPIO->AFR[GPIO_NUMBER >> 3U]) |= ((ALT_FUNC) << (4 * (GPIO_NUMBER & 0x07U))))
#define CLR_GPIO_AFR(GPIO, GPIO_NUMBER)            ((GPIO->AFR[GPIO_NUMBER >> 3U]) &= ~((0x0F) << (4 * (GPIO_NUMBER & 0x07U))))

/*****************************************************************************************************************/
/*                                             GPIO PULL-UP/PULL-DOWN MACROS                                     */
/*****************************************************************************************************************/
#define SET_GPIO_OTYPER(GPIO, GPIO_NUMBER, MODE)   ((GPIO->OTYPER) |= ((MODE) >> 4U) << GPIO_NUMBER)
#define CLR_GPIO_OTYPER(GPIO, GPIO_NUMBER)         ((GPIO->OTYPER) &= ~(GPIO_OTYPER_OT_0 << GPIO_NUMBER))

/*****************************************************************************************************************/
/*                                        GPIO TIMER ALTERNATE FUNCTIONS                                         */
/*****************************************************************************************************************/
/**
 * @brief   AF 1 selection
 */
#define GPIO_AF1_TIM1          ((uint8_t)0x01)      /* TIM1 Alternate Function mapping */
#define GPIO_AF1_TIM2          ((uint8_t)0x01)      /* TIM2 Alternate Function mapping */

/**
 * @brief   AF 2 selection
 */
#define GPIO_AF2_TIM3          ((uint8_t)0x02)      /* TIM3 Alternate Function mapping */
#define GPIO_AF2_TIM4          ((uint8_t)0x02)      /* TIM4 Alternate Function mapping */
#define GPIO_AF2_TIM5          ((uint8_t)0x02)      /* TIM5 Alternate Function mapping */

/**
 * @brief   AF 3 selection
 */
#define GPIO_AF3_TIM8          ((uint8_t)0x03)      /* TIM8 Alternate Function mapping  */
#define GPIO_AF3_TIM9          ((uint8_t)0x03)      /* TIM9 Alternate Function mapping  */
#define GPIO_AF3_TIM10         ((uint8_t)0x03)      /* TIM10 Alternate Function mapping */
#define GPIO_AF3_TIM11         ((uint8_t)0x03)      /* TIM11 Alternate Function mapping */

/**
 * @brief   AF 3 selection
 */
#define GPIO_AF9_TIM12         ((uint8_t)0x09)      /* TIM12 Alternate Function mapping  */
#define GPIO_AF9_TIM13         ((uint8_t)0x09)      /* TIM13 Alternate Function mapping  */
#define GPIO_AF9_TIM14         ((uint8_t)0x09)      /* TIM14 Alternate Function mapping */

/* Configure cloks
 * Max SYSCLK: 168MHz
 * Max APB1: SYSCLK/4 = 42MHz
 * Max APB2: SYSCLK/2 = 84MHz
 *
 */
#define TIMER_MAX_FREQUENCY_1 ((uint32_t)168000000)   /* TIM1, TIM8, TIM8, TIM10, TIM11 Max timer clock */
#define TIMER_MAX_FREQUENCY_2 ((uint32_t)84000000)    /* TIM2, TIM5, TIM3, TIM4, TIM12, TIM13, TIM14  Max timer clock */




///******************************************************************************/
///*                GPIO ALTERNATIVE FUNCTIONS                                  */
///*                                                                            */
///* AFR[0] -> GPIO0 até GPIO7                                                  */
///* GPIOD->AFR[0] |= (ALTERNATE_FUNCTION_NUMBER << (4 * GPIO_NUMBER))          */
///* AFR[1] -> GPIO8 até GPIO15                                                 */
///* GPIOD->AFR[0] |= (ALTERNATE_FUNCTION_NUMBER << (4 * (GPIO_NUMBER - 8)))    */
///******************************************************************************/
//#define SET_GPIO_AFRL(GPIO, GPIO_NUMBER, ALT_FUNC)((GPIO->AFR[0]) |= ((ALT_FUNC) << (4 * (GPIO_NUMBER))))
//#define CLR_GPIO_AFRL(GPIO, GPIO_NUMBER)((GPIO->AFR[0]) &= ~((0x0F) << (4 * (GPIO_NUMBER))))
//
//#define SET_GPIO_AFRH(GPIO, GPIO_NUMBER, ALT_FUNC)((GPIO->AFR[1]) |= ((ALT_FUNC) << (4 * (GPIO_NUMBER - 8))))
//#define CLR_GPIO_AFRH(GPIO, GPIO_NUMBER)((GPIO->AFR[1]) &= ~(0x0F << (4 * (GPIO_NUMBER - 8))))
//
///******************************************************************************/
///*               GPIO MODE MACROS                                             */
///******************************************************************************/
//#define GPIO_MODER_MODE_INPUT 0x00
//#define GPIO_MODER_MODE_OUTPUT 0x01
//#define GPIO_MODER_MODE_ALT_FUNC 0x02
//#define GPIO_MODER_MODE_ANALOG 0x03
//
//#define SET_GPIO_MODE(GPIO, GPIO_NUMBER, MODE)  ((GPIO->MODER) |= ((MODE) << (2 * (GPIO_NUMBER))))
//#define CLR_GPIO_MODE(GPIO, GPIO_NUMBER)        ((GPIO->MODER) &= ~((0x03) << (2 * (GPIO_NUMBER))))
//
///******************************************************************************/
///*               GPIO PULL-UP/PULL-DOWN MACROS                                */
///******************************************************************************/
//#define GPIO_PUPDR_NO_PULL 0x00
//#define GPIO_PUPDR_PULL_UP 0x01
//#define GPIO_PUPDR_PULL_DOWN 0x02
//#define GPIO_PUPDR_PULL_RESERVED 0x03
//
//#define SET_GPIO_PULL(GPIO, GPIO_NUMBER, MODE)  ((GPIO->PUPDR) |= ((MODE) << (2 * (GPIO_NUMBER))))
//#define CLR_GPIO_PULL(GPIO, GPIO_NUMBER)        ((GPIO->PUPDR) &= ~((0x03) << (2 * (GPIO_NUMBER))))
//
///******************************************************************************/
///*               GPIO SPEED MACROS                                            */
///******************************************************************************/
//#define GPIO_OSPEED_LOW_SPEED_2MHZ 0x00
//#define GPIO_OSPEED_MEDIUM_SPEED_25MHZ 0x01
//#define GPIO_OSPEED_FAST_SPEED_50MHZ 0x02
//#define GPIO_OSPEED_HIGH_SPEED_100MHZ 0x03
//
//#define SET_GPIO_SPEED(GPIO, GPIO_NUMBER, MODE)  ((GPIO->OSPEEDR) |= ((MODE) << (2 * (GPIO_NUMBER))))
//#define CLR_GPIO_SPEED(GPIO, GPIO_NUMBER)        ((GPIO->OSPEEDR) &= ~((0x03) << (2 * (GPIO_NUMBER))))
//
//
///*******************************************************************************/
///*               GPIO TIMER ALTERNATE FUNCTIONS                               */
///************************ *****************************************************/
///**
// * @brief   AF 1 selection
// */
//#define GPIO_AF1_TIM1          ((uint8_t)0x01)      /* TIM1 Alternate Function mapping */
//#define GPIO_AF1_TIM2          ((uint8_t)0x01)      /* TIM2 Alternate Function mapping */
//
///**
// * @brief   AF 2 selection
// */
//#define GPIO_AF2_TIM3          ((uint8_t)0x02)      /* TIM3 Alternate Function mapping */
//#define GPIO_AF2_TIM4          ((uint8_t)0x02)      /* TIM4 Alternate Function mapping */
//#define GPIO_AF2_TIM5          ((uint8_t)0x02)      /* TIM5 Alternate Function mapping */
//
///**
// * @brief   AF 3 selection
// */
//#define GPIO_AF3_TIM8          ((uint8_t)0x03)      /* TIM8 Alternate Function mapping  */
//#define GPIO_AF3_TIM9          ((uint8_t)0x03)      /* TIM9 Alternate Function mapping  */
//#define GPIO_AF3_TIM10         ((uint8_t)0x03)      /* TIM10 Alternate Function mapping */
//#define GPIO_AF3_TIM11         ((uint8_t)0x03)      /* TIM11 Alternate Function mapping */
//
///**
// * @brief   AF 3 selection
// */
//#define GPIO_AF9_TIM12         ((uint8_t)0x09)      /* TIM12 Alternate Function mapping  */
//#define GPIO_AF9_TIM13         ((uint8_t)0x09)      /* TIM13 Alternate Function mapping  */
//#define GPIO_AF9_TIM14         ((uint8_t)0x09)      /* TIM14 Alternate Function mapping */
//
//
///******************************************************************************/
///*               GPIO CLOCK ENABLE                                                 */
///******************************************************************************/
//#define GPIOA_CLK_ENABLE()    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN)
//#define GPIOB_CLK_ENABLE()    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN)
//#define GPIOC_CLK_ENABLE()    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN)
//#define GPIOD_CLK_ENABLE()    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN)
//#define GPIOE_CLK_ENABLE()    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN)
//
///* Configure cloks
// * Max SYSCLK: 168MHz
// * Max APB1: SYSCLK/4 = 42MHz
// * Max APB2: SYSCLK/2 = 84MHz
// *
// */
//#define TIMER_MAX_FREQUENCY_1 ((uint32_t)168000000)   /* TIM1, TIM8, TIM8, TIM10, TIM11 Max timer clock */
//#define TIMER_MAX_FREQUENCY_2 ((uint32_t)84000000)    /* TIM2, TIM5, TIM3, TIM4, TIM12, TIM13, TIM14  Max timer clock */
//
//
///******************************************************************************/
///*               TIMER MACROS                                                 */
///******************************************************************************/
//#define TIMER1_CLK_ENABLE()   SET_MASK(RCC->APB2ENR, RCC_APB2ENR_TIM1EN)
//#define TIMER8_CLK_ENABLE()   SET_MASK(RCC->APB2ENR, RCC_APB2ENR_TIM8EN)
//
///* These Timers have 4 channels */
//#define TIMER2_CLK_ENABLE()   SET_MASK(RCC->APB1ENR, RCC_APB1ENR_TIM2EN)
//#define TIMER3_CLK_ENABLE()   SET_MASK(RCC->APB1ENR, RCC_APB1ENR_TIM3EN)
//#define TIMER4_CLK_ENABLE()   SET_MASK(RCC->APB1ENR, RCC_APB1ENR_TIM4EN)
//#define TIMER5_CLK_ENABLE()   SET_MASK(RCC->APB1ENR, RCC_APB1ENR_TIM5EN)
//
///* These Timers have 2 independent 16 bit channels for PWM */
//#define TIMER9_CLK_ENABLE()   SET_MASK(RCC->APB2ENR, RCC_APB2ENR_TIM9EN)
//#define TIMER12_CLK_ENABLE()  SET_MASK(RCC->APB1ENR, RCC_APB1ENR_TIM12EN)
//
//#define TIMER10_CLK_ENABLE()  SET_MASK(RCC->APB2ENR, RCC_APB2ENR_TIM10EN)
//#define TIMER11_CLK_ENABLE()  SET_MASK(RCC->APB2ENR, RCC_APB2ENR_TIM11EN)
//#define TIMER13_CLK_ENABLE()  SET_MASK(RCC->APB1ENR, RCC_APB1ENR_TIM13EN)
//#define TIMER14_CLK_ENABLE()  SET_MASK(RCC->APB1ENR, RCC_APB1ENR_TIM14EN)
//
//
///* Configure cloks
// * Max SYSCLK: 168MHz
// * Max APB1: SYSCLK/4 = 42MHz
// * Max APB2: SYSCLK/2 = 84MHz
// *
// */
//#define TIMER_MAX_FREQUENCY_1 ((uint32_t)168000000)   /* TIM1, TIM8, TIM8, TIM10, TIM11 Max timer clock */
//#define TIMER_MAX_FREQUENCY_2 ((uint32_t)84000000)    /* TIM2, TIM5, TIM3, TIM4, TIM12, TIM13, TIM14  Max timer clock */


typedef enum
{
  PWM_CH_1 = 1,
  PWM_CH_2,
  PWM_CH_3,
  PWM_CH_4,
  PWM_CH_1N,
  PWM_CH_2N,
  PWM_CH_3N,
}PWM_ChannelOnly_t;


/*
 * @brief
 */
typedef struct
{
  PWM_Channel_t     Channel;
  TIM_TypeDef       *TIMx;
  uint32_t          MaxFrequency;
  PWM_ChannelOnly_t ChannelOnly;
  GPIO_TypeDef      *GPIOx;
  uint8_t           Pin;
  uint32_t          AlternateFunction;
} PWM_RegConfig_t;


/* *****************************************************************************
 *        CONFIGURATION TABLES
 ***************************************************************************** */
/* The table below correlates, for each available channel configuration, which*/
/*  is the FTM peripheral, the channel, the port and the pin. These values are*/
/*  used for proper control and configuration of the peripherals so that the  */
/*  driver can work correctly.                                                */
/* - TIM Number: 0 = TIM0, 1 = TIM1...                                        */
/* - TIM Channel 0 = CH0, 1 = CH1...                                          */
/* - Port Pin: PORTA, PORTB, ....                                             */
/*    given pin, in order to select the TIMx_CHy value.                       */
/* THE LIST HAS TO BE SEQUENTIAL, FOLLOWING THE ENUM ORDER!                   */
#define ADD_CONFIG( ENUM, TIM, MAX_FREQ, CHN, PORT, PIN, AF)  { ENUM, TIM, MAX_FREQ, CHN, PORT, PIN, AF }

const PWM_RegConfig_t PWM_RegTable[PWM_NUMBER_OF_CHANNELS] =
{
  /*          PWM_Channel_t,   TIMER,      MAXIMUM FREQUENCY,    CHANNEL, GPIO PORT, GPIO PIN,  ALTERNATE FUN */
  ADD_CONFIG(PWM1_CH1_PE9,     TIM1,   TIMER_MAX_FREQUENCY_1,  PWM_CH_1,     GPIOE,    9,  GPIO_AF1_TIM1),
  ADD_CONFIG(PWM1_CH1N_PA7,    TIM1,   TIMER_MAX_FREQUENCY_1,  PWM_CH_1N,    GPIOA,    7,  GPIO_AF1_TIM1),
  ADD_CONFIG(PWM1_CH2_PE11,    TIM1,   TIMER_MAX_FREQUENCY_1,  PWM_CH_2,     GPIOE,    11, GPIO_AF1_TIM1),
  ADD_CONFIG(PWM1_CH2N_PB0,    TIM1,   TIMER_MAX_FREQUENCY_1,  PWM_CH_2N,    GPIOB,    0,  GPIO_AF1_TIM1),
  ADD_CONFIG(PWM1_CH3_PE13,    TIM1,   TIMER_MAX_FREQUENCY_1,  PWM_CH_3,     GPIOE,    13, GPIO_AF1_TIM1),
  ADD_CONFIG(PWM1_CH3N_PE12,   TIM1,   TIMER_MAX_FREQUENCY_1,  PWM_CH_3N,    GPIOE,    12, GPIO_AF1_TIM1),
  ADD_CONFIG(PWM1_CH4_PE14,    TIM1,   TIMER_MAX_FREQUENCY_1,  PWM_CH_4,     GPIOE,    14, GPIO_AF1_TIM1),

  ADD_CONFIG(PWM2_CH1_PA0,     TIM2,   TIMER_MAX_FREQUENCY_2,  PWM_CH_1,     GPIOA,    0,  GPIO_AF1_TIM2),
  ADD_CONFIG(PWM2_CH2_PA1,     TIM2,   TIMER_MAX_FREQUENCY_2,  PWM_CH_2,     GPIOA,    1,  GPIO_AF1_TIM2),
  ADD_CONFIG(PWM2_CH3_PA2,     TIM2,   TIMER_MAX_FREQUENCY_2,  PWM_CH_3,     GPIOA,    2,  GPIO_AF1_TIM2),
  ADD_CONFIG(PWM2_CH4_PA3,     TIM2,   TIMER_MAX_FREQUENCY_2,  PWM_CH_4,     GPIOA,    3,  GPIO_AF1_TIM2),

  ADD_CONFIG(PWM3_CH1_PA6,     TIM3,   TIMER_MAX_FREQUENCY_2,  PWM_CH_1,     GPIOA,    6,  GPIO_AF2_TIM3),
  ADD_CONFIG(PWM3_CH2_PA7,     TIM3,   TIMER_MAX_FREQUENCY_2,  PWM_CH_2,     GPIOA,    7,  GPIO_AF2_TIM3),
  ADD_CONFIG(PWM3_CH3_PB0,     TIM3,   TIMER_MAX_FREQUENCY_2,  PWM_CH_3,     GPIOB,    0,  GPIO_AF2_TIM3),
  ADD_CONFIG(PWM3_CH4_PB1,     TIM3,   TIMER_MAX_FREQUENCY_2,  PWM_CH_4,     GPIOB,    1,  GPIO_AF2_TIM3),

  ADD_CONFIG(PWM4_CH1_PD12,    TIM4,   TIMER_MAX_FREQUENCY_2,  PWM_CH_1,     GPIOD,    12, GPIO_AF2_TIM4),
  ADD_CONFIG(PWM4_CH2_PD13,    TIM4,   TIMER_MAX_FREQUENCY_2,  PWM_CH_2,     GPIOD,    13, GPIO_AF2_TIM4),
  ADD_CONFIG(PWM4_CH3_PD14,    TIM4,   TIMER_MAX_FREQUENCY_2,  PWM_CH_3,     GPIOD,    14, GPIO_AF2_TIM4),
  ADD_CONFIG(PWM4_CH4_PD15,    TIM4,   TIMER_MAX_FREQUENCY_2,  PWM_CH_4,     GPIOD,    15, GPIO_AF2_TIM4),

  ADD_CONFIG(PWM4_CH1_PB6,     TIM4,   TIMER_MAX_FREQUENCY_2,  PWM_CH_1,     GPIOB,    6, GPIO_AF2_TIM4),
  ADD_CONFIG(PWM4_CH2_PB7,     TIM4,   TIMER_MAX_FREQUENCY_2,  PWM_CH_2,     GPIOB,    7, GPIO_AF2_TIM4),
  ADD_CONFIG(PWM4_CH3_PB8,     TIM4,   TIMER_MAX_FREQUENCY_2,  PWM_CH_3,     GPIOB,    8, GPIO_AF2_TIM4),
  ADD_CONFIG(PWM4_CH4_PB9,     TIM4,   TIMER_MAX_FREQUENCY_2,  PWM_CH_4,     GPIOB,    9, GPIO_AF2_TIM4),

  ADD_CONFIG(PWM5_CH1_PA0,     TIM5,   TIMER_MAX_FREQUENCY_2,  PWM_CH_1,     GPIOA,    0,  GPIO_AF2_TIM5),
  ADD_CONFIG(PWM5_CH2_PA1,     TIM5,   TIMER_MAX_FREQUENCY_2,  PWM_CH_2,     GPIOA,    1,  GPIO_AF2_TIM5),
  ADD_CONFIG(PWM5_CH3_PA2,     TIM5,   TIMER_MAX_FREQUENCY_2,  PWM_CH_3,     GPIOA,    2,  GPIO_AF2_TIM5),

  ADD_CONFIG(PWM8_CH1_PC6,     TIM8,   TIMER_MAX_FREQUENCY_1,  PWM_CH_1,     GPIOC,    6,  GPIO_AF3_TIM8),
  ADD_CONFIG(PWM8_CH1N_PA5,    TIM8,   TIMER_MAX_FREQUENCY_1,  PWM_CH_1N,    GPIOA,    5,  GPIO_AF3_TIM8),
  ADD_CONFIG(PWM8_CH2_PC7,     TIM8,   TIMER_MAX_FREQUENCY_1,  PWM_CH_2,     GPIOC,    7,  GPIO_AF3_TIM8),
  ADD_CONFIG(PWM8_CH2N_PB0,    TIM8,   TIMER_MAX_FREQUENCY_1,  PWM_CH_2N,    GPIOB,    0,  GPIO_AF3_TIM8),
  ADD_CONFIG(PWM8_CH3_PC8,     TIM8,   TIMER_MAX_FREQUENCY_1,  PWM_CH_3,     GPIOC,    8,  GPIO_AF3_TIM8),
  ADD_CONFIG(PWM8_CH3N_PB1,    TIM8,   TIMER_MAX_FREQUENCY_1,  PWM_CH_3N,    GPIOB,    1,  GPIO_AF3_TIM8),
  ADD_CONFIG(PWM8_CH4_PC9,     TIM8,   TIMER_MAX_FREQUENCY_1,  PWM_CH_4,     GPIOC,    9,  GPIO_AF3_TIM8),

  ADD_CONFIG(PWM9_CH1_PE5,     TIM9,   TIMER_MAX_FREQUENCY_1,  PWM_CH_1,     GPIOE,    5,  GPIO_AF3_TIM9),
  ADD_CONFIG(PWM9_CH2_PE6,     TIM9,   TIMER_MAX_FREQUENCY_1,  PWM_CH_2,    GPIOE,    6,  GPIO_AF3_TIM9),

  ADD_CONFIG(PWM10_CH1_PB8,    TIM10,  TIMER_MAX_FREQUENCY_1,  PWM_CH_1,    GPIOB,    8,  GPIO_AF3_TIM10),

  ADD_CONFIG(PWM11_CH1_PB9,    TIM11,  TIMER_MAX_FREQUENCY_1,  PWM_CH_1,    GPIOB,    9,  GPIO_AF3_TIM11),

  ADD_CONFIG(PWM12_CH1_PB14,   TIM12,  TIMER_MAX_FREQUENCY_2,  PWM_CH_1,     GPIOB,    14, GPIO_AF9_TIM12),
  ADD_CONFIG(PWM12_CH2_PB15,   TIM12,  TIMER_MAX_FREQUENCY_2,  PWM_CH_2,     GPIOB,    15, GPIO_AF9_TIM12),

  ADD_CONFIG(PWM13_CH1_PA6,    TIM13,  TIMER_MAX_FREQUENCY_2,  PWM_CH_1,     GPIOA,    6,  GPIO_AF9_TIM13),

  ADD_CONFIG(PWM14_CH1_PA7,    TIM14,  TIMER_MAX_FREQUENCY_2,  PWM_CH_1,     GPIOA,    7,  GPIO_AF9_TIM14),
};


const uint32_t PWM_Res2MaxValue[PWM_NUMBER_OF_RESOLUTIONS] = {
    255,   /*!< PWM_8BITS */
    1023,  /*!< PWM_10BITS */
    4095,  /*!< PWM_12BITS */
    16383, /*!< PWM_14BITS */
    65535, /*!< PWM_16BITS */
};

/**
 * @brief Configurations and status saved for all PWM IDs.
 */
struct
{
  PWM_Parameters_t Parameters;    /*!< Parameters that can be configured */
  uint8_t          isInitialized; /*!< Flags if the a pin was configured */
}PWM_List[PWM_MAX_ID];


EStatus_t PWM_SetDutyByFrequency(PWM_Channel_t Channel, float Frequency,
    float Duty);

EStatus_t PWM_ConfigByRes(PWM_Channel_t Channel, PWM_Resolution_t Resolution,
    float Frequency, float Duty);

EStatus_t PWM_SetDutyByRes(PWM_Channel_t Channel, PWM_Resolution_t Resolution,
    float Duty);


EStatus_t PWM_Init(uint8_t ID, PWM_Parameters_t Parameter)
{
  EStatus_t  returnValue = ANSWERED_REQUEST;

  if(ID < PWM_MAX_ID)
  {
    if(PWM_List[ID].isInitialized == 0)
    {
      if( Parameter.Channel < PWM_NUMBER_OF_CHANNELS &&
          Parameter.Mode < PWM_NUMBER_OF_MODES &&
          Parameter.Reslution < PWM_NUMBER_OF_RESOLUTIONS &&
          Parameter.Preference < PWM_NUMBER_OF_PREFERENCES)
      {

        /* Enable TIMER RCC CLK */
        if(PWM_RegTable[Parameter.Channel].TIMx == TIM1) {TIMER1_CLK_ENABLE();}
        if(PWM_RegTable[Parameter.Channel].TIMx == TIM2) {TIMER2_CLK_ENABLE();}
        if(PWM_RegTable[Parameter.Channel].TIMx == TIM3) {TIMER3_CLK_ENABLE();}
        if(PWM_RegTable[Parameter.Channel].TIMx == TIM4) {TIMER4_CLK_ENABLE();}
        if(PWM_RegTable[Parameter.Channel].TIMx == TIM5) {TIMER5_CLK_ENABLE();}
        if(PWM_RegTable[Parameter.Channel].TIMx == TIM8) {TIMER8_CLK_ENABLE();}
        if(PWM_RegTable[Parameter.Channel].TIMx == TIM9) {TIMER9_CLK_ENABLE();}
        if(PWM_RegTable[Parameter.Channel].TIMx == TIM10){TIMER10_CLK_ENABLE();}
        if(PWM_RegTable[Parameter.Channel].TIMx == TIM11){TIMER11_CLK_ENABLE();}
        if(PWM_RegTable[Parameter.Channel].TIMx == TIM12){TIMER12_CLK_ENABLE();}
        if(PWM_RegTable[Parameter.Channel].TIMx == TIM13){TIMER13_CLK_ENABLE();}
        if(PWM_RegTable[Parameter.Channel].TIMx == TIM14){TIMER14_CLK_ENABLE();}

        SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->EGR, TIM_EGR_UG);

        /* Configuring Timer CAPTURE/COMPARE MODE  */
        switch(PWM_RegTable[Parameter.Channel].ChannelOnly)
        {
          case PWM_CH_1N:
            /* Reset the CCxE Bit CHANNEL 3 */
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
                TIM_CCER_CC1NE);
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR1,
                TIM_CCMR1_OC1M);
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR1,
                TIM_CCMR1_CC1S);

//            if(Parameter.Polarity)
//              SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
//            TIM_CCER_CC1NP);
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR1,
                (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1));
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR1,
                TIM_CCMR1_OC1PE);
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
                TIM_CCER_CC1NE);
            break;

          case PWM_CH_1:
            /* Reset the CCxE Bit CHANNEL 3 */
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
                TIM_CCER_CC1E);
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR1,
                TIM_CCMR1_OC1M);
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR1,
                TIM_CCMR1_CC1S);

//            if(Parameter.Polarity)
//              SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
//            TIM_CCER_CC1P);
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR1,
                (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1));
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR1,
                TIM_CCMR1_OC1PE);
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
                TIM_CCER_CC1E);
            break;

          case PWM_CH_2N:
            /* Reset the CCxE Bit CHANNEL 3 */
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
                TIM_CCER_CC2NE);
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR1,
                TIM_CCMR1_OC1M);
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR1,
                TIM_CCMR1_CC1S);

//            if(Parameter.Polarity)
//              SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
//            TIM_CCER_CC2NP);
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR1,
                (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1));
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR1,
                TIM_CCMR1_OC1PE);
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
                TIM_CCER_CC2NE);
            break;

          case PWM_CH_2:
            /* Reset the CCxE Bit CHANNEL 2 */
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
                TIM_CCER_CC2E);
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR1,
                TIM_CCMR1_OC2M);
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR1,
                TIM_CCMR1_CC2S);

//            if(Parameter.Polarity)
//              SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
//            TIM_CCER_CC2P);
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR1,
                (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1));
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR1,
                TIM_CCMR1_OC2PE);
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
                TIM_CCER_CC2E);
            break;

          case PWM_CH_3N:
            /* Reset the CCxE Bit CHANNEL 3 */
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
                TIM_CCER_CC3NE);
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR2,
                TIM_CCMR2_OC3M);
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR2,
                TIM_CCMR2_CC3S);

//            if(Parameter.Polarity)
//              SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
//            TIM_CCER_CC3NP);
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR2,
                (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1));
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR2,
                TIM_CCMR2_OC3PE);
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
                TIM_CCER_CC3NE);
            break;

          case PWM_CH_3:
            /* Reset the CCxE Bit CHANNEL 3 */
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
                TIM_CCER_CC3E);
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR2,
                TIM_CCMR2_OC3M);
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR2,
                TIM_CCMR2_CC3S);

//            if(Parameter.Polarity)
//              SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
//            TIM_CCER_CC3P);
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR2,
                (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1));
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR2,
                TIM_CCMR2_OC3PE);
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
                TIM_CCER_CC3E);
            break;

          case PWM_CH_4:
            /* Reset the CCxE Bit CHANNEL 4 */
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
                TIM_CCER_CC4E);
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR2,
                TIM_CCMR2_OC4M);
            CLEAR_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR2,
                TIM_CCMR2_CC4S);

//            if(Parameter.Polarity)
//              SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
//            TIM_CCER_CC4P);
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR2,
                (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1));
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCMR2,
                TIM_CCMR2_OC4PE);
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CCER,
                TIM_CCER_CC4E);
            break;

          default:
            returnValue = ERR_PARAM_VALUE;
            break;
        }

        if( returnValue == ANSWERED_REQUEST )
        {

          /* Enable GPIO RCC CLK */
          if(PWM_RegTable[Parameter.Channel].GPIOx == GPIOA){GPIOA_CLK_ENABLE();}
          if(PWM_RegTable[Parameter.Channel].GPIOx == GPIOB){GPIOB_CLK_ENABLE();}
          if(PWM_RegTable[Parameter.Channel].GPIOx == GPIOC){GPIOC_CLK_ENABLE();}
          if(PWM_RegTable[Parameter.Channel].GPIOx == GPIOD){GPIOD_CLK_ENABLE();}
          if(PWM_RegTable[Parameter.Channel].GPIOx == GPIOE){GPIOE_CLK_ENABLE();}

          /* Configure GPIO Port as alternate function */
          CLR_GPIO_AFR(PWM_RegTable[Parameter.Channel].GPIOx,
              PWM_RegTable[Parameter.Channel].Pin);
          SET_GPIO_AFR(PWM_RegTable[Parameter.Channel].GPIOx,
              PWM_RegTable[Parameter.Channel].Pin,
              PWM_RegTable[Parameter.Channel].AlternateFunction);

          /* Configure Port register */
          CLR_GPIO_MODE(PWM_RegTable[Parameter.Channel].GPIOx,
              PWM_RegTable[Parameter.Channel].Pin);
          SET_GPIO_MODE(PWM_RegTable[Parameter.Channel].GPIOx,
              PWM_RegTable[Parameter.Channel].Pin, GPIO_MODER_MODE_ALT_FUNC);

          /* Configure port output type register as push-pull */
          CLR_GPIO_OTYPER(PWM_RegTable[Parameter.Channel].GPIOx,
              PWM_RegTable[Parameter.Channel].Pin);
          SET_GPIO_OTYPER(PWM_RegTable[Parameter.Channel].GPIOx,
              PWM_RegTable[Parameter.Channel].Pin, GPIO_PUSHPULL);


          if(Parameter.Duty < 0.0)   Parameter.Duty = 0.0;
          if(Parameter.Duty > 100.0) Parameter.Duty = 100.0;

          if( Parameter.Preference == PWM_FREQUENCY )
          {
            returnValue = PWM_SetDutyByFrequency(Parameter.Channel,
                Parameter.Frequency, Parameter.Duty);
          }
          else
          {
            returnValue = PWM_ConfigByRes(Parameter.Channel,
                Parameter.Reslution, Parameter.Frequency, Parameter.Duty);
          }

          if( returnValue == ANSWERED_REQUEST )
          {
            /* Enable the Main Output */
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->BDTR,
                TIM_BDTR_MOE);
            SET_MASK(PWM_RegTable[Parameter.Channel].TIMx->CR1, TIM_CR1_CEN);

            PWM_List[ID].Parameters = Parameter;
            PWM_List[ID].isInitialized = 1;
          }

        }



      }
      else
      {
        if( Parameter.Channel >= PWM_NUMBER_OF_CHANNELS )
        {
          returnValue = ERR_PARAM_VALUE;
        }
        else
        {
          returnValue = ERR_PARAM_RANGE;
        }
      }
    }
    else
    {
      returnValue = ERR_ENABLED;
    }
  }
  else
  {
    returnValue = ERR_PARAM_ID;
  }
  return returnValue;
}


EStatus_t PWM_SetDuty(uint8_t ID, float Duty)
{
  EStatus_t returnValue = OPERATION_RUNNING;


  /* Validating parameters */
  if( ID < PWM_MAX_ID && PWM_List[ID].isInitialized == 1)
  {
    if( Duty > 100.0 ){ Duty = 100.0;}
    if( Duty < 0.0 ){ Duty = 0.0;}

    if( PWM_List[ID].Parameters.Preference == PWM_FREQUENCY )
    {
      returnValue = PWM_SetDutyByFrequency(PWM_List[ID].Parameters.Channel,
          PWM_List[ID].Parameters.Frequency, Duty);
    }
    else
    {
      returnValue = PWM_SetDutyByRes(PWM_List[ID].Parameters.Channel,
          PWM_List[ID].Parameters.Reslution, Duty);
    }

  }
  else
  {
    if(ID >= PWM_MAX_ID){
      returnValue = ERR_PARAM_ID;
    }else{
      returnValue = ERR_DISABLED;
    }
  }

  return returnValue;
}


EStatus_t PWM_SetFrequency(uint8_t ID, float Frequency)
{
  EStatus_t returnValue = OPERATION_RUNNING;


  /* Validating parameters */
  if( ID < PWM_MAX_ID && PWM_List[ID].isInitialized == 1)
  {
    if( Frequency > 0.0 )
    {

      /* Feature available only if preference is frequency */
      if( PWM_List[ID].Parameters.Preference == PWM_FREQUENCY )
      {
        returnValue = PWM_SetDutyByFrequency(PWM_List[ID].Parameters.Channel,
            Frequency, PWM_List[ID].Parameters.Duty);
      }
      else
      {
        returnValue = ERR_FAULT;
      }

    }
    else
    {
      returnValue = ERR_PARAM_RANGE;
    }

  }
  else
  {
    if(ID >= PWM_MAX_ID){
      returnValue = ERR_PARAM_ID;
    }else{
      returnValue = ERR_DISABLED;
    }
  }

  return returnValue;
}

EStatus_t PWM_SetDutyByFrequency(PWM_Channel_t Channel, float Frequency,
    float Duty)
{
  EStatus_t returnCode = ANSWERED_REQUEST;
  uint8_t prescalerFound  = 0;
  float auxCounter;
  uint32_t counter;
  uint32_t prescaler;

  /* Find the values for timer registers matching Frequency and DUty */
  for(prescaler = 0; prescaler <= 0xffff ; prescaler++)
  {
    /* PWM_freq = MaxFreq / ( (Prescaler + 1) * Counter ) */
    auxCounter = ((PWM_RegTable[Channel].MaxFrequency
        /((float)prescaler + 1.0))/Frequency);
    counter = (uint32_t) auxCounter;
    if( counter <= 0xffff && counter > 0)
    {
      prescalerFound = 1;
      break;
    }
  }

  if(!prescalerFound)
  {
    returnCode =  ERR_FAILED;
  }
  else
  {
    /* Loading Timer's registers with pwm computed parameters */
    PWM_RegTable[Channel].TIMx->ARR = (uint16_t)counter;
    PWM_RegTable[Channel].TIMx->PSC = (uint16_t)prescaler;

    switch(PWM_RegTable[Channel].ChannelOnly)
    {
    case PWM_CH_1N:
    case PWM_CH_1:
      PWM_RegTable[Channel].TIMx->CCR1 = counter * (Duty / 100);
      break;

    case PWM_CH_2N:
    case PWM_CH_2:
      PWM_RegTable[Channel].TIMx->CCR2 = counter * (Duty / 100);
      break;

    case PWM_CH_3N:
    case PWM_CH_3:
      PWM_RegTable[Channel].TIMx->CCR3 = counter * (Duty / 100);
      break;

    case PWM_CH_4:
      PWM_RegTable[Channel].TIMx->CCR4 = counter * (Duty / 100);
      break;
    default:
      returnCode = ERR_PARAM_VALUE;
    }
  }

  return returnCode;
}


EStatus_t PWM_ConfigByRes(PWM_Channel_t Channel, PWM_Resolution_t Resolution,
    float Frequency, float Duty)
{
  EStatus_t returnCode = ANSWERED_REQUEST;
  uint32_t counter;
  uint32_t prescaler;
  float auxPrescaler;
  uint32_t intDuty;

  /* Find the values for timer registers matching Resolution and DUty */
  counter = PWM_Res2MaxValue[Resolution]; /* Fixing PWM resolution */
  auxPrescaler = ( PWM_RegTable[Channel].MaxFrequency /
      ( (float)counter * Frequency ) ) - 1.0;
  prescaler = (uint32_t) auxPrescaler;
  if( prescaler <= 0xffff )
  {
    /* Loading Timer's registers with pwm computed parameters */
    Duty = Duty * ( (float)PWM_Res2MaxValue[Resolution] * 0.01 );
    intDuty = (uint32_t) Duty;
    intDuty &= PWM_Res2MaxValue[Resolution];

    switch(PWM_RegTable[Channel].ChannelOnly)
    {
    case PWM_CH_1N:
    case PWM_CH_1:
      PWM_RegTable[Channel].TIMx->CCR1 = intDuty;
      break;

    case PWM_CH_2N:
    case PWM_CH_2:
      PWM_RegTable[Channel].TIMx->CCR2 = intDuty;
      break;

    case PWM_CH_3N:
    case PWM_CH_3:
      PWM_RegTable[Channel].TIMx->CCR3 = intDuty;
      break;

    case PWM_CH_4:
      PWM_RegTable[Channel].TIMx->CCR4 = intDuty;
      break;
    default:
      returnCode = ERR_PARAM_VALUE;
    }

    if( returnCode == ANSWERED_REQUEST )
    {
      PWM_RegTable[Channel].TIMx->ARR = (uint16_t)counter;
      PWM_RegTable[Channel].TIMx->PSC = (uint16_t)prescaler;
    }

  }
  else
  {
    returnCode = ERR_FAILED;
  }

  return returnCode;
}


EStatus_t PWM_SetDutyByRes(PWM_Channel_t Channel, PWM_Resolution_t Resolution,
    float Duty)
{
  EStatus_t returnCode = ANSWERED_REQUEST;
  uint32_t intDuty;

  Duty = Duty * ( (float)PWM_Res2MaxValue[Resolution] * 0.01 );
  intDuty = (uint32_t) Duty;
  intDuty &= PWM_Res2MaxValue[Resolution];


  switch(PWM_RegTable[Channel].ChannelOnly)
  {
  case PWM_CH_1N:
  case PWM_CH_1:
    PWM_RegTable[Channel].TIMx->CCR1 = intDuty;
    break;

  case PWM_CH_2N:
  case PWM_CH_2:
    PWM_RegTable[Channel].TIMx->CCR2 = intDuty;
    break;

  case PWM_CH_3N:
  case PWM_CH_3:
    PWM_RegTable[Channel].TIMx->CCR3 = intDuty;
    break;

  case PWM_CH_4:
    PWM_RegTable[Channel].TIMx->CCR4 = intDuty;
    break;
  default:
    returnCode = ERR_PARAM_VALUE;
  }

  return returnCode;
}

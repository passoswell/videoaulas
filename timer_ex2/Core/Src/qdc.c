#include "qdc.h"
#include "macros.h"
#include "stm32f4xx.h"

/******************************************************************************/
/*                GPIO ALTERNATIVE FUNCTIONS                                  */
/*                                                                            */
/* AFR[0] -> GPIO0 to  GPIO7                                                  */
/* GPIOD->AFR[0] |= (ALTERNATE_FUNCTION_NUMBER << (4 * GPIO_NUMBER))          */
/* AFR[1] -> GPIO8 to  GPIO15                                                 */
/* GPIOD->AFR[0] |= (ALTERNATE_FUNCTION_NUMBER << (4 * (GPIO_NUMBER - 8)))    */
/******************************************************************************/
#define SET_GPIO_AFRL(GPIO, GPIO_NUMBER, ALT_FUNC)((GPIO->AFR[0]) |= ((ALT_FUNC) << (4 * (GPIO_NUMBER))))
#define CLR_GPIO_AFRL(GPIO, GPIO_NUMBER)((GPIO->AFR[0]) &= ~((0x0F) << (4 * (GPIO_NUMBER))))

#define SET_GPIO_AFRH(GPIO, GPIO_NUMBER, ALT_FUNC)((GPIO->AFR[1]) |= ((ALT_FUNC) << (4 * (GPIO_NUMBER - 8))))
#define CLR_GPIO_AFRH(GPIO, GPIO_NUMBER)((GPIO->AFR[1]) &= ~(0x0F << (4 * (GPIO_NUMBER - 8))))

/******************************************************************************/
/*               GPIO MODE MACROS                                             */
/******************************************************************************/
#define GPIO_MODER_MODE_INPUT 0x00
#define GPIO_MODER_MODE_OUTPUT 0x01
#define GPIO_MODER_MODE_ALT_FUNC 0x02
#define GPIO_MODER_MODE_ANALOG 0x03

#define SET_GPIO_MODE(GPIO, GPIO_NUMBER, MODE)  ((GPIO->MODER) |= ((MODE) << (2 * (GPIO_NUMBER))))
#define CLR_GPIO_MODE(GPIO, GPIO_NUMBER)        ((GPIO->MODER) &= ~((0x03) << (2 * (GPIO_NUMBER))))

/******************************************************************************/
/*               GPIO PULL-UP/PULL-DOWN MACROS                                */
/******************************************************************************/
#define GPIO_PUPDR_NO_PULL 0x00
#define GPIO_PUPDR_PULL_UP 0x01
#define GPIO_PUPDR_PULL_DOWN 0x02
#define GPIO_PUPDR_PULL_RESERVED 0x03

#define SET_GPIO_PULL(GPIO, GPIO_NUMBER, MODE)  ((GPIO->PUPDR) |= ((MODE) << (2 * (GPIO_NUMBER))))
#define CLR_GPIO_PULL(GPIO, GPIO_NUMBER)        ((GPIO->PUPDR) &= ~((0x03) << (2 * (GPIO_NUMBER))))

/******************************************************************************/
/*               GPIO SPEED MACROS                                            */
/******************************************************************************/
#define GPIO_OSPEED_LOW_SPEED_2MHZ 0x00
#define GPIO_OSPEED_MEDIUM_SPEED_25MHZ 0x01
#define GPIO_OSPEED_FAST_SPEED_50MHZ 0x02
#define GPIO_OSPEED_HIGH_SPEED_100MHZ 0x03

#define SET_GPIO_SPEED(GPIO, GPIO_NUMBER, MODE)  ((GPIO->OSPEEDR) |= ((MODE) << (2 * (GPIO_NUMBER))))
#define CLR_GPIO_SPEED(GPIO, GPIO_NUMBER)        ((GPIO->OSPEEDR) &= ~((0x03) << (2 * (GPIO_NUMBER))))


/**
 * @brief Configurations and status saved for all QDC IDs.
 */
struct
{
  QDC_Parameters_t Parameters;    /*!< Parameters that can be configured */
  TIM_TypeDef      *TIMx;         /*!< Pointer to a QDC register on the MCU*/
  uint8_t          isInitialized; /*!< Flags if the a pin was configured */
}QDC_List[QDC_MAX_ID];

void TIMER_Setup(TIM_TypeDef *TIM_Instance, QDC_Resolution Resolution);


EStatus_t QDC_Init(uint8_t ID, QDC_Parameters_t Parameter)
{
  EStatus_t  returnValue = ANSWERED_REQUEST;


  if(ID < QDC_MAX_ID)
  {
    if(QDC_List[ID].isInitialized == 0)
    {
      if( Parameter.Port < QDC_NUMBER_OF_PORTS &&
          Parameter.PullConfig < QDC_NUMBER_OF_DRIVERS &&
          Parameter.Resolution < QDC_NUMBER_OF_RESOLITIONS )
      {


        switch(Parameter.Port)
        {
        case QDC1_CH1_PE9_CH2_PE11:
          SET_MASK(RCC->APB2ENR, RCC_APB2ENR_TIM1EN); /* Enabling TIM1 clock */
          SET_MASK(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN);/* Enabling GPIOC clock */

          /* Setting as Alternate Mode */
          CLR_GPIO_MODE(GPIOE, 9);
          CLR_GPIO_MODE(GPIOE, 11);
          SET_GPIO_MODE(GPIOE, 9, GPIO_MODER_MODE_ALT_FUNC);
          SET_GPIO_MODE(GPIOE, 11, GPIO_MODER_MODE_ALT_FUNC);
          /* Setting the mode of Alternate Function */
          CLR_GPIO_AFRH(GPIOE, 9);
          CLR_GPIO_AFRH(GPIOE, 11);
          SET_GPIO_AFRH(GPIOE, 9, 1);
          SET_GPIO_AFRH(GPIOE, 11, 1);
          /* Setting the GPIO Pull config */
          CLR_GPIO_PULL(GPIOE, 9);
          CLR_GPIO_PULL(GPIOE, 11);

          if( Parameter.PullConfig == QDC_PULL_UP )
          {
            SET_GPIO_PULL(GPIOE, 9, GPIO_PUPDR_PULL_UP);
            SET_GPIO_PULL(GPIOE, 11, GPIO_PUPDR_PULL_UP);
          }
          else if( Parameter.PullConfig == QDC_PULL_DOWN )
          {
            SET_GPIO_PULL(GPIOE, 9, GPIO_PUPDR_PULL_DOWN);
            SET_GPIO_PULL(GPIOE, 11, GPIO_PUPDR_PULL_DOWN);
          }

          /* Setting the GPIO Speed */
          CLR_GPIO_SPEED(GPIOE, 9);
          CLR_GPIO_SPEED(GPIOE, 11);
          SET_GPIO_SPEED(GPIOE, 9, GPIO_OSPEED_HIGH_SPEED_100MHZ);
          SET_GPIO_SPEED(GPIOE, 11, GPIO_OSPEED_HIGH_SPEED_100MHZ);

          TIMER_Setup(TIM1, Parameter.Resolution);

          QDC_List[ID].TIMx = TIM1;
          break;

        case QDC2_CH1_PA5_CH2_PB3:
          SET_MASK(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);  /* Enabling TIM2 clock */
          SET_MASK(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);/* Enabling GPIOA clock */
          SET_MASK(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);/* Enabling GPIOB clock */

          /* Setting as Alternate Mode */
          CLR_GPIO_MODE(GPIOA, 5);
          CLR_GPIO_MODE(GPIOB, 3);
          SET_GPIO_MODE(GPIOA, 5, GPIO_MODER_MODE_ALT_FUNC);
          SET_GPIO_MODE(GPIOB, 3, GPIO_MODER_MODE_ALT_FUNC);
          /* Setting the mode of Alternate Function */
          CLR_GPIO_AFRL(GPIOA, 5);
          CLR_GPIO_AFRL(GPIOB, 3);
          SET_GPIO_AFRL(GPIOA, 5, 1);
          SET_GPIO_AFRL(GPIOB, 3, 1);
          /* Setting the GPIO Pull config */
          CLR_GPIO_PULL(GPIOA, 5);
          CLR_GPIO_PULL(GPIOB, 3);

          if( Parameter.PullConfig == QDC_PULL_UP )
          {
            SET_GPIO_PULL(GPIOA, 5, GPIO_PUPDR_PULL_UP);
            SET_GPIO_PULL(GPIOB, 3, GPIO_PUPDR_PULL_UP);
          }
          else if( Parameter.PullConfig == QDC_PULL_DOWN )
          {
            SET_GPIO_PULL(GPIOA, 5, GPIO_PUPDR_PULL_DOWN);
            SET_GPIO_PULL(GPIOB, 3, GPIO_PUPDR_PULL_DOWN);
          }

          /* Setting the GPIO Speed */
          CLR_GPIO_SPEED(GPIOA, 5);
          CLR_GPIO_SPEED(GPIOB, 3);
          SET_GPIO_SPEED(GPIOA, 5, GPIO_OSPEED_HIGH_SPEED_100MHZ);
          SET_GPIO_SPEED(GPIOB, 3, GPIO_OSPEED_HIGH_SPEED_100MHZ);

          TIMER_Setup(TIM2, Parameter.Resolution);

          QDC_List[ID].TIMx = TIM2;
          break;

        case QDC3_CH1_PA6_CH2_PA7:
          SET_MASK(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);  /* Enabling TIM3 clock */
          SET_MASK(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);/* Enabling GPIOA clock */

          /* Setting as Alternate Mode */
          CLR_GPIO_MODE(GPIOA, 6);
          CLR_GPIO_MODE(GPIOA, 7);
          SET_GPIO_MODE(GPIOA, 6, GPIO_MODER_MODE_ALT_FUNC);
          SET_GPIO_MODE(GPIOA, 7, GPIO_MODER_MODE_ALT_FUNC);
          /* Setting the mode of Alternate Function */
          CLR_GPIO_AFRL(GPIOA, 6);
          CLR_GPIO_AFRL(GPIOA, 7);
          SET_GPIO_AFRL(GPIOA, 6, 2);
          SET_GPIO_AFRL(GPIOA, 7, 2);
          /* Setting the GPIO Pull config */
          CLR_GPIO_PULL(GPIOA, 6);
          CLR_GPIO_PULL(GPIOA, 7);

          if( Parameter.PullConfig == QDC_PULL_UP )
          {
            SET_GPIO_PULL(GPIOA, 6, GPIO_PUPDR_PULL_UP);
            SET_GPIO_PULL(GPIOB, 7, GPIO_PUPDR_PULL_UP);
          }
          else if( Parameter.PullConfig == QDC_PULL_DOWN )
          {
            SET_GPIO_PULL(GPIOA, 6, GPIO_PUPDR_PULL_DOWN);
            SET_GPIO_PULL(GPIOB, 7, GPIO_PUPDR_PULL_DOWN);
          }

          /* Setting the GPIO Speed */
          CLR_GPIO_SPEED(GPIOA, 6);
          CLR_GPIO_SPEED(GPIOA, 7);
          SET_GPIO_SPEED(GPIOA, 6, GPIO_OSPEED_HIGH_SPEED_100MHZ);
          SET_GPIO_SPEED(GPIOA, 7, GPIO_OSPEED_HIGH_SPEED_100MHZ);

          TIMER_Setup(TIM3, Parameter.Resolution);

          QDC_List[ID].TIMx = TIM3;
          break;

        case QDC3_CH1_PB4_CH2_PB5:
          SET_MASK(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);  /* Enabling TIM3 clock */
          SET_MASK(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);/* Enabling GPIOB clock */

          /* Setting as Alternate Mode */
          CLR_GPIO_MODE(GPIOB, 4);
          CLR_GPIO_MODE(GPIOB, 5);
          SET_GPIO_MODE(GPIOB, 4, GPIO_MODER_MODE_ALT_FUNC);
          SET_GPIO_MODE(GPIOB, 5, GPIO_MODER_MODE_ALT_FUNC);
          /* Setting the mode of Alternate Function */
          CLR_GPIO_AFRL(GPIOB, 4);
          CLR_GPIO_AFRL(GPIOB, 5);
          SET_GPIO_AFRL(GPIOB, 4, 2);
          SET_GPIO_AFRL(GPIOB, 5, 2);
          /* Setting the GPIO Pull config */
          CLR_GPIO_PULL(GPIOB, 4);
          CLR_GPIO_PULL(GPIOB, 5);

          if( Parameter.PullConfig == QDC_PULL_UP )
          {
            SET_GPIO_PULL(GPIOB, 4, GPIO_PUPDR_PULL_UP);
            SET_GPIO_PULL(GPIOB, 5, GPIO_PUPDR_PULL_UP);
          }
          else if( Parameter.PullConfig == QDC_PULL_DOWN )
          {
            SET_GPIO_PULL(GPIOB, 4, GPIO_PUPDR_PULL_DOWN);
            SET_GPIO_PULL(GPIOB, 5, GPIO_PUPDR_PULL_DOWN);
          }

          /* Setting the GPIO Speed */
          CLR_GPIO_SPEED(GPIOB, 4);
          CLR_GPIO_SPEED(GPIOB, 5);
          SET_GPIO_SPEED(GPIOB, 4, GPIO_OSPEED_HIGH_SPEED_100MHZ);
          SET_GPIO_SPEED(GPIOB, 5, GPIO_OSPEED_HIGH_SPEED_100MHZ);

          TIMER_Setup(TIM3, Parameter.Resolution);

          QDC_List[ID].TIMx = TIM3;
          break;

        case QDC4_CH1_PD12_CH2_PD13:
          SET_MASK(RCC->APB1ENR, RCC_APB1ENR_TIM4EN); /* Enabling TIM4 clock */
          SET_MASK(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);/* Enabling GPIOD clock */

          /* Setting as Alternate Mode */
          CLR_GPIO_MODE(GPIOD, 12);
          CLR_GPIO_MODE(GPIOD, 13);
          SET_GPIO_MODE(GPIOD, 12, GPIO_MODER_MODE_ALT_FUNC);
          SET_GPIO_MODE(GPIOD, 13, GPIO_MODER_MODE_ALT_FUNC);
          /* Setting the mode of Alternate Function */
          CLR_GPIO_AFRH(GPIOD, 12);
          CLR_GPIO_AFRH(GPIOD, 13);
          SET_GPIO_AFRH(GPIOD, 12, 2);
          SET_GPIO_AFRH(GPIOD, 13, 2);
          /* Setting the GPIO Pull config */
          CLR_GPIO_PULL(GPIOD, 12);
          CLR_GPIO_PULL(GPIOD, 13);

          if( Parameter.PullConfig == QDC_PULL_UP )
          {
            SET_GPIO_PULL(GPIOD, 12, GPIO_PUPDR_PULL_UP);
            SET_GPIO_PULL(GPIOD, 13, GPIO_PUPDR_PULL_UP);
          }
          else if( Parameter.PullConfig == QDC_PULL_DOWN )
          {
            SET_GPIO_PULL(GPIOD, 12, GPIO_PUPDR_PULL_DOWN);
            SET_GPIO_PULL(GPIOD, 13, GPIO_PUPDR_PULL_DOWN);
          }

          /* Setting the GPIO Speed */
          CLR_GPIO_SPEED(GPIOD, 12);
          CLR_GPIO_SPEED(GPIOD, 13);
          SET_GPIO_SPEED(GPIOD, 12, GPIO_OSPEED_HIGH_SPEED_100MHZ);
          SET_GPIO_SPEED(GPIOD, 13, GPIO_OSPEED_HIGH_SPEED_100MHZ);

          TIMER_Setup(TIM4, Parameter.Resolution);

          QDC_List[ID].TIMx = TIM4;
          break;

        case QDC5_CH1_PA0_CH2_PA1:
          SET_MASK(RCC->APB1ENR, RCC_APB1ENR_TIM5EN);  /* Enabling TIM5 clock */
          SET_MASK(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);/* Enabling GPIOA clock */

          /* Setting as Alternate Mode */
          CLR_GPIO_MODE(GPIOA, 0);
          CLR_GPIO_MODE(GPIOA, 1);
          SET_GPIO_MODE(GPIOA, 0, GPIO_MODER_MODE_ALT_FUNC);
          SET_GPIO_MODE(GPIOA, 1, GPIO_MODER_MODE_ALT_FUNC);
          /* Setting the mode of Alternate Function */
          CLR_GPIO_AFRL(GPIOA, 0);
          CLR_GPIO_AFRL(GPIOA, 1);
          SET_GPIO_AFRL(GPIOA, 0, 2);
          SET_GPIO_AFRL(GPIOA, 1, 2);
          /* Setting the GPIO Pull config */
          CLR_GPIO_PULL(GPIOA, 0);
          CLR_GPIO_PULL(GPIOA, 1);

          if( Parameter.PullConfig == QDC_PULL_UP )
          {
            SET_GPIO_PULL(GPIOA, 0, GPIO_PUPDR_PULL_UP);
            SET_GPIO_PULL(GPIOA, 1, GPIO_PUPDR_PULL_UP);
          }
          else if( Parameter.PullConfig == QDC_PULL_DOWN )
          {
            SET_GPIO_PULL(GPIOA, 0, GPIO_PUPDR_PULL_DOWN);
            SET_GPIO_PULL(GPIOA, 1, GPIO_PUPDR_PULL_DOWN);
          }

          /* Setting the GPIO Speed */
          CLR_GPIO_SPEED(GPIOA, 0);
          CLR_GPIO_SPEED(GPIOB, 1);
          SET_GPIO_SPEED(GPIOA, 0, GPIO_OSPEED_HIGH_SPEED_100MHZ);
          SET_GPIO_SPEED(GPIOB, 1, GPIO_OSPEED_HIGH_SPEED_100MHZ);

          TIMER_Setup(TIM5, Parameter.Resolution);

          QDC_List[ID].TIMx = TIM5;
          break;

        case QDC8_CH1_PC6_CH2_PC7:
          SET_MASK(RCC->APB2ENR, RCC_APB2ENR_TIM8EN); /* Enabling TIM8 clock */
          SET_MASK(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);/* Enabling GPIOC clock */

          /* Setting as Alternate Mode */
          CLR_GPIO_MODE(GPIOC, 6);
          CLR_GPIO_MODE(GPIOC, 7);
          SET_GPIO_MODE(GPIOC, 6, GPIO_MODER_MODE_ALT_FUNC);
          SET_GPIO_MODE(GPIOC, 7, GPIO_MODER_MODE_ALT_FUNC);
          /* Setting the mode of Alternate Function */
          CLR_GPIO_AFRL(GPIOC, 6);
          CLR_GPIO_AFRL(GPIOC, 7);
          SET_GPIO_AFRL(GPIOC, 6, 3);
          SET_GPIO_AFRL(GPIOC, 7, 3);
          /* Setting the GPIO Pull config */
          CLR_GPIO_PULL(GPIOC, 6);
          CLR_GPIO_PULL(GPIOC, 7);

          if( Parameter.PullConfig == QDC_PULL_UP )
          {
            SET_GPIO_PULL(GPIOC, 6, GPIO_PUPDR_PULL_UP);
            SET_GPIO_PULL(GPIOC, 7, GPIO_PUPDR_PULL_UP);
          }
          else if( Parameter.PullConfig == QDC_PULL_DOWN )
          {
            SET_GPIO_PULL(GPIOC, 6, GPIO_PUPDR_PULL_DOWN);
            SET_GPIO_PULL(GPIOC, 7, GPIO_PUPDR_PULL_DOWN);
          }

          /* Setting the GPIO Speed */
          CLR_GPIO_SPEED(GPIOC, 6);
          CLR_GPIO_SPEED(GPIOC, 7);
          SET_GPIO_SPEED(GPIOC, 6, GPIO_OSPEED_HIGH_SPEED_100MHZ);
          SET_GPIO_SPEED(GPIOC, 7, GPIO_OSPEED_HIGH_SPEED_100MHZ);

          TIMER_Setup(TIM8, Parameter.Resolution);

          QDC_List[ID].TIMx = TIM8;
          break;

          default:
            returnValue = ERR_PARAM_VALUE;
            break;
        }


        if( returnValue == ANSWERED_REQUEST )
        {
          QDC_List[ID].Parameters = Parameter;
          QDC_List[ID].isInitialized = 1;
        }


      }
      else
      {
        returnValue = ERR_PARAM_VALUE;
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


EStatus_t QDC_Read(uint8_t ID, int32_t *Pulses )
{
  int16_t aux;
  CLEAR_MASK(QDC_List[ID].TIMx->CR1, TIM_CR1_CEN);
  aux = QDC_List[ID].TIMx->CNT; /* CNT is a 16 bits wide register */
  QDC_List[ID].TIMx->CNT = 0;
  SET_MASK(QDC_List[ID].TIMx->CR1, TIM_CR1_CEN);
  /* Type conversion intended, where *pulse must be negative is
   * aux is negative, but in a 32 bits variable.
   */
  *Pulses =  aux;
  return ANSWERED_REQUEST;
}


void TIMER_Setup(TIM_TypeDef *TIM_Instance, QDC_Resolution Resolution)
{
  /* Timer Counter Mode */
  TIM_Instance->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
  TIM_Instance->CR1 |= 0x00000000U;

  /* Clock Division */
  TIM_Instance->CR1 &= ~TIM_CR1_CKD;
  TIM_Instance->CR1 |= (uint32_t)0x00000000U;

  /* Preload */
  MODIFY_REG(TIM_Instance->CR1, TIM_CR1_ARPE, 0x00000000U);

  /* Period (setting count limit to max value possible */
  TIM_Instance->ARR = (uint32_t)0xFFFF;  /* ARR is a 16 bits wide register */

  /* Prescaler Divider and resolution*/
  switch( Resolution ){
  case QDC_X1:
    TIM_Instance->PSC = (uint32_t)1U; /* Dividing clock by 2 to support QDC_X1*/
    TIM_Instance->EGR = TIM_EGR_UG;
    TIM_Instance->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_ECE);
    TIM_Instance->SMCR |= TIM_SMCR_SMS_0; /* Same as QDC_X2 */
    break;
  case QDC_X2:
    TIM_Instance->PSC = (uint32_t)0U;
    TIM_Instance->EGR = TIM_EGR_UG;
    TIM_Instance->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_ECE);
    TIM_Instance->SMCR |= TIM_SMCR_SMS_0;
    break;
  case QDC_X4:
    TIM_Instance->PSC = (uint32_t)0U;
    TIM_Instance->EGR = TIM_EGR_UG;
    TIM_Instance->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_ECE);
    TIM_Instance->SMCR |= 3; /* There is no macro for this option */
    break;
  default:
    /* Defaulting to QDC_X4 */
    TIM_Instance->PSC = (uint32_t)0U;
    TIM_Instance->EGR = TIM_EGR_UG;
    TIM_Instance->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_ECE);
    TIM_Instance->SMCR |= 3;
    break;
  }

  TIM_Instance->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
  TIM_Instance->CCMR1 |= (TIM_CCMR1_CC1S_0 | (TIM_CCMR1_CC1S_0 << 8U));
  TIM_Instance->CCMR1 &= ~(TIM_CCMR1_IC1PSC_Msk | TIM_CCMR1_IC2PSC_Msk);
  TIM_Instance->CCMR1 &= ~(TIM_CCMR1_IC2PSC | TIM_CCMR1_IC2F);

  /* Prescaler */
  TIM_Instance->CCMR1 |= 0x00000000U | (0x00000000U << 8U);

  /* Filter */
  TIM_Instance->CCMR1 |= (0x00000000U << 4U) | (0x00000000U << 12U);

  TIM_Instance->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
  TIM_Instance->CCER &= ~(TIM_CCER_CC1NP | TIM_CCER_CC2NP);

  /* Polarity */
  TIM_Instance->CCER |= 0x00000000U | (0x00000000U << 4U);

  /* Master Trigger */
  TIM_Instance->CR2 &= ~TIM_CR2_MMS;
  TIM_Instance->CR2 |= 0x00000000U;

  /* Master Config*/
  TIM_Instance->SMCR &= ~TIM_SMCR_MSM;
  TIM_Instance->SMCR |= 0x00000000U;

  /* TIM Channel */
  TIM_Instance->CCER &= ~(TIM_CCER_CC1E << (0x0000003CU & 0x1FU));
  TIM_Instance->CCER |= (uint32_t)(0x00000001U << (0x0000003CU & 0x1FU));

  /* TIM enable */
  TIM_Instance->CR1 |= TIM_CR1_CEN;

}

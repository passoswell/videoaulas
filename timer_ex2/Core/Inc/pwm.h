/**
 * @file  pwm.h
 * @date  13-December-2020
 * @brief PWM configuration and access.
 *
 * @author
 * @author
 */


#ifndef PWM_H
#define PWM_H


#include <stdint.h>
#include "stdstatus.h"
#include "setup.h"


/**
 * @brief Max number of PWM IDs.
 */
#ifndef PWM_MAX_ID
#define PWM_MAX_ID                                                             2
#endif

/**
 * @brief List of PWM pins.
 */
typedef enum
{
  PWM1_CH1_PE9,
  PWM1_CH1N_PA7,
  PWM1_CH2_PE11,
  PWM1_CH2N_PB0,
  PWM1_CH3_PE13,
  PWM1_CH3N_PE12,
  PWM1_CH4_PE14,

  PWM2_CH1_PA0,
  PWM2_CH2_PA1,
  PWM2_CH3_PA2,
  PWM2_CH4_PA3,

  PWM3_CH1_PA6,
  PWM3_CH2_PA7,
  PWM3_CH3_PB0,
  PWM3_CH4_PB1,

  PWM4_CH1_PD12,
  PWM4_CH2_PD13,
  PWM4_CH3_PD14,
  PWM4_CH4_PD15,

  PWM4_CH1_PB6,
  PWM4_CH2_PB7,
  PWM4_CH3_PB8,
  PWM4_CH4_PB9,

  PWM5_CH1_PA0,
  PWM5_CH2_PA1,
  PWM5_CH3_PA2,

  PWM8_CH1_PC6,
  PWM8_CH1N_PA5,
  PWM8_CH2_PC7,
  PWM8_CH2N_PB0,
  PWM8_CH3_PC8,
  PWM8_CH3N_PB1,
  PWM8_CH4_PC9,

  PWM9_CH1_PE5,
  PWM9_CH2_PE6,

  PWM10_CH1_PB8,

  PWM11_CH1_PB9,

  PWM12_CH1_PB14,
  PWM12_CH2_PB15,

  PWM13_CH1_PA6,

  PWM14_CH1_PA7,

  PWM_NUMBER_OF_CHANNELS,
} PWM_Channel_t;

/**
 * @brief List ways the PWM signal transitions from high to low and low to high.
 */
typedef enum
{
  PWM_EDGE_ALIGNED   = 0, /*!< PWM signal as taught in books and Internet*/
  PWM_CENTER_ALIGNED = 1, /*!< Used when controlling motor and reading current*/
  PWM_NUMBER_OF_MODES,
} PWM_Mode_t;

/**
 * @brief List of PWM duty cycle resolutions implemented.
 */
typedef enum
{
  PWM_8BITS = 0, /*!< Duty cycle with 256 levels*/
  PWM_10BITS,    /*!< Duty cycle with 1024 levels*/
  PWM_12BITS,    /*!< Duty cycle with 4096 levels*/
  PWM_14BITS,    /*!< Duty cycle with 16384 levels*/
  PWM_16BITS,    /*!< Duty cycle with 65536 levels*/
  PWM_NUMBER_OF_RESOLUTIONS,
} PWM_Resolution_t;

/**
 * @brief .
 */
typedef enum
{
  PWM_FREQUENCY  = 0, /*!< Exact frequency will be configured*/
  PWM_RESOLUTION,     /*!< Exact resolution, faster than PWM_FREQUENCY*/
  PWM_NUMBER_OF_PREFERENCES,
} PWM_Preference_t;

typedef struct
{
  PWM_Channel_t    Channel;
  PWM_Mode_t       Mode;
  float            Frequency;
  PWM_Resolution_t Reslution;
  PWM_Preference_t Preference;
  float            Duty;
} PWM_Parameters_t;


/**
 * @brief  Routine configures a PWM output pin.
 * @param  ID : ID that should be allocated and configured.
 * @param  Parameter : desired PWM parameters.
 * @retval EStatus_t
 */
EStatus_t PWM_Init(uint8_t ID, PWM_Parameters_t Parameter);


/**
 * @brief  Routine sets a desired duty cycle.
 * @param  ID : ID that should be allocated and configured.
 * @param  Duty : A number between 0.0 and 100.0.
 * @retval EStatus_t
 */
EStatus_t PWM_SetDuty(uint8_t ID, float Duty);


/**
 * @brief  Routine configures the PWM frequency.
 * @param  ID : ID that should be allocated and configured.
 * @param  Frequency : The new frequency.
 * @retval EStatus_t
 * @note   This function works only with PWM_FREQUENCY as
 *         the selected preference because there is no way to guarantee
 *         frequency and resolution for all possible values.
 */
EStatus_t PWM_SetFrequency(uint8_t ID, float Frequency);

#endif /* PWM_H */

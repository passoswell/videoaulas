/**
 * @file  qdc.h
 * @date  11-December-2020
 * @brief Quadrature Decoder and Counter configuration and access.
 *
 * @author
 * @author
 */


#ifndef QDC_H
#define QDC_H


#include <stdint.h>
#include "stdstatus.h"
#include "setup.h"

/**
 * @brief Max number of QDC IDs.
 */
#ifndef QDC_MAX_ID
#define QDC_MAX_ID                                                             1
#endif


/**
 * @brief List of QDC ports and routed pins.
 */
typedef enum
{
  QDC1_CH1_PE9_CH2_PE11 = 0,  /*!< Using TIM1 */
  QDC2_CH1_PA5_CH2_PB3,       /*!< Using TIM2 */
  QDC3_CH1_PA6_CH2_PA7,       /*!< Using TIM3 */
  QDC3_CH1_PB4_CH2_PB5,       /*!< Using TIM3 */
  QDC4_CH1_PD12_CH2_PD13,     /*!< Using TIM4 */
  QDC5_CH1_PA0_CH2_PA1,       /*!< Using TIM5 */
  QDC8_CH1_PC6_CH2_PC7,       /*!< Using TIM8 */
  QDC_NUMBER_OF_PORTS,
} QDC_Port_t;

/**
 * @brief List of pull configurations.
 */
typedef enum
{
  QDC_PULL_DISABLED = 0,
  QDC_PULL_UP,
  QDC_PULL_DOWN,
  QDC_NUMBER_OF_DRIVERS,
}QDC_PullCfg_t;

/**
 * @brief QDC resolution, the output frequency can be 1x, 2x or 4x CH1 freq.
 */
typedef enum{
  QDC_X1 = 0, /*!<PPR is the same as in the device's datasheet*/
  QDC_X2,     /*!<PPR is the same as in the device's datasheet times 2*/
  QDC_X4,     /*!<PPR is the same as in the device's datasheet times 4*/
  QDC_NUMBER_OF_RESOLITIONS,
}QDC_Resolution;

/**
 * @brief QDC configuration structure.
 */
typedef struct
{
  QDC_Port_t          Port;
  QDC_PullCfg_t       PullConfig;
  QDC_Resolution      Resolution;
} QDC_Parameters_t;

/**
 * @brief  QDC configuration routine.
 * @param  ID : ID that should be allocated and configured.
 * @param  Parameter : set of desired configuration parameters.
 * @note
 * @retval EStatus_t
 */
EStatus_t QDC_Init(uint8_t ID, QDC_Parameters_t Parameter);


/**
 * @brief  Routine to read the QDC.
 * @param  ID : ID that should be allocated and configured.
 * @param  Pulses : number of pulses detected.
 * @note
 * @retval EStatus_t
 */
EStatus_t QDC_Read(uint8_t ID, int32_t *Pulses );

#endif /* QDC_H */

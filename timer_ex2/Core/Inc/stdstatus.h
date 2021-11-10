/**
 * @file  returncode.h
 * @date  01-November-2020
 * @brief Common execution status (EStatus) definitions.
 *
 * @author
 * @author
 */

#ifndef STDSTATUS_H
#define STDSTATUS_H

/***** FIRMWARE VERSION ***************************************************** */
#define COMMON_STDSTATUS_VER_MAJOR                                          2021
#define COMMON_STDSTATUS_VER_MINOR                                             1
#define COMMON_STDSTATUS_VER_PATCH                                             3


/* This header file contains generic return codes, designed to fit the needs  */
/*  from all projects, so that all of them can use these values to represent  */
/*  their information.                                                        */
/* It consists of an enumeration, where each value has its fixed code. The    */
/*  return values can represent 'error' or 'information' values. There is an  */
/*  item in the enumeration that separates them.                              */

/**
 * @brief List of common execution status values.
 */
typedef enum
{
/***** INFORMATION TYPE RETURN LIST *******************************************/
/* NAME --------------------------- VALUE ---- DESCRIPTION                    */
  ANSWERED_REQUEST          = 0x0000, /*!< Operation ended successfully.      */
  OPERATION_IDLE            = 0x0001, /*!< No operation running.              */
  OPERATION_RUNNING         = 0x0002, /*!< Operation still running.           */
  FUNCTION_BUSY             = 0x0003, /*!< Busy with other caller's operation.*/
/******** TYPE DELIMITER ITEM *************************************************/
  RETURN_ERROR_VALUE        = 0x8000, /*!< Error codes follow this item       */
/******* ERROR TYPE RETURN LIST ***********************************************/
  ERR_FAILED                = 0x8001, /*!< Requested operation failed.        */
  ERR_FAULT                 = 0x8002, /*!< Fault detected.                    */
  ERR_BUSY                  = 0x8003, /*!< Device is busy.                    */
  ERR_ENABLED               = 0x8004, /*!< Device is enabled.                 */
  ERR_DISABLED              = 0x8005, /*!< Device is disabled.                */
  ERR_TIMEOUT               = 0x8006, /*!< Expected time expired.             */
  ERR_OVERFLOW              = 0x8007, /*!< Buffer overflow occurred.          */
  ERR_MATH                  = 0x8008, /*!< Mathematical error.                */
  ERR_CRC                   = 0x8009, /*!< CRC error detected.                */
  ERR_CKSUM                 = 0x8010, /*!< Check sum error detected.          */
  ERR_COMMAND               = 0x8011, /*!< Invalid command or request.        */
  ERR_NULL_POINTER          = 0x8012, /*!< A unexpected null pointer was found*/
  ERR_VARIABLE_CORRUPTED    = 0x8013, /*!< Variable(s) has unexpected value   */
  ERR_LIST_ORDER            = 0x8014, /*!< List items not following the enum  */
  ERR_NOT_AVAILABLE         = 0x8015, /*!< Resource not available             */
  ERR_NOT_IMPLEMENTED       = 0x8016, /*!< Function or method not implemented */
  ERR_BUFFER_SIZE           = 0x8017, /*!< Buffer is smaller than data size   */
  ERR_REGISTER_SIZE         = 0x8018, /*!< Register is smaller than data size */
  ERR_DEVICE_NOT_FOUND      = 0x8019, /*!< Device was no found                */
  ERR_WRITE_PROTECTED       = 0x8020, /*!< Device is write protected          */
  ERR_INVALID_FILE_SYSTEM   = 0x8021, /*!< No valid file system found         */
  ERR_RESOURCE_DEPLETED     = 0x8022, /*!< Cannot alocate hardware or memory  */

  ERR_PARAM                 = 0x8080, /*!< Invalid parameter.                 */

  ERR_PARAM_ID              = 0x8081, /*!< Invalid ID.                        */
  ERR_PARAM_VALUE           = 0x8082, /*!< Invalid value.                     */
  ERR_PARAM_RANGE           = 0x8083, /*!< Invalid parameter's range          */
  ERR_PARAM_COMMAND         = 0x8084, /*!< Invalid command parameter .        */
  ERR_PARAM_NAME            = 0x8085, /*!< Name string entered is invalid     */
  ERR_PARAM_OFFSET          = 0x8086, /*!< Invalid offset                     */

} EStatus_t;


#endif /* STDSTATUS_H */



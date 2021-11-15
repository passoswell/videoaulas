#include <stdint.h>
#include "main.h"
#include "cmsis_os.h"




TaskHandle_t hledTask; /* Task handles for LED blink task */
TaskHandle_t hbuttonTask;

void ledTask(void *arg);
void buttonTask(void *arg);




void start_rtos(void)
{
  /* Create a task for LED blink */
  xTaskCreate(ledTask,    //Funcao
      "ledTask",          //Nome
      128,                //Pilha
      NULL,               //Parametro
      1,                  //Prioridade
      &hledTask);

  /* Create a task for LED blink */
  xTaskCreate(buttonTask, //Function
      "buttonTask",       //Name string
      128,                //Stack size
      NULL,               //Input parameter
      1,                  //Priority
      &hbuttonTask);      //Handler

  /* Start the scheduler */
  vTaskStartScheduler();


  while(1); /* Execution will never reach this line */
}





void ledTask(void *arg)
{
  while(1)
  {
    /* Blinking another LED*/
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);

    vTaskDelay(pdMS_TO_TICKS(100)); /* 100 ms */

    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

    vTaskDelay(pdMS_TO_TICKS(100)); /* 100 ms */
  }

  vTaskDelete(hledTask); /* Execution will never reach this line */
}


void buttonTask(void *arg)
{
  uint8_t PB1_state;
  while(1)
  {
    /* Showing in a LED the state of a button */
    PB1_state = HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin);
    if( PB1_state == 0 )
    {
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
    }
    else
    {
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
    }
    vTaskDelay(pdMS_TO_TICKS(100)); /* 100 ms */
  }

  vTaskDelete(hbuttonTask); /* Execution will never reach this line */
}

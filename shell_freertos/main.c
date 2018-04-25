/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"

#include "shell.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Task priorities. */
#define blink_task_PRIORITY (configMAX_PRIORITIES - 2)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void blink_task(void *pvParameters);

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int
main(void)
{
  /* Init board hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  LED_RED_INIT(LOGIC_LED_OFF);
  LED_GREEN_INIT(LOGIC_LED_OFF);
  LED_BLUE_INIT(LOGIC_LED_OFF);

  if (xTaskCreate(blink_task, "Blink_task", configMINIMAL_STACK_SIZE + 10, NULL, blink_task_PRIORITY, NULL) != pdPASS)
  {
    PRINTF("Task creation failed!.\r\n");
    while (1)
      ;
  }

  shell_init();

  vTaskStartScheduler();
  for (;;)
    ;
}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
static void
blink_task(void *pvParameters)
{
  uint32_t    delay = (100 / portTICK_PERIOD_MS);

  for (;;)
  {
    LED_RED_ON();
    vTaskDelay(delay);
    LED_RED_OFF();
    vTaskDelay(delay);

    LED_GREEN_ON();
    vTaskDelay(delay);
    LED_GREEN_OFF();
    vTaskDelay(delay);

    LED_BLUE_ON();
    vTaskDelay(delay);
    LED_BLUE_OFF();
    vTaskDelay(delay);
  }
}

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "sys_tick.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int
main(void)
{
  uint32_t delay = 100;
  /* Init board hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  sys_tick_init();

  PRINTF("blinky.\r\n");

  LED_RED_INIT(LOGIC_LED_OFF);
  LED_GREEN_INIT(LOGIC_LED_OFF);
  LED_BLUE_INIT(LOGIC_LED_OFF);

  while (1)
  {
    LED_RED_ON();
    sys_tick_delay(delay);
    LED_RED_OFF();
    sys_tick_delay(delay);

    LED_GREEN_ON();
    sys_tick_delay(delay);
    LED_GREEN_OFF();
    sys_tick_delay(delay);

    LED_BLUE_ON();
    sys_tick_delay(delay);
    LED_BLUE_OFF();
    sys_tick_delay(delay);
  }
}

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "sys_tick.h"

#include "event_dispatcher.h"
#include "mainloop_timer.h"
#include "blinky.h"
#include "shell.h"

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
  /* Init board hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  event_dispatcher_init();
  mainloop_timer_init();
  sys_tick_init();

  PRINTF("blinky.\r\n");

  blinky_init();
  shell_init();

  while (1)
  {
    event_dispatcher_dispatch();
  }
}

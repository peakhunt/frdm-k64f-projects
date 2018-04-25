#include <stdint.h>
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "board.h"

/* Initialize debug console. */
void BOARD_InitDebugConsole(void)
{
#if 0
  uint32_t uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;

  DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartClkSrcFreq);
#endif
}

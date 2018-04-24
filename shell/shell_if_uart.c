#include "app_common.h"
#include "shell_if_uart.h"
#include "circ_buffer.h"
#include "event_list.h"
#include "event_dispatcher.h"

#include "fsl_uart.h"

////////////////////////////////////////////////////////////////////////////////
//
// private definitions
//
////////////////////////////////////////////////////////////////////////////////
#define USART_IRQ_READ_LENGTH           1

#if 1
#define SHELL_UART          UART0
#define SHELL_UART_CLKSRC   UART0_CLK_SRC
#define SHELL_UART_CLK_FREQ CLOCK_GetFreq(UART0_CLK_SRC)
#define SHELL_UART_IRQn UART0_RX_TX_IRQn
#define SHELL_UART_IRQHandler UART0_RX_TX_IRQHandler
#else
#define SHELL_UART          UART3
#define SHELL_UART_CLKSRC   UART3_CLK_SRC
#define SHELL_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define SHELL_UART_IRQn UART3_RX_TX_IRQn
#define SHELL_UART_IRQHandler UART3_RX_TX_IRQHandler
#endif

////////////////////////////////////////////////////////////////////////////////
//
// private variables
//
////////////////////////////////////////////////////////////////////////////////
/*
static UART_HandleTypeDef*    _huart = &huart1;
static IRQn_Type              _irqn  = USART1_IRQn;

static uint8_t                _usart_irq_buffer[USART_IRQ_READ_LENGTH];
*/
static ShellIntf              _shell_uart_if;
static CircBuffer             _rx_cb;
static volatile uint8_t       _rx_buffer[CLI_RX_BUFFER_LENGTH];

////////////////////////////////////////////////////////////////////////////////
//
// RX IRQ
//
////////////////////////////////////////////////////////////////////////////////
void
SHELL_UART_IRQHandler(void)
{
  uint8_t data;

  /* If new data arrived. */
  if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(SHELL_UART))
  {
    data = UART_ReadByte(SHELL_UART);

    if(circ_buffer_enqueue(&_rx_cb, &data, 1, true) == false)
    {
      // fucked up. overflow mostly.
      // do something here
    }

    event_set(1 << DISPATCH_EVENT_UART_CLI_RX);
  }

#if defined __CORTEX_M && (__CORTEX_M == 4U)
  __DSB();
#endif
}

////////////////////////////////////////////////////////////////////////////////
//
// private utilities
//
////////////////////////////////////////////////////////////////////////////////
static void
shell_if_uart_config_uart(void)
{
  uart_config_t   config;

  UART_GetDefaultConfig(&config);

  config.baudRate_Bps = 115200;
  config.enableTx = true;
  config.enableRx = true;

  UART_Init(SHELL_UART, &config, SHELL_UART_CLK_FREQ);

}

static void
shell_if_uart_enter_critical(CircBuffer* cb)
{
  NVIC_DisableIRQ(SHELL_UART_IRQn);
}

static void
shell_if_uart_leave_critical(CircBuffer* cb)
{
  NVIC_EnableIRQ(SHELL_UART_IRQn);
}

////////////////////////////////////////////////////////////////////////////////
//
// callbacks for core shell and rx interrupt
//
////////////////////////////////////////////////////////////////////////////////
static uint8_t
shell_if_uart_get_rx_data(ShellIntf* intf, uint8_t* data)
{
  if(circ_buffer_dequeue(&_rx_cb, data, 1, false) == false)
  {
    return false;
  }
  return true;
}

static void
shell_if_uart_put_tx_data(ShellIntf* intf, uint8_t* data, uint16_t len)
{
  // XXX blocking operation. should be improved later
  UART_WriteBlocking(SHELL_UART, data, len);
}

static void
shell_if_uart_event_handler(uint32_t event)
{
  shell_handle_rx(&_shell_uart_if);
}

////////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
////////////////////////////////////////////////////////////////////////////////
void
shell_if_uart_init(void)
{
  _shell_uart_if.cmd_buffer_ndx    = 0;
  _shell_uart_if.get_rx_data       = shell_if_uart_get_rx_data;
  _shell_uart_if.put_tx_data       = shell_if_uart_put_tx_data;

  INIT_LIST_HEAD(&_shell_uart_if.lh);

  circ_buffer_init(&_rx_cb, _rx_buffer, CLI_RX_BUFFER_LENGTH,
      shell_if_uart_enter_critical,
      shell_if_uart_leave_critical);

  shell_if_uart_config_uart();

  event_register_handler(shell_if_uart_event_handler, DISPATCH_EVENT_UART_CLI_RX);
  shell_if_register(&_shell_uart_if);

  UART_EnableInterrupts(SHELL_UART, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
  EnableIRQ(SHELL_UART_IRQn);
}

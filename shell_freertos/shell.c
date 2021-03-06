#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>


#include "shell.h"
#include "fsl_debug_console.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"

////////////////////////////////////////////////////////////////////////////////
//
// private definitions
//
////////////////////////////////////////////////////////////////////////////////

#define SHELL_MAX_COLUMNS_PER_LINE      128
#define SHELL_COMMAND_MAX_ARGS          4

#define SHELL_UART_IF_RX_EVENT          (1 << 0)

#define VERSION       "FRDM-K64 Shell V0.2a"

typedef void (*shell_command_handler)(ShellIntf* intf, int argc, const char** argv);

typedef struct
{
  const char*           command;
  const char*           description;
  shell_command_handler handler;
} ShellCommand;

extern void shell_if_uart_event_handler(uint32_t event);

////////////////////////////////////////////////////////////////////////////////
//
// private prototypes
//
////////////////////////////////////////////////////////////////////////////////
static void shell_command_help(ShellIntf* intf, int argc, const char** argv);
static void shell_command_version(ShellIntf* intf, int argc, const char** argv);

////////////////////////////////////////////////////////////////////////////////
//
// private variables
//
////////////////////////////////////////////////////////////////////////////////
const uint8_t                 _welcome[] = "\r\n**** Welcome ****\r\n";
const uint8_t                 _prompt[]  = "\r\nFRDM-K64F> ";

static char                   _print_buffer[SHELL_MAX_COLUMNS_PER_LINE + 1];

static EventGroupHandle_t     _shell_if_event_group;

static LIST_HEAD(_shell_intf_list);

static ShellCommand     _commands[] = 
{
  {
    "help",
    "show this command",
    shell_command_help,
  },
  {
    "version",
    "show version",
    shell_command_version,
  },
};

////////////////////////////////////////////////////////////////////////////////
//
// shell utilities
//
////////////////////////////////////////////////////////////////////////////////
static inline void
shell_prompt(ShellIntf* intf)
{
  intf->put_tx_data(intf, (uint8_t*)_prompt, sizeof(_prompt) -1);
}

////////////////////////////////////////////////////////////////////////////////
//
// shell command handlers
//
////////////////////////////////////////////////////////////////////////////////
static void
shell_command_help(ShellIntf* intf, int argc, const char** argv)
{
  size_t i;

  shell_printf(intf, "\r\n");

  for(i = 0; i < sizeof(_commands)/sizeof(ShellCommand); i++)
  {
    shell_printf(intf, "%-20s: ", _commands[i].command);
    shell_printf(intf, "%s\r\n", _commands[i].description);
  }
}

static void
shell_command_version(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "\r\n");
  shell_printf(intf, "%s\r\n", VERSION);
}

////////////////////////////////////////////////////////////////////////////////
//
// shell core
//
////////////////////////////////////////////////////////////////////////////////
static void
shell_execute_command(ShellIntf* intf, char* cmd)
{
  static const char*    argv[SHELL_COMMAND_MAX_ARGS];
  int                   argc = 0;
  size_t                i;
  char                  *s, *t;

  while((s = strtok_r(argc  == 0 ? cmd : NULL, " \t", &t)) != NULL)
  {
    if(argc >= SHELL_COMMAND_MAX_ARGS)
    {
      shell_printf(intf, "\r\nError: too many arguments\r\n");
      return;
    }
    argv[argc++] = s;
  }

  if(argc == 0)
  {
    return;
  }

  for(i = 0; i < sizeof(_commands)/sizeof(ShellCommand); i++)
  {
    if(strcmp(_commands[i].command, argv[0]) == 0)
    {
      shell_printf(intf, "\r\nExecuting %s\r\n", argv[0]);
      _commands[i].handler(intf, argc, argv);
      return;
    }
  }
  shell_printf(intf, "%s", "\r\nUnknown Command: ");
  shell_printf(intf, "%s", argv[0]);
  shell_printf(intf, "%s", "\r\n");
}


void
shell_printf(ShellIntf* intf, const char* fmt, ...)
{
  va_list   args;
  int       len;

  va_start(args, fmt);
  len = vsnprintf(_print_buffer, SHELL_MAX_COLUMNS_PER_LINE, fmt, args);
  va_end(args);

  intf->put_tx_data(intf, (uint8_t*)_print_buffer, len);
}


////////////////////////////////////////////////////////////////////////////////
//
// shell task
//
////////////////////////////////////////////////////////////////////////////////
static void
shell_task(void* param)
{
  extern void shell_if_uart_init(void);
  EventBits_t   uxBits;
  uint32_t      dummy = 0;

  _shell_if_event_group = xEventGroupCreate();

  shell_if_uart_init();
  //shell_if_usb_init();

  while(1)
  {
    uxBits = xEventGroupWaitBits(
        _shell_if_event_group,
        SHELL_UART_IF_RX_EVENT,
        pdTRUE,
        pdFALSE,
        portMAX_DELAY);
    //vTaskDelay(1000/portTICK_PERIOD_MS);

    if((uxBits & SHELL_UART_IF_RX_EVENT) != 0)
    {
      shell_if_uart_event_handler(dummy);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// public interface
//
////////////////////////////////////////////////////////////////////////////////
void
shell_init(void)
{
  if(xTaskCreate(shell_task, "shell_task", configMINIMAL_STACK_SIZE + 128, NULL, configMAX_PRIORITIES - 2, NULL) != pdPASS)
  {
    PRINTF("shell task creationfailed!.\r\n");
    while(1)
      ;
  }
}

void
shell_notify_uart_rx_event_from_isr(void)
{
  BaseType_t xHigherPriorityTaskWoken,
             xResult;

  xHigherPriorityTaskWoken = pdFALSE;

  xResult = xEventGroupSetBitsFromISR(_shell_if_event_group,
                                      SHELL_UART_IF_RX_EVENT,
                                      &xHigherPriorityTaskWoken);

  // what happens if this fails?
  if(xResult != pdFAIL)
  {
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

/*
void
shell_start(void)
{
  ShellIntf* intf;

  list_for_each_entry(intf, &_shell_intf_list, lh)
  {
    intf->put_tx_data(intf, (uint8_t*)_welcome, sizeof(_welcome) -1);
    shell_prompt(intf);
  }
}
*/


void
shell_if_register(ShellIntf* intf)
{
  list_add_tail(&intf->lh, &_shell_intf_list);
}

void
shell_handle_rx(ShellIntf* intf)
{
  uint8_t   b;

  while(1)
  {
    if(intf->get_rx_data(intf, &b) == false)
    {
      return;
    }

    if(b != '\r' && intf->cmd_buffer_ndx < SHELL_MAX_COMMAND_LEN)
    {
      if(b == '\b' || b == 0x7f)
      {
        if(intf->cmd_buffer_ndx > 0)
        {
          shell_printf(intf, "%c%c%c", b, 0x20, b);
          intf->cmd_buffer_ndx--;
        }
      }
      else
      {
        shell_printf(intf, "%c", b);
        intf->cmd_buffer[intf->cmd_buffer_ndx++] = b;
      }
    }
    else if(b == '\r')
    {
      intf->cmd_buffer[intf->cmd_buffer_ndx++] = '\0';

      shell_execute_command(intf, (char*)intf->cmd_buffer);

      intf->cmd_buffer_ndx = 0;
      shell_prompt(intf);
    }
  }
}

struct list_head*
shell_get_intf_list(void)
{
  return &_shell_intf_list;
}

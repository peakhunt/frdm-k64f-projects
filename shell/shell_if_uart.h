#ifndef __SHELL_IF_UART_DEF_H__
#define __SHELL_IF_UART_DEF_H__

#include "shell.h"

extern void shell_if_uart_init(void);
extern void uart_read_callback(void* huart, bool error);

#endif //!__SHELL_IF_UART_DEF_H__

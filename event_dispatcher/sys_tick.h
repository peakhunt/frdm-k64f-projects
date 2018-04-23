/*
 * quite amazing Kinetis SDK doesn't come with this simple delay
 *
 */
#ifndef __SYS_TICK_DEF_H__
#define __SYS_TICK_DEF_H__

#include <stdint.h>

extern void sys_tick_init(void);
extern uint32_t sys_tick_get(void);
extern void sys_tick_delay(volatile uint32_t delay);

#endif /* !__SYS_TICK_DEF_H__ */

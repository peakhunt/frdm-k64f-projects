#include <stdint.h>
#include "sys_tick.h"
#include "fsl_common.h"

#include "event_list.h"
#include "event_dispatcher.h"

#define HWTIMER_PERIOD      1000U       // 1ms tick

/////////////////////////////////////////////////
//
// globals
//
/////////////////////////////////////////////////
static volatile uint32_t      _sys_tick = 0;

/////////////////////////////////////////////////
//
// systick IRQ handler
//
/////////////////////////////////////////////////
void SysTick_Handler(void)
{
  _sys_tick++;
  event_set(1 << DISPATCH_EVENT_TIMER_TICK);
}   

/////////////////////////////////////////////////
//
// public interfaces
//
/////////////////////////////////////////////////
void
sys_tick_init(void)
{
  SysTick_Config(SystemCoreClock / HWTIMER_PERIOD);
}

uint32_t
sys_tick_get(void)
{
  return _sys_tick;
}

void
sys_tick_delay(volatile uint32_t delay)
{
  uint32_t tickstart = 0;

  tickstart = sys_tick_get();

  while((sys_tick_get() - tickstart) < delay)
  {
  }
}

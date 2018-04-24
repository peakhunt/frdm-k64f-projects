#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "sys_tick.h"

#include "mainloop_timer.h"

#define BLINKY_INTERVAL         100

static SoftTimerElem    _blinky_timer;
static int              _led_ndx = 0;

static void
blinky_callback(SoftTimerElem* te)
{
  switch(_led_ndx)
  {
  case 0:
    LED_RED_ON();
    break;

  case 1:
    LED_RED_OFF();
    break;

  case 2:
    LED_GREEN_ON();
    break;

  case 3:
    LED_GREEN_OFF();
    break;

  case 4:
    LED_BLUE_ON();
    break;

  case 5:
    LED_BLUE_OFF();
    break;
  }

  if(_led_ndx >= 5)
  {
    _led_ndx = 0;
  }
  else
  {
    _led_ndx++;
  }

  mainloop_timer_schedule(&_blinky_timer, BLINKY_INTERVAL);
}


void
blinky_init(void)
{
  LED_RED_INIT(LOGIC_LED_OFF);
  LED_GREEN_INIT(LOGIC_LED_OFF);
  LED_BLUE_INIT(LOGIC_LED_OFF);

  soft_timer_init_elem(&_blinky_timer);
  _blinky_timer.cb    = blinky_callback;
  mainloop_timer_schedule(&_blinky_timer, BLINKY_INTERVAL);
}

/*
  servo_spindle.c - nozzle servo on the spindle PWM pin (pick-and-place bench, HT5023 project)

  Grbl uses all three timers of the Uno, Timer2 only for the spindle PWM on D11 (OC2A). A hobby
  servo needs a 0.5-2.5 ms pulse every ~20 ms, so Timer2 is used for that instead:

  - normal mode, 1/32 prescaler: 2 us per tick, the counter wraps every 256 ticks (512 us);
  - both edges of a pulse are made by the compare hardware ("set OC2A on match", then "clear OC2A
    on match"), so a late interrupt never changes the pulse width;
  - the overflow interrupt (every 512 us) only arms the next edge. The start tick is chosen so the
    pulse always ends on tick 128 or 192, and starts on tick 64 or later: an edge is never armed
    less than 128 us before it happens.
*/

#ifdef SERVO_HOST_TEST
  #include "servo_host_test.h"
#else
  #include "grbl.h"
#endif

#if !defined(SERVO_HOST_TEST) && (!defined(VARIABLE_SPINDLE) || defined(USE_SPINDLE_DIR_AS_ENABLE_PIN))
  #error "SERVO_SPINDLE needs VARIABLE_SPINDLE without USE_SPINDLE_DIR_AS_ENABLE_PIN (D11 = OC2A)"
#endif

#ifndef SERVO_FORCE_COMPARE
  #define SERVO_FORCE_COMPARE() (TCCR2B |= (1<<FOC2A))  // apply the COM2A action now
#endif

static volatile uint16_t servo_ticks = SERVO_MIN_US / SERVO_TICK_US;  // pulse width in ticks
static volatile uint8_t servo_run = 0;
static uint8_t servo_wrap = 0;       // the next wrap of the frame (ISR only)
static uint8_t servo_end_wrap = 0;   // wrap in which the running pulse ends; 0 = no pulse
static uint8_t servo_end_tick = 0;


ISR(TIMER2_OVF_vect)
{
  if (servo_wrap == 0) {
    servo_end_wrap = 0;
    if (servo_run) {
      uint16_t p = servo_ticks;                     // 250 ... 1250 ticks
      uint8_t s = (uint8_t)(128 - (uint8_t)p);      // so the pulse ends on tick 128 ...
      if (s < 64) { s += 64; }                      // ... or on 192
      uint16_t end = s + p;
      servo_end_wrap = (uint8_t)(end >> 8);         // >= 1, because p >= 250 and s >= 64
      servo_end_tick = (uint8_t)end;
      OCR2A = s;
      TCCR2A = (1<<COM2A1) | (1<<COM2A0);           // rising edge on tick s
    }
  } else if (servo_end_wrap) {
    if (servo_wrap == servo_end_wrap) {
      OCR2A = servo_end_tick;
      TCCR2A = (1<<COM2A1);                         // falling edge on servo_end_tick
    } else if (servo_wrap == servo_end_wrap + 1) {
      TCCR2A = 0;                                   // pin back to its port bit: low
      servo_end_wrap = 0;
    }
  }
  if (++servo_wrap >= SERVO_FRAME_WRAPS) { servo_wrap = 0; }
}


void servo_init()
{
  servo_run = 0;
  servo_end_wrap = 0;
  SPINDLE_PWM_PORT &= ~(1<<SPINDLE_PWM_BIT);        // low whenever the timer does not drive it
  SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT);
  TCCR2A = 0;                                       // normal mode, output not connected
  TCCR2B = (1<<CS21) | (1<<CS20);                   // 1/32 prescaler: 2 us per tick
  TIMSK2 = (1<<TOIE2);                              // overflow interrupt every 512 us
}


void servo_set_us(float us)
{
  if (us < SERVO_MIN_US) { us = SERVO_MIN_US; }
  if (us > SERVO_MAX_US) { us = SERVO_MAX_US; }
  uint16_t t = (uint16_t)(us / SERVO_TICK_US + 0.5);
  uint8_t sreg = SREG;
  cli();
  servo_ticks = t;
  SREG = sreg;
}


void servo_start() { servo_run = 1; }


void servo_stop()
{
  uint8_t sreg = SREG;
  cli();
  servo_run = 0;
  servo_end_wrap = 0;
  TCCR2A = (1<<COM2A1);                             // "clear on match" ...
  SERVO_FORCE_COMPARE();                            // ... forced now: the pin goes low at once
  TCCR2A = 0;
  SREG = sreg;
}


uint8_t servo_running() { return servo_run; }

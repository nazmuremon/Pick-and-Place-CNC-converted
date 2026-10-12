/*
  servo_spindle.h - nozzle servo on the spindle PWM pin (pick-and-place bench, HT5023 project)

  M3 S<us> sends servo pulses of <us> microseconds (500-2500) on D11, every 20.48 ms.
  M5, a reset or an alarm stops the pulses (the servo goes limp).
*/

#ifndef servo_spindle_h
#define servo_spindle_h

#define SERVO_TICK_US      2      // Timer2 at 1/32 of 16 MHz
#define SERVO_MIN_US       500
#define SERVO_MAX_US       2500
#define SERVO_FRAME_WRAPS  40     // 40 x 512 us = 20.48 ms between pulses (48.8 Hz)

void servo_init();
void servo_set_us(float us);   // pulse width for the next frames (clamped to 500-2500 us)
void servo_start();
void servo_stop();             // ends a running pulse at once and stops the pulses
uint8_t servo_running();

#endif

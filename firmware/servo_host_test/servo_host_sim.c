/*
  A model of the ATmega328P Timer2 in normal mode, driving servo_spindle.c on the PC.

  Reads commands from stdin, one per line:
    us <microseconds>      servo_set_us()
    start | stop           servo_start() / servo_stop()
    run <ticks> <latency>  run the timer; each overflow interrupt starts 0..<latency> ticks late
  Prints every edge of the D11 pin: "rise <tick>" / "fall <tick>" (1 tick = 2 us).
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "servo_host_test.h"

uint8_t TCCR2A, TCCR2B, OCR2A, TIMSK2, SREG, PORTB, DDRB;

static uint8_t latch = 0;   /* the OC2A output latch */
static uint8_t tcnt = 0;
static long tick = 0;
static int last_pin = 0;

static void compare_action(void)
{
  switch ((TCCR2A >> 6) & 3) {
    case 1: latch ^= 1; break;
    case 2: latch = 0; break;
    case 3: latch = 1; break;
    default: break;
  }
}

void host_force_compare(void) { compare_action(); }

static int pin(void) { return ((TCCR2A >> 6) & 3) ? latch : ((PORTB >> 3) & 1); }

static void report(void)
{
  int p = pin();
  if (p != last_pin) { printf("%s %ld\n", p ? "rise" : "fall", tick); last_pin = p; }
}

int main(void)
{
  char cmd[32];
  double value;
  long n, latency;
  int pending = 0;
  long due = 0;
  srand(7);
  servo_init();
  report();
  while (scanf("%31s", cmd) == 1) {
    if (!strcmp(cmd, "us") && scanf("%lf", &value) == 1) { servo_set_us((float)value); }
    else if (!strcmp(cmd, "start")) { servo_start(); }
    else if (!strcmp(cmd, "stop")) { servo_stop(); report(); }
    else if (!strcmp(cmd, "run") && scanf("%ld %ld", &n, &latency) == 2) {
      for (long i = 0; i < n; i++) {
        tick++;
        tcnt++;
        if (tcnt == 0) { pending = 1; due = tick + (latency > 0 ? rand() % (latency + 1) : 0); }
        if (tcnt == OCR2A) { compare_action(); }
        report();
        if (pending && tick >= due) { pending = 0; TIMER2_OVF_vect(); report(); }
      }
    }
  }
  return 0;
}

/* Stand-ins for the AVR registers, so servo_spindle.c runs on the PC (tests/test_servo_firmware.py). */
#ifndef servo_host_test_h
#define servo_host_test_h

#include <stdint.h>

extern uint8_t TCCR2A, TCCR2B, OCR2A, TIMSK2, SREG, PORTB, DDRB;

#define SPINDLE_PWM_PORT PORTB
#define SPINDLE_PWM_DDR  DDRB
#define SPINDLE_PWM_BIT  3
#define COM2A1 7
#define COM2A0 6
#define FOC2A  7
#define CS21   1
#define CS20   0
#define TOIE2  0

#define ISR(vector) void vector(void)
#define cli() ((void)0)

void host_force_compare(void);
#define SERVO_FORCE_COMPARE() host_force_compare()

#include "servo_spindle.h"

void TIMER2_OVF_vect(void);

#endif

#include "config.h"
#include "timer1.h"

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

void (*timer1_overflow_callback)(void);
void (*timer1_compa_callback)(void);

ISR(TIMER1_OVF_vect)
{
  if (timer1_overflow_callback)
    timer1_overflow_callback();
}

ISR(TIMER1_COMPA_vect)
{
  if (timer1_compa_callback)
    timer1_compa_callback();
}

void timer1_reset()
{
  TCNT1=0;
}

void timer1_set_back_tics(uint16_t delayTics)
{
  if (delayTics > timer1_period_tics)
    TCNT1 = 0;
  else
    TCNT1 = timer1_period_tics - delayTics;
}

void timer1_init(void)
{
  TCCR1B = _BV(CS10) | _BV(CS11); // prescaler = 1/64, 4us per tic, 262ms cycle
  
  OCR1A = timer1_period_tics;   

}

void timer1_set_overflow_callback(void (*callback)(void))
{
  timer1_overflow_callback = callback;
  if (timer1_overflow_callback)
    TIMSK1 |= _BV(TOIE1);
  else
    TIMSK1 &= ~(_BV(TOIE1));
}

void timer1_set_compa_callback(void (*callback)(void))
{
  timer1_compa_callback = callback;
  if (timer1_compa_callback)
    TIMSK1 |= _BV(OCIE1A);
  else
    TIMSK1 &= ~(_BV(OCIE1A));
}

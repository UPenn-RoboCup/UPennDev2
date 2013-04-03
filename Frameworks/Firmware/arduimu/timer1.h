#ifndef __TIMER1_H
#define __TIMER1_H

void timer1_init(void);
void timer1_set_overflow_callback(void (*callback)(void));
void timer1_set_compa_callback(void (*callback)(void));
void timer1_set_back_tics(uint16_t delayTics);
void timer1_reset();

#endif // __TIMER1_H

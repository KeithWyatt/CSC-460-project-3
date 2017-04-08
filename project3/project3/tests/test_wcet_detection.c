#include <avr/io.h>
#define F_CPU 16000000
#include <util/delay.h>
#include "../os.h"

/*
This test creates a time-based task which violates its WCET. This should error.
*/


void Task_P2()
{
  PORTA |= (1<<PA1);
  _delay_ms(500);
  PORTA &= ~(1<<PA1);
}

void Task_P1()
{
  for(;;) {
    PORTA |= (1<<PA2);
    _delay_ms(200);
    Task_Create_System(Task_P2, 0);
    _delay_ms(300);
    PORTA &= ~(1<<PA2);
    Task_Next();
  }
}


void a_main()
{
	DDRA |= (1<<PA1);
    DDRA |= (1<<PA2);
    Task_Create_Period(Task_P1, 0, 100, 80, 10);
}

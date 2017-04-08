#include <avr/io.h>
#define F_CPU 16000000
#include <util/delay.h>
#include "../os.h"

/*
This test creates a RR task and a time-based task.
The expected behaviour is for the time task to pre-empt the RR task, run
to completion, and then for the RR task to resume.
*/



void Task_P2()
{
  for(;;)
  {
    PORTA |= (1<<PA1);
    _delay_ms(100);
    PORTA &= ~(1<<PA1);
    Task_Next();
  }
}

void Task_P1()
{
    PORTA |= (1<<PA2);
    _delay_ms(500);
    PORTA &= ~(1<<PA2);
}

void a_main()
{
    DDRA |= (1<<PA2);
    DDRA |= (1<<PA1);
	PORTA = 0;
    Task_Create_RR(Task_P1, 0);
    Task_Create_Period(Task_P2, 0, 100, 80, 1); // sleep 100ms

}

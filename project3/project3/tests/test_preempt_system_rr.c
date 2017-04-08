#include <avr/io.h>
#define F_CPU 16000000
#include <util/delay.h>
#include "../os.h"

/*
This test creates a RR task which creates a system task, and then delays.
The expected behaviour is for the system task to pre-empt the time-based task, run
to completion, and then for the RR task to resume.
*/



void Task_P2()
{
    PORTA |= (1<<PA1);
    _delay_ms(100);
    PORTA &= ~(1<<PA1);
}

void Task_P1()
{
  for(;;) {
    PORTA |= (1<<PA2);
    Task_Create_System(Task_P2, 0); // sleep 100ms
    _delay_ms(500);
    PORTA &= ~(1<<PA2);
    _delay_ms(500);
  }
}

void Task_P3()
{
  for(;;){
    PORTA |= (1<<PA0);
    _delay_ms(300);
    PORTA &= ~(1<<PA0);
    _delay_ms(300);
  }
}

void a_main()
{
    DDRA |= (1<<PA0);
    DDRA |= (1<<PA2);
    DDRA |= (1<<PA1);
    Task_Create_RR(Task_P1, 0);
    Task_Create_RR(Task_P3, 0);
}

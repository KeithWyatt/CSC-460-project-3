#include <avr/io.h>
#define F_CPU 16000000
#include <util/delay.h>
#include "../os.h"

/*
This test creates 2 time-based tasks which will eventually conflict. When this conflict
happens, we should abort the OS with an error code (5).
*/



void Task_P2()
{
  for(;;){
    PORTA |= (1<<PA1);
    _delay_ms(100);
    PORTA &= ~(1<<PA1);
    Task_Next();
  }
}

void Task_P1()
{
  for(;;) {
    PORTA |= (1<<PA2);
    _delay_ms(100);
    PORTA &= ~(1<<PA2);
    Task_Next();
  }
}


void a_main()
{
    DDRA |= (1<<PA2);
    DDRA |= (1<<PA1);
    Task_Create_Period(Task_P1, 0, 100, 15, 10);
    Task_Create_Period(Task_P2, 0, 50, 15, 60);
}

#include <avr/io.h>
#define F_CPU 16000000
#include <util/delay.h>
#include "../os.h"

/*
This test creates a time-based task which creates a system task, and then delays.
The expected behaviour is for the system task to pre-empt the time-based task, run
to completion, and then for the time-based task to resume, and complete.
*/



void Task_P2()
{
    PORTA |= (1<<PA1);
    _delay_ms(100);
    PORTA &= ~(1<<PA1);
    //Task_Terminate();
}

void Task_P1()
{
	for(;;) {
		PORTA |= (1<<PA2);
		Task_Create_System(Task_P2, 0); // sleep 100ms
		_delay_ms(500);
		PORTA &= ~(1<<PA2);
		Task_Next();
	}
}

void a_main()
{
    DDRA |= (1<<PA2);
    DDRA |= (1<<PA1);
    Task_Create_Period(Task_P1, 0, 100, 80, 1);
    //Task_Terminate();
}

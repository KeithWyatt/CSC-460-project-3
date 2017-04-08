/*
base.c
March 27th 2017
Keith Wyatt & Becky Crouteau

*/

#define F_CPU 16000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "../UART/uart.h"
#include "../os.h"


#define LIGHTPIN PC0		// pin 37
#define LIGHTPIN2 PC1	// pin 36

uint8_t LASER = 2;
uint8_t SERVOX = 3;
uint8_t SERVOY = 4;
uint8_t ROOMBA = 5;
uint8_t AUTO = 6;

volatile char prevRoomba = 'X';

void LaserTask();
void StartADC();
void servoTask();
void RoombaTask();
void autoTask();

/*======================= Toggle Pins ================================*/

void EnableLaserPin()
{
	// pin 37
	PORTC |= _BV(LIGHTPIN);
}
void DisableLaserPin()
{
	PORTC &= ~_BV(LIGHTPIN);
}

void EnableLaserPin2()
{
	// pin 36
	PORTC |= _BV(LIGHTPIN2);
}
void DisableLaserPin2()
{
	PORTC &= ~_BV(LIGHTPIN2);
}


void LaserTask(){

	uint8_t laser = 0;
	uint8_t oldLaser = 0;
	
	DDRB |= (0<<DDB0);
	PORTB |= (1<<PORTB0);

	for(;;){
		// reads from pin PB0 (53)
		// set to 1 if button is pushed
		// set to 0 if button not pushed
		if( (PINB & _BV(PB0))){
			laser = 0;
	//		DisableLaserPin();
		}else{
			
			laser = 1;
		}
		if( laser != oldLaser){
	//		EnableLaserPin();
			Bluetooth_Send_Byte(LASER);
			Bluetooth_Send_Byte(laser);
		}
		oldLaser = laser;
		Task_Next();
	}
}

void autoTask(){

	uint8_t autonomous = 0;
	uint8_t oldAutonomous = 0;
	uint8_t counter = 0;
	uint8_t flag = 0;
	
	DDRB |= (0<<DDB0);
	PORTB |= (1<<PORTB1);

	for(;;){
		// reads from pin PB1 (52)
		// set to 1 if button is pushed
		// set to 0 if button not pushed
		if( (PINB & _BV(PB1))){
			autonomous = 0;
			counter = 0;
			DisableLaserPin();
		}else{
			autonomous = 1;	
		}
		if(autonomous == 1 && counter == 0){
			flag = 1;
			if(oldAutonomous == 1){
				EnableLaserPin();
				autonomous = 0;
				oldAutonomous = 0;
				flag = 0;
			}
			
			Bluetooth_Send_Byte(AUTO);
			Bluetooth_Send_Byte(autonomous);
			counter++;
			if(flag){
				oldAutonomous++;
			}
		}
		Task_Next();
	}
}


void RoombaTask(){
	
//	DisableLaserPin();

	
	uint8_t lowX = 0;
	uint8_t highX = 0;
	
	uint8_t lowY = 0;
	uint8_t highY = 0;
	
	uint16_t roombaX = 0;
	uint16_t roombaY = 0;
	
	int ch = 7;
	int ch2 = 7;
	ch = ch&0b00000010;
	ch2 = ch2&0b00000011;
	char move = 'A';
	// Read values from the joystick for roomba
	// map the values project 1 mapped form 0-1023 to 0-255
	for(;;){

		// clears the MUX4:0 bits
		ADMUX = (ADMUX & 0xE0);	
		
		// Analog channel 10
		ADMUX |= ch;
		// set the MUX5 bit (A8)
		ADCSRB = (1<<MUX5); 

		// starting analog to digital conversion
		ADCSRA |= (1 <<ADSC);
		// lets the ADCSRA do its conversion before trying to read
		while( ADCSRA & (1<<ADSC));

		// get the low and high bits from the conversion (low first)
		lowX = ADCL;
		highX = ADCH;

		// ADC register can give upto 10 bits which is 0-1023
		roombaX = ((highX << 8) | lowX);

		// clears the MUX4:0 bits
		ADMUX = (ADMUX & 0xE0);	

		// Analog channel 11
		ADMUX |= ch2;
		// set the MUX5 bit (A8)
		ADCSRB = (1<<MUX5); 

		// starting analog to digital conversion
		ADCSRA |= (1 <<ADSC);
		// lets the ADCSRA do its conversion before trying to read
		while( ADCSRA & (1<<ADSC));

		// get the low and high bits from the conversion (low first)
		lowY = ADCL;
		highY = ADCH;

		// ADC register can give upto 10 bits which is 0-1023
		roombaY = ((highY << 8) | lowY);

		// move backwards
		if((roombaY > 800) && (roombaX >325) && (roombaX < 675)){
			move = 'B';
		// move forwards slowly
		}else if((roombaY > 200) && (roombaY < 400) && (roombaX >350) && (roombaX < 650)){
		move = 'S';
		//move forwards
		}else if((roombaY < 200) && (roombaX >350) && (roombaX < 650)){
			move = 'F';
		// forward left
		}else if((roombaY < 250) && (roombaX < 450)){
		move = 'I';
		// forward right
		}else if((roombaY < 250) && (roombaX > 550)){
		move = 'K';
		// turn left
		}else if((roombaX < 200) && (roombaY >350) && (roombaY < 650)){
			move = 'L';
		// turn right
		}else if ((roombaX > 800) && (roombaY >350) && (roombaY < 650)){
			move = 'R';
		// backwards left
		}else if((roombaY > 750) && (roombaX < 400)){
			move = 'G';
		// backwards right
		}else if((roombaY > 750) && (roombaX > 600)){
			move = 'J';
		}else{
			move = 'X';
		}
		if(move != prevRoomba){
			EnableLaserPin();
			Bluetooth_Send_Byte(ROOMBA);
			Bluetooth_Send_Byte(move);
			prevRoomba = move;
		}

		Task_Next();
	}
}


void ServoTask(){

	uint8_t lowX = 0;
	uint8_t highX = 0;
	
	uint8_t lowY = 0;
	uint8_t highY = 0;
	
	uint16_t servoX = 0;
	uint16_t servoY = 0;
	// Read servo values

	// initializes the pin for servo x direction
	for(;;){
		DisableLaserPin2();
		DisableLaserPin();
		// clears the MUX4:0 bits
		ADMUX = (ADMUX & 0xE0);	

		// set the MUX5 bit (A8)
		ADCSRB = (1<<MUX5); 

		// starting analog to digital conversion
		ADCSRA |= (1 <<ADSC);
		// lets the ADCSRA do its conversion before trying to read
		while( ADCSRA & (1<<ADSC));

		// get the low and high bits from the conversion (low first)
		lowX = ADCL;
		highX = ADCH;

		// ADC register can give upto 10 bits which is 0-1023
		servoX = ((highX << 8) | lowX);

		// initalizes the pin for servo y direction
	
			// clears the MUX4:0 bits
		ADMUX = (ADMUX & 0xE0);	
		// set to read from the correct pin number (A9)
		ADMUX |= (1<<MUX0);
		// set the MUX5 bit
		ADCSRB = (1<<MUX5);

		// starting analog to digital conversion
		ADCSRA |= (1 <<ADSC);
		// lets the ADCSRA do its conversion before trying to read
		while( ADCSRA & (1<<ADSC));

		lowY = ADCL;
		highY = ADCH;

		// ADC register can give upto 10 bits which is 0-1023
		servoY = ((highY << 8) | lowY);

		if(servoY > 800){
			EnableLaserPin2();
		}
		if(servoX > 800){
			EnableLaserPin();
		}

			// joystick deadzone
			if(servoX > 520 || servoX < 480){
				
				// send HW adress and instruction over bluetooth
				Bluetooth_Send_Byte(SERVOX);
				Bluetooth_Send_Byte(lowX);
				Bluetooth_Send_Byte(highX);

			}
			if( servoY > 520 || servoY < 480){
				
				//// send HW adress and instruction over bluetooth
				Bluetooth_Send_Byte(SERVOY);
				Bluetooth_Send_Byte(lowY);
				Bluetooth_Send_Byte(highY);

			}
	
		Task_Next();
	}
}

void StartADC(){

	// enables the analog to digital converter
	// prescaler set to 128 
	ADMUX |= (1<<REFS0);
	ADCSRA |= (1<<ADEN) | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2);

}


void a_main(){

	DDRB |= _BV(DDB0);
	DDRC |= _BV(LIGHTPIN);

	Bluetooth_UART_Init();
	StartADC();
	
	Task_Create_Period(LaserTask, 1, 10,1,1);
	Task_Create_Period(ServoTask, 1,10,4, 2);
	Task_Create_Period(RoombaTask, 1,10,4,6);
	Task_Create_Period(autoTask,1,10,1,10);
}

/*
 * remoteStn.c
 *
 * Created: 3/27/2017 10:36:52 PM
 *  Author: Becky Croteau & Keith Wyatt
 */
 #define F_CPU 16000000

 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include <util/delay.h>
 #include "./roomba/roomba.h"
 #include "./uart/uart.h"
 #include "os.h"

 //#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
 //#define usToTicks(_us) (( clockCyclesPerMicrosecond * _us) / 8)

 #define TESTPORT PORTL		//digital pin 42
 #define TESTPIN PL7

 #define LASERPIN PC6		//digital pin 31
 #define LASERPORT PORTC

 #define SERVOPIN PE4		//PWM pin 2

 #define SERVO_DEFAULT_POS 316
 #define SERVO_LOW_POS 535
 #define SERVO_HIGH_POS 97

uint8_t LASER = 0;
uint8_t SERVO = 1;
uint8_t ROOMBA = 2;
uint8_t LCD = 3;

PID BluetoothReceivePID;
PID LaserTaskPID;
PID ServoTaskPID;

volatile BOOL laserState;
volatile int servoState;
volatile int prevServoState;
volatile char roombaState;


#define QueueSize 16
#define QueueHead 0

int laserQueue[QueueSize];
int laserTail;
CHAN laserChan;

int roombaQueue[QueueSize];
int roombaTail;
CHAN roombaChan;

int servoQueue[QueueSize];
int servoTail;
CHAN servoChan;


/*======================= Toggle Pins ================================*/

void EnableTestPin()
{
	TESTPORT |= _BV(TESTPIN);
	_delay_ms(200);
}

void DisableTestPin()
{
	TESTPORT &= ~_BV(TESTPIN);
}

void EnableLaserPin()
{
	LASERPORT |= _BV(LASERPIN);
}
void DisableLaserPin()
{
	LASERPORT &= ~_BV(LASERPIN);
}

/*========================= Helper Functions ============================*/

void Servo_Init() {
	// Setup ports and timers
	DDRH |= _BV(PH3); //set pin 6 to output

	// Configure timer/counter1 as phase and frequency PWM mode

	TCCR4A |= (1<<COM4A1) | (1<<COM4B1) | (1<<WGM41);  //NON Inverted PWM
	TCCR4B |= (1<<WGM43) | (1<<WGM42) | (1<<CS41) | (1<<CS40); //PRESCALER=64 MODE 14 (FAST PWM)
	ICR4 = 4999;



	OCR4A = SERVO_DEFAULT_POS; // 90 Degrees
}

void InitQueue(int *queue)
{
	for(int i = 0; i < QueueSize; i++)
	{
		queue[i] = -1;
	}
}

int Queue_isEmpty(int *queue)
{
	if(queue[QueueHead] == -1)
	{
		return 1;
	}
	return 0;
}

int Queue_isFull(int *tail)
{
	if(*tail == QueueSize -1)
	{
		return 1;
	}
	return 0;
}

void Enqueue(int *queue, int value, int *tail)
{
	if(*tail == QueueSize - 1)
	{
		return;
	}
	queue[*tail] = value;
	*tail = (*tail + 1);
}

int Dequeue(int *queue, int *tail)
{
	int headData = queue[QueueHead];
	queue[QueueHead] = -1;
	int i = 1;
	while(queue[i] != -1)
	{
		if(i >= QueueSize -1) return -1;
		queue[i-1] = queue[i];
		i++;
	}
	
	*tail  = (*tail - 1);
	return headData;
}


/*========================= TASKS ======================================*/
void LaserTask()
{
	for(;;)
	{
		
		Recv(laserChan);
		if(!Queue_isEmpty(&laserQueue))
		{
				
			laserState = Dequeue(laserQueue,&laserTail);
			
			if(laserState == 1)
			{
				EnableLaserPin();				
			}
			else if(laserState == 0)
			{			
				DisableLaserPin();
			}
			else
			{
				continue;
			}
		}
		Task_Next();
	}
}

void ServoTask()
{
	for(;;)
	{
		Recv(servoChan);
		if(!Queue_isEmpty(servoQueue))
		{
			EnableTestPin();
			servoState = Dequeue(servoQueue, &servoTail);

			if(servoState > 510 && (prevServoState >= (SERVO_HIGH_POS + 50)))
			{

				if(servoState > 1000)
				{
					prevServoState -= 5;
				}
				prevServoState -= 1;
				OCR4A = prevServoState;
			}
			else if(servoState < 490 && (prevServoState <= (SERVO_LOW_POS - 50)))
			{
				if(servoState < 20)
				{
					prevServoState += 5;
				}
				prevServoState += 1;
				OCR4A = prevServoState;
			}			
		}
		Task_Next();
	}
}

void BluetoothSendTask()
{

}

void BluetoothReceiveTask()
{
	uint8_t task_flag;
	uint8_t laserData;
	uint16_t servoData;
	uint8_t servoDataL;
	uint8_t servoDataH;
	char roombaData;
	 
	for(;;)
	{

		if(UCSR1A & (1<<RXC1))
		{			
			task_flag = Bluetooth_Receive_Byte();

			//task_flag = SERVO;		//testing without base station
			
			if(task_flag == LASER)
			{
				laserData = Bluetooth_Receive_Byte();
				Enqueue(laserQueue,laserData,&laserTail);
				Send(laserChan, 0);
			}
			else if(task_flag == SERVO)
			{
				servoDataL = Bluetooth_Receive_Byte();
				servoDataH = Bluetooth_Receive_Byte();
				servoData = ( ((servoDataH) << 8) | (servoDataL) );
				Enqueue(servoQueue,servoData,&servoTail);
				Send(servoChan,0);

			}
			else if(task_flag == ROOMBA)
			{
				roombaData = Bluetooth_Receive_Byte();
				Enqueue(roombaQueue,roombaData,&roombaTail);
				Send(roombaChan,0);
			}
			else if(task_flag == LCD)
			{

			}
			else
			{
				continue;
			}
		}
			
		Task_Next();
	}

}
/* ============================ Roomba =============================*/

void RoombaManualDrive()
{
	if(roombaState == 'F')
	{
		Roomba_Drive(ROOMBA_SPEED,DRIVE_STRAIGHT);
	}
	else if(roombaState == 'B')
	{
		Roomba_Drive(-ROOMBA_SPEED,DRIVE_STRAIGHT);
	}
	else if(roombaState == 'L')
	{
		Roomba_Drive(ROOMBA_TURN,IN_PLACE_CCW);
	}
	else if(roombaState == 'R')
	{
		Roomba_Drive(ROOMBA_TURN, IN_PLACE_CW);
	}
}

void RoombaTask()
{
	for(;;)
	{
		Recv(roombaChan);
		if(!Queue_isEmpty(roombaQueue))
		{
			roombaState = Dequeue(roombaQueue,&roombaTail);
			RoombaManualDrive();
		}

		Task_Next();
	}
}

void GetRoombaSensorDataTask()
{
}


void a_main()
{
	//Initialize ports
	DDRL |= _BV(LASERPIN);
	PORTL &= ~_BV(LASERPIN);

	//Test pin 42
	PORTL &= ~_BV(PL7);

	Bluetooth_UART_Init();
	Roomba_UART_Init();
	Servo_Init();

	servoState = SERVO_DEFAULT_POS;
	prevServoState = SERVO_DEFAULT_POS;

	laserTail = 0;
	roombaTail = 0;
	servoTail = 0;

	laserChan = Chan_Init();
	roombaChan = Chan_Init();
	servoChan = Chan_Init();

	InitQueue(laserQueue);
	InitQueue(roombaQueue);
	InitQueue(servoQueue);

	

	//Create Tasks
	BluetoothReceivePID = Task_Create_System(BluetoothReceiveTask,1);
	LaserTaskPID = Task_Create_System(LaserTask, 1);
	ServoTaskPID = Task_Create_System(ServoTask,1);
}

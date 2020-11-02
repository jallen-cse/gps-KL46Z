#include "stdio.h"
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"

char buffer1[256], buffer2[256], *fields[16];
char* incoming = buffer1;
char* processing = buffer2;		//might need volatile?

volatile int count = 0;					//might not need volatile?

volatile bool message_ready = 0;

int current_lat, current_lon, fix_accuracy;
float destination_lat, destination_lon;

bool first_msg = 1;

void hardware_init()
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();
}

void setup_UART1_interrupt()
{
	UART1->C2 |= (1 << 5);			//receiver interrupt enable
	NVIC_EnableIRQ(13);				//core API enable UART1 interrupts ... page 56 ref manual
}

void UART1_IRQHandler(void)
{
	if(UART1->S1 && (1 << 5))			//if UART1 read data ready flag is set = if full character is received
	{
		incoming[count] = UART1->D;
		count++;
	}

	if(incoming[count-2] == 13 && incoming[count-1] == 10)	//<cr> <lf>	must be a way to watch status flag for line ends
	{
		char* temp = incoming;
		incoming = processing;
		processing = temp;

		count = 0;
		message_ready = 1;
	}
}

int get_heading(float end_lat, float end_lon)
{
	return (int)(450 - (180 * atan2f((end_lat * 10000) - current_lat, (end_lon * 10000) - current_lon) / 3.1415926)) % 360;
}

int get_distance_m(float end_lat, float end_lon)
{
	float lon_scaler = cosf(3.1415926 * abs(end_lat) / 180);
	return 11.1 * sqrt(pow((end_lat * 10000) - current_lat, 2) + pow(lon_scaler * ((end_lon * 10000) - current_lon), 2));					//x^2 + y^2
}

void parse_message(char* message)
{
	int field_index = 0;
	char* token = strtok(message, ",");

	while(token != NULL && field_index < 16)
	{
		fields[field_index] = token;			//point fields index to same location as token
		field_index++;

		token = strtok(NULL, ",");
	}

	if(!strcmp(fields[0], "$GPGGA"))			//if message is of GPGGA format
	{
		current_lat = (10000*((int)(10000*atof(fields[2])) / 1000000)) + (((int)(10000*atof(fields[2])) % 1000000) / 60);
		current_lon = (10000*((int)(10000*atof(fields[4])) / 1000000)) + (((int)(10000*atof(fields[4])) % 1000000) / 60);

		if(!strcmp(fields[3], "S"))				//if in southern hemisphere
		{
			current_lat = current_lat * -1;
		}

		if(!strcmp(fields[5], "W"))				//if in western hemisphere
		{
			current_lon = current_lon * -1;
		}

		fix_accuracy = atoi(fields[6]);
	}
}

int main()
{
	hardware_init();

    SIM->SOPT5 &= ~(1 << 6); 		//uart1 receive data source select to RX ... probably unnecessary

    SIM->SCGC5 |= (1 << 13);		//port E clock gating
    PORTE->PCR[1] |= 0x3 << 8;		//port E1 to alternative 3 (UART0 RX, pg 172)

    SIM->SCGC4 |= (1 << 11);		//UART1 clocking

    UART1->BDH = 0;					//upper bits of baud divisor -> unused here
    UART1->BDL = 156;				//baud divisor -> maybe need to change something if we missing characters
    UART1->C2 |= (1 << 2);			//enable UART1 receive

    setup_UART1_interrupt();

    destination_lat = 30.81;
    destination_lon = -120;

    while(1)
    {
    	if(message_ready && !(first_msg))
    	{
    		parse_message(processing);
    		message_ready = 0;

    		if(!strcmp(fields[0], "$GPGGA"))
    		{
    			printf("Current Latitude: %d\nCurrent Longitude: %d\nFix Accuracy: %d\n--To %f, %f--\nCompass Heading: %d\tDistance (m): %d\n\n", current_lat, current_lon, fix_accuracy, destination_lat, destination_lon, get_heading(destination_lat, destination_lon), get_distance_m(destination_lat, destination_lon));
    		}
       	}
    	else if(message_ready && first_msg)
    	{
    		first_msg = 0;
    		message_ready = 0;
    	}
    }

    return 0;
}
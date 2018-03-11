/********************************************************************************
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010

 Application example: Robot control over serial port via XBee wireless communication module 
 					  (located on the ATMEGA260 microcontroller adaptor board)

 Concepts covered:  serial communication
 
 Serial Port used: UART0

 There are two components to the motion control:
 1. Direction control using pins PORTA0 to PORTA3
 2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B.

 In this experiment for the simplicity PL3 and PL4 are kept at logic 1.
 
 Pins for PWM are kept at logic 1.
  
 Connection Details:  	
 						
  Motion control:		L-1---->PA0;		L-2---->PA1;
   						R-1---->PA2;		R-2---->PA3;
   						PL3 (OC5A) ----> Logic 1; 	PL4 (OC5B) ----> Logic 1; 


  Serial Communication:	PORTD 2 --> RXD1 UART1 receive for RS232 serial communication
						PORTD 3 --> TXD1 UART1 transmit for RS232 serial communication

						PORTH 0 --> RXD2 UART 2 receive for USB - RS232 communication
						PORTH 1 --> TXD2 UART 2 transmit for USB - RS232 communication

						PORTE 0 --> RXD0 UART0 receive for ZigBee wireless communication
						PORTE 1 --> TXD0 UART0 transmit for ZigBee wireless communication

						PORTJ 0 --> RXD3 UART3 receive available on microcontroller expansion socket
						PORTJ 1 --> TXD3 UART3 transmit available on microcontroller expansion socket

Serial communication baud rate: 9600bps
To control robot use number pad of the keyboard which is located on the right hand side of the keyboard.
Make sure that NUM lock is on.

Commands:
			Keyboard Key   ASCII value	Action
				8				0x38	Forward
				2				0x32	Backward
				4				0x34	Left
				6				0x36	Right
				5				0x35	Stop
				7				0x37	Buzzer on
				9				0x39	Buzzer off

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0 (For more information read section: Selecting proper optimization 
 						options below figure 2.22 in the Software Manual)

 2. Difference between the codes for RS232 serial, USB and wireless communication is only in the serial port number.
 	Rest of the things are the same. 

 3. For USB communication check the Jumper 1 position on the ATMEGA2560 microcontroller adaptor board

 4. Auxiliary power can supply current up to 1 Ampere while Battery can supply current up to 
 	2 Ampere. When both motors of the robot changes direction suddenly without stopping, 
	it produces large current surge. When robot is powered by Auxiliary power which can supply
	only 1 Ampere of current, sudden direction change in both the motors will cause current 
	surge which can reset the microcontroller because of sudden fall in voltage. 
	It is a good practice to stop the motors for at least 0.5seconds before changing 
	the direction. This will also increase the useable time of the fully charged battery.
	the life of the motor.

*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/


#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include <math.h> //included to support power function
#include "lcd.c"
//#include "lcd.h"
#define ROBOT_ID 1 	/* ID of the Robot, Should be different (unique) for every Robot in the system */

unsigned char data; 	// to store the received data from UDR0
unsigned char cmd[350]; // to store the commands received
unsigned int bytecount = 0; 		// Counts the number of bytes received from the packet
unsigned int numberofbytes = 0; 	// Number of command bytes in the packet
unsigned int pkt_state = 0; 		// Enumerates the states of the packet-receiving FSM code
int busy = 0; 				//Status of the robot(whether executing the commands or idle)

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0; //Left white line sensor value
unsigned char Center_white_line = 0; //Center white line sensor value
unsigned char Right_white_line = 0; //Right white line sensor value
unsigned char thr = 0x14; //Threshold value for white line sensing
unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning


void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();
void update_sensor_value();

/**
* Functions specific to the Multi-Bot Controller system
*/

void black_line_follower();
void left_turn();
void right_turn();
void u_turn();


//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}




//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}



void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}


//Function to initialize ports
void port_init()
{
	motion_pin_config();
	buzzer_pin_config();
	lcd_port_config();
	adc_pin_config();
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config	

}

void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}



void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt 
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt 
}

//ISR for right position encoder
ISR(INT5_vect)  
{
 ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

void forward (void) //both wheels Forward
{
  motion_set (0x06);
}

void stop (void) //both wheels stationary
{
  motion_set (0x00);
}

void back (void) //both wheels backward
{
  motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}


void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}



//Function used for turning robot by specified degrees

void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 

 while (1)
 {
  lcd_print(2,8,ShaftCountRight,2);
  lcd_print(2,11,ShaftCountLeft,2);
  lcd_print(2,14,ReqdShaftCountInt,2);
  if((ShaftCountRight >= ReqdShaftCountInt) || (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 lcd_print(2,8,ShaftCountRight,2);
 lcd_print(2,11,ShaftCountLeft,2);
 lcd_print(2,14,ReqdShaftCountInt,2);
 stop(); //Stop robot
}

void left_degrees(unsigned int Degrees) 
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}

void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}

void soft_left_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left(); //Turn soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}




//Called function of forward_mm() and back_mm() for moving the robot by a specified distance (in mm) in set direction (as in calling function)

void linear_distance_mm(unsigned int DistanceInMM) 
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountRight = 0;

 lcd_print(2,8,12,2);

 while(1)
 {
  lcd_print(2,8,ShaftCountRight,3);
  lcd_print(2,12,ReqdShaftCountInt,3);
  if(ShaftCountRight > ReqdShaftCountInt)
  {
    break;
  }
 } 
  lcd_print(2,8,ShaftCountRight,3);
  lcd_print(2,12,ReqdShaftCountInt,3);
 stop(); //Stop robot
}

//Moves Robot by the specified distance in forward direction(in mm)

void forward_mm(unsigned int DistanceInMM) 
{
 forward();
 linear_distance_mm(DistanceInMM);
}

//Moves Robot by the specified distance in backward direction(in mm)

void back_mm(unsigned int DistanceInMM) 
{
 lcd_print(2,8,8,2);
 back();
 lcd_print(2,8,9,2);
 linear_distance_mm(DistanceInMM);
 lcd_print(2,8,20,2);
}

// Updates values of white line sensors and displays it on the LCD screen

void update_sensor_value()
{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
}

// Takes left turn by moving 40 mm forward first (because robot's center of rotation is different from it's white line sensor position)

void left_turn()
{
	velocity(190,190);
	forward_mm(40);
	left_degrees(70);
	update_sensor_value();			

	while(Left_white_line<thr && Center_white_line<thr && Right_white_line<thr)
	{
		left_degrees(2);
		lcd_print(2,1,777,3);
		update_sensor_value();	
	}
	lcd_print(2,1,999,3);
	stop();
}

// Takes right turn by moving 40 mm forward first (because robot's center of rotation is different from it's white line sensor position)

void right_turn()
{
	velocity(190,190);
	forward_mm(40);
	right_degrees(70);
	update_sensor_value();			

	while(Left_white_line<thr && Center_white_line<thr && Right_white_line<thr)
	{
		right_degrees(2);
		update_sensor_value();	
	}
	stop();
}


// Takes U turn by first taking some specified degrees left turn, and then moves 2 degrees at a time until it finds the black line.

void u_turn()
{
	velocity(190,190);
	left_degrees(160);
	update_sensor_value();			

	while(Left_white_line<thr && Center_white_line<thr && Right_white_line<thr)
	{
		left_degrees(2);
		update_sensor_value();	
	}
	stop();
}


//Code for black line following

void black_line_follower()
{
	forward();
	velocity(210,210);
	
	while(1)
	{

		update_sensor_value();

		flag=0;

		if(Center_white_line>thr && Left_white_line<thr && Right_white_line<thr) // On black line
		{
			flag=1;
			forward();
			velocity(210,210);
			lcd_print(2,1,1,1);
			
		}

		if((Center_white_line<thr && Left_white_line>thr && Right_white_line<thr) && (flag==0)) // Take left to be on center of black line
		{
			flag=1;
			forward();
			velocity(140,210);
			lcd_print(2,1,2,1);
			
		}

		if((Center_white_line<thr && Left_white_line<thr && Right_white_line>thr) && (flag==0)) // Take right to be on center of black line
		{
			flag=1;
			forward();
			velocity(210,140);
			lcd_print(2,1,3,1);
			
		}

		if(Center_white_line<thr && Left_white_line<thr && Right_white_line<thr) // Code to do away with the problem of black line's width less than the distance between white line sensors (All white sensing on black line)
		{

			forward();
			velocity(140,180);	

		}
		if(Center_white_line>thr && (Left_white_line>thr || Right_white_line>thr)) // Sensing point detected (atleast two sensors sensing black)
		{


			forward();

			velocity(210,210);
			
			lcd_print(2,1,5,1);
		
			buzzer_on();

			_delay_ms(50);

			buzzer_off();

			forward_mm(20);

			return;
			
		}

	}
}

//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}

SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt (getting a byte from ZigBee)
{
	data = UDR0; 			// making a copy of data from UDR0 in 'data' variable 

	bytecount++;

	if(pkt_state == 0)		// Expecting 0xff (Start delimiter byte)
	{
		if(data == 0xff)
		{
			pkt_state = 1;
		}
		else
		{
			bytecount = 0;
		}
	}
	else if(pkt_state == 1)		// Expecting Robot ID
	{
		if(data == ROBOT_ID)
		{
			pkt_state = 2;
		}
		else
		{
			pkt_state = 0;
			bytecount = 0;
		}
	}
	else if(pkt_state == 2)		// Expecting number of command bytes in the packet
	{
		numberofbytes = (int)data;
		pkt_state = 3;
	}
	else if(pkt_state == 3)		// Receiving actual packet (commands)
	{
		cmd[bytecount-4] = data;

		if(bytecount == numberofbytes + 3)
		{
			busy = 1;
		}
	}
}

//Function To Initialize all The Devices
void init_devices()
{
 cli(); //Clears the global interrupts
 port_init();  //Initializes all the ports
 
 adc_init();
 timer5_init();
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 uart0_init(); //Initailize UART1 for serial communication
 sei();   //Enables the global interrupts
}

//Main Function
int main(void)
{
	init_devices();
	
	lcd_set_4bit();
	lcd_init();

	int i=0;	

	while(1)
	{
		while(!busy)	// Checking whether robot is busy executing received commands or not
		{
			lcd_print(2,1,bytecount,2);
			lcd_print(2,4,numberofbytes,2);
		}

		lcd_print(2,1,44,2);

		velocity(200,200);

		while(i<numberofbytes)	// Executing number of received commands
		{
			lcd_print(1,14,i,2);
			
			if(cmd[i] == 0x46)			//ASCII value of F (follow black line upto the next sensing point (atleast 2 sensors sensing black line))
			{
				black_line_follower();
			}
			else if (cmd[i] == 0x4C)		//ASCII value of L (take left turn and try to find black line on the track)
			{				
				left_turn();
			}
			else if (cmd[i] == 0x52)		//ASCII value of R (take right turn and try to find black line on the track)
			{
				right_turn();
			}
			else if (cmd[i] == 0x55)		//ASCII value of U (take U turn and try to find black line on the track)
			{
				u_turn();
			}
			
			i++;
		}
		stop();
		busy = 0;
		pkt_state = 0;
		bytecount = 0;
		numberofbytes = 0;
		i = 0;
		lcd_print(1,14,i,2);
		UDR0 = (char)(ROBOT_ID + 48);			//Sending the Robot ID as an acknowledgemet of completion of command execution to the central controller (destination reached)
	}	
}
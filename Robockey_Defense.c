/*
 * Puck_Find.c
 *
 * Created: 11/8/2015 8:33:45 PM
 *  Author: stanley
 */ 

#include <avr/io.h>
#include "m_general.h"
#include "m_usb.h"
#include "m_bus.h"
#include "m_wii.h"
#include "m_rf.h"

void read_in_terminal();
void ADC_init();
void Read_ADC();
void Clock_source_1();
void Determine_max_point();
void puck_whether_behind();
//Wireless Communication
//Our Wireless address 0x50 0x51 0x52
#define CHANNEL 1
//Robot 1
#define RXADDRESS 0x50
#define PACKET_LENGTH 10
char buffer[PACKET_LENGTH] ={0,0,0,0,0,0,0,0,0,0};

volatile int bufferStoreVariable = 0;
volatile int wifi_Ax = 0;
void wirelessCommunication();
void initializeWireless();
void checkWirelessIsOn();
void readWifi();
// go to goal
void go_to_goal();
// find puck
void find_puck();
void find_puck_behind();
void find_puck_behind_right();
void test2_behind_right();
void test2_behind_left();
//Localization Variables
unsigned int wii_data[12] = {0, 0, 0,0 ,0 ,0 ,0, 0, 0, 0, 0, 0};
long distance[6] = {0,0,0,0,0,0};
void whereamI(unsigned int *wii_data);
int checkWillData();
void initializeWii();
double pi = 3.14159265358979311;
// defense
void  CheckWhethergoBackToDefend();
volatile int defend = 0;
volatile int defend_num = 0;
volatile int defend_case = 0;
void Defend();
void goToOurGoal();
void checkRobotPosition();
volatile int positive = 2;
volatile int stop = 0;
//PID variables
int P = 20;
int I = 5;
int D = 0;
int error = 0;
int error_old_all = 0;
int photo_front_old = 0;
int Upper_photo_filter;
int time_run_onetime;
// for photo transistor
int photoUpperRight_1= 0; 
int photoUpperRight_2 = 0;
int photoUpperLeft_1 = 0;
int photoUpperLeft_2 = 0;
int photoLowerRight_1 = 0;
int photoLowerRight_2 = 0;
int photoLowerLeft_1 = 0;
int photoLowerLeft_2 = 0;
int photoCenter = 0;
int photo_straight_down = 0;
int photo[9] = {0,0,0,0,0,0,0,0,0};
volatile int max = 0;
volatile int max_number = 0;
int i = 0;
int PID_Output;

volatile int determine = 0;
volatile int goal_behind = 0;
volatile int test = 2;
volatile int get_puck = 0;
volatile double robot_angle = 0;
volatile double robotPosition[2] = {0,0};
volatile int tryout = 0;
volatile int maxee = 0;
volatile int rotate_half_circle = 0;
volatile int time_to_half_cirlce = 0;
volatile int turnright = 0;
volatile int count_clock = 0;

int main(void)
{
	//be able to read data in terminal
	//read_in_terminal();
	//set clock source to 16Mhz
	m_clockdivide(0);
	//Localization
	initializeWii();
	//Wireless Communication
	initializeWireless();
	//CheckWireless
	checkWirelessIsOn();
	//Clock Source 1
	Clock_source_1();
	//initial ADC	
	ADC_init();
	 
    while(1)
    {
		//request data from wii
		m_wii_read((unsigned int*) wii_data);
		//if wii data = 1023 green on
		if(checkWillData() == -1){
			m_green(TOGGLE);
		}
		else if(checkWillData() == 1){
			whereamI(wii_data);
			m_green(OFF);
		}
		switch(buffer[0]){
			case 0xA0:
				if(robotPosition[0] < 0){
					if (count_clock > 150){
						set(DDRB,0);
						set(PORTB,0);
					}else{
						clear(PORTB,0);
					}
					test = 2;
				}else if(robotPosition[0] > 0){
					if (count_clock > 150){
						set(DDRB,1);
						set(PORTB,1);
					}else{
						clear(PORTB,1);
					} 
					test = 1;
				}
				break;
			case 0xA1:
				if(test ==1){
					set(PORTB,1);
				}else if(test ==2){
					set(PORTB,0);
				}
				if(max < 300){
					checkRobotPosition();
				}else if(max > 300){
					find_puck();
				}
				break;
			case 0xA4:
				OCR1B = 0;
				OCR1C = 0;
				break;
		}
		Read_ADC();
		Determine_max_point();
		set(TIMSK1,OCIE1B);
		/*m_usb_tx_string("buffer = ");
		m_usb_tx_int((int)buffer[0]);
		m_usb_tx_string(" \n  ");
		m_wait(30000);*/
		/*m_usb_tx_string("max_num = ");
		m_usb_tx_int(max_number);
		m_usb_tx_string("   ");
		m_usb_tx_string("lowerleft1= ");
		m_usb_tx_int(photoLowerLeft_1);
		m_usb_tx_string("   ");
		m_usb_tx_string("lowerleft2 = ");
		m_usb_tx_int(photoLowerLeft_2);
		m_usb_tx_string("  ");
		m_usb_tx_string("lower right 1 = ");
		m_usb_tx_int(photoLowerRight_1);
		m_usb_tx_string("   ");
		m_usb_tx_string("lower right2 = ");
		m_usb_tx_int(photoLowerRight_2);
		m_usb_tx_string("   ");
		m_usb_tx_string("center = ");
		m_usb_tx_int(photoCenter);
		m_usb_tx_string("   ");
		m_usb_tx_string("straight down = ");
		m_usb_tx_int(photo_straight_down);
		m_usb_tx_string("   ");
		m_usb_tx_string("upperright2 = ");
		m_usb_tx_int(photoUpperRight_2);
		m_usb_tx_string("   ");
		m_usb_tx_string("upperright1 = ");
		m_usb_tx_int(photoUpperRight_1);
		m_usb_tx_string("  ");
		m_usb_tx_string("upperleft1 = ");
		m_usb_tx_int(photoUpperLeft_1);
		m_usb_tx_string("   ");
		m_usb_tx_string("upperleft2 = ");
		m_usb_tx_int(photoUpperLeft_2);
		m_usb_tx_string("   ");
		m_usb_tx_string("\n");
		m_wait(30000);*/
    }
}
ISR(TIMER1_COMPB_vect){
	if(count_clock < 300){
		count_clock = count_clock +1;
	}else if(count_clock ==300){
		count_clock = 0;
	}
	
}

ISR(INT2_vect){
	m_rf_read(buffer,PACKET_LENGTH);// pull the packet
}

void find_puck(){
	if(get_puck ==0){
		//m_green(ON);
		//clear(DDRB,0);
		// puck is at right side of robot
		if(max_number < 5){
			if(max_number == 0){
				OCR1B = 833;
				OCR1C = 833;
			}else{
				OCR1B = 833;
				OCR1C = 60*max_number ;
			}
		}
		// puck is at left side of robot
		else if(max_number > 4){
			OCR1B = 60*(max_number - 4);
			OCR1C = 833;
		}
		if(photo_straight_down < 200 ){
			get_puck = 1;
			//m_red(ON);
		}
	}
	else if(get_puck == 1){
		//set(PORTB,0);
		go_to_goal();
		if(photo_straight_down > 900){
			get_puck = 0;
			//m_red(OFF);
		}
		
	}
}


void puck_whether_behind(){
	if(test ==1 && (robot_angle <0) && max_number <3 && max_number > 0){//(test ==2 && (robot_angle > 0 ) && max_number <3 && max_number > 0) ||
		goal_behind = 1;
		
	}
	else if(test ==1 && (robot_angle < 0 ) && max_number <7 && max_number > 4){//(test ==2 && (robot_angle > 0 ) && max_number <7 && max_number > 4) ||
		goal_behind = 2;
	}
	else if(test ==2 && (robot_angle > 0 ) && max_number <3 && max_number > 0){//(test ==2 && (robot_angle > 0 ) && max_number <7 && max_number > 4) ||
		goal_behind = 3;
	}
	else if (test ==2 && (robot_angle > 0 ) && max_number <7 && max_number > 4){
		goal_behind = 4;
	}
	
}
void test2_behind_left(){
	if(photoCenter < 800 && goal_behind ==4 && turnright ==0 && rotate_half_circle ==0 && time_to_half_cirlce ==0){
		determine = 0;
		set(DDRB,1);
		set(PORTB,1);
	}
	if (photoCenter > 800 && goal_behind ==4 && turnright ==0 && rotate_half_circle ==0 && time_to_half_cirlce ==0){
		determine = 1;
		turnright =1;
		
	}
	if(turnright ==1){
		determine = 1;
		clear(PORTB,1);
		set(DDRB,0);
		set(PORTB,0);
	}
	if(robot_angle < -100 && robot_angle > -135 && goal_behind == 4 && rotate_half_circle == 0 && turnright == 1){
		determine = 2;
		turnright = 0;
		rotate_half_circle = 1;
	}
	if(rotate_half_circle ==1){
		determine = 2;
		m_red(ON);
		
	}
	if(((max_number == 2) && rotate_half_circle == 1 && time_to_half_cirlce ==0) || robotPosition[0] < -80 || photo_straight_down < 200){
		time_to_half_cirlce = 1;
		determine = 5;
		rotate_half_circle = 0;
	}
	if(time_to_half_cirlce ==1){
		determine = 5;
		m_red(OFF);
	}
	if (((robot_angle > 125 && robot_angle < 135) && time_to_half_cirlce == 1)||(time_to_half_cirlce ==1 && (robot_angle < -165 || robot_angle > 165)  && max_number == 2) ){
		determine = 0;
		clear(PORTB,0);
		rotate_half_circle = 0;
		time_to_half_cirlce = 0;
		goal_behind = 0;
		find_puck();
	}
	switch(determine){
		case 0:
			//set(DDRB,1);
			//set(PORTB,1);
			find_puck();
			break;
		case 1:
			OCR1B = 100;
			OCR1C = 833;
			break;
		case 2:
			OCR1C = 833;
			OCR1B = 833;
			break;
		case 5:
			//clear(PORTB,0);
			//set(DDRB,1);
			//set(PORTB,1);
			OCR1C = 180;
			OCR1B = 500;
			break;
	}
	determine = 0;
	
}

void test2_behind_right(){
	if(photoCenter < 800 && goal_behind ==3 && turnright ==0 && rotate_half_circle ==0 && time_to_half_cirlce ==0){
		determine = 0;
		//set(DDRB,1);
		//set(PORTB,1);
	}
	if (photoCenter > 800 && goal_behind ==3 && turnright ==0 && rotate_half_circle ==0 && time_to_half_cirlce ==0){
		determine = 1;
		turnright =1;
		
	}
	if(turnright ==1){
		determine = 1;
		//clear(PORTB,1);
		//set(DDRB,0);
		//set(PORTB,0);
	}
	if(robot_angle < -50 && robot_angle > -60 && goal_behind == 3 && rotate_half_circle == 0 && turnright == 1){
		determine = 2;
		turnright = 0;
		rotate_half_circle = 1;
	}
	if(rotate_half_circle ==1){
		determine = 2;
		//m_red(ON);
		
	}
	if(((max_number == 6) && rotate_half_circle == 1 && time_to_half_cirlce ==0) || robotPosition[0] < -80 || photo_straight_down < 200){
		time_to_half_cirlce = 1;
		determine = 5;
		rotate_half_circle = 0;
	}
	if(time_to_half_cirlce ==1){
		determine = 5;
		//m_red(OFF);
	}
	if (((robot_angle > 45 && robot_angle < 55) && time_to_half_cirlce == 1)||(time_to_half_cirlce ==1 && (robot_angle < 10 && robot_angle > -10)  && max_number == 6) ){
		determine = 0;
		//clear(PORTB,0);
		rotate_half_circle = 0;
		time_to_half_cirlce = 0;
		goal_behind = 0;
		find_puck();
	}
	switch(determine){
		case 0:
			//set(DDRB,1);
			//set(PORTB,1);
			find_puck();
			break;
		case 1:
			OCR1B = 833;
			OCR1C = 100;
			break;
		case 2:
			OCR1C = 833;
			OCR1B = 833;
			break;
		case 5:
			//clear(PORTB,0);
			//set(DDRB,1);
			//set(PORTB,1);
			OCR1C = 500;
			OCR1B = 160;
			break;
	}
	determine = 0;
}

void find_puck_behind_right(){
	if(photoCenter < 800 && goal_behind ==2 && turnright ==0 && rotate_half_circle ==0 && time_to_half_cirlce ==0){
		determine = 0;
		//set(DDRB,1);
		//set(PORTB,1);
	}
	if (photoCenter > 800 && goal_behind ==2 && turnright ==0 && rotate_half_circle ==0 && time_to_half_cirlce ==0){
		determine = 1;
		turnright =1;
		
	}
	if(turnright ==1){
		determine = 1;
		//clear(PORTB,1);
		//set(DDRB,0);
		//set(PORTB,0);
		
	}
	if(robot_angle < 65 && robot_angle > 55 && goal_behind == 2 && rotate_half_circle == 0 && turnright == 1){
		determine = 2;
		turnright = 0;
		rotate_half_circle = 1;
	}
	if(rotate_half_circle ==1){
		determine = 2;
		//m_red(ON);
		
	}
	if(((max_number == 2) && rotate_half_circle == 1 && time_to_half_cirlce ==0) || robotPosition[0] > 80 || photo_straight_down < 200){
		time_to_half_cirlce = 1;
		determine = 5;
		rotate_half_circle = 0;
	}
	if(time_to_half_cirlce ==1){
		determine = 5;
		//m_red(OFF);
		//clear(PORTB,0);
		//set(DDRB,1);
		//set(PORTB,1);
		
	}
	if (((robot_angle < -45 && robot_angle > -55) && time_to_half_cirlce == 1)||(time_to_half_cirlce ==1 && (robot_angle < 30 && robot_angle > -30)  && max_number == 2) ){
		determine = 0;
		//clear(PORTB,0);
		rotate_half_circle = 0;
		time_to_half_cirlce = 0;
		goal_behind = 0;
		find_puck();
	}
	switch(determine){
		case 0:
			//set(DDRB,1);
			//set(PORTB,1);
			find_puck();
			break;
		case 1:
			OCR1B = 100;
			OCR1C = 833;
			break;
		case 2:
			OCR1C = 833;
			OCR1B = 833;
			break;
		case 5:
			//clear(PORTB,0);
			//set(DDRB,1);
			//set(PORTB,1);
			OCR1C = 200;
			OCR1B = 500;
			break;
	}
	determine = 0;
}

void find_puck_behind(){
	//m_green(ON);
	//determine = 0;
	if(photoCenter < 800 && goal_behind ==1 && turnright ==0 && rotate_half_circle ==0 && time_to_half_cirlce ==0){
		determine = 0;
		//set(DDRB,1);
		//set(PORTB,1);
	}
	if (photoCenter > 800 && goal_behind ==1 && turnright ==0 && rotate_half_circle ==0 && time_to_half_cirlce ==0){
		determine = 1;
		turnright =1;
		
	}
	if(turnright ==1){
		determine = 1;
		//clear(PORTB,1);
		//set(DDRB,0);
		//set(PORTB,0);
		
	}
	if(robot_angle < 145 && robot_angle > 135 && goal_behind == 1 && rotate_half_circle == 0 && turnright == 1){
		determine = 2;
		turnright = 0;
		rotate_half_circle = 1;
	}
	if(rotate_half_circle ==1){
		determine = 2;
		//m_red(ON);
		
	}
	if(((max_number == 6) && rotate_half_circle == 1 && time_to_half_cirlce ==0) || robotPosition[0] > 80 || photo_straight_down < 200){
		time_to_half_cirlce = 1;
		determine = 5;
		rotate_half_circle = 0;
	}
	if(time_to_half_cirlce ==1){
		determine = 5;
		//m_red(OFF);
		//clear(PORTB,0);
		//set(DDRB,1);
		//set(PORTB,1);
		
	}
	if (((robot_angle < -100 && robot_angle > -105) && time_to_half_cirlce == 1)||(time_to_half_cirlce ==1 && (robot_angle < -155 || robot_angle > 160)  && max_number == 6) ){
		determine = 0;
		//clear(PORTB,0);
		rotate_half_circle = 0;
		time_to_half_cirlce = 0;
		goal_behind = 0;
		find_puck();
	}
	switch(determine){
		case 0:
			//set(DDRB,1);
			//set(PORTB,1);
			find_puck();
			break;
		case 1:
			OCR1B = 833;
			OCR1C = 100;
			break;
		case 2:
			OCR1C = 833;
			OCR1B = 833;
			break;
		case 5:
			//clear(PORTB,0);
			//set(DDRB,1);
			//set(PORTB,1);
			OCR1C = 500;
			OCR1B = 150;
			break;
	}
	determine = 0;
		
}

void CheckWhethergoBackToDefend(){
	if(max < 120){
		defend = 1;
	}
	else if(max > 120){
		defend = 0;
	}
}

void goToOurGoal(){
	//Defend Goal B and Attack Goal A
	if (test == 1){
		if(75 < robot_angle && robot_angle < 105){
			//goal is at x = 115, y = 0
			if(robotPosition[1] > 0){
				OCR1B = 650;
				OCR1C = 833;
			}else if(robotPosition[1] < 0){
				OCR1B = 833;
				OCR1C = 650;
			}else if(robotPosition[1] == 0){
				OCR1B = 833;
				OCR1C = 833;
			}
		}else if(robot_angle < -90 || (robot_angle < 180 && robot_angle > 105)){
			OCR1B = 833; // B6 left wheel
			OCR1C = 100; // B7 right wheel
		}else if(robot_angle < 85 && robot_angle > -90 ){
			OCR1B = 100; // left wheel
			OCR1C = 833; // right wheel
		}
	}
	//Attack Goal B Defend Goal A
	if (test == 2){
		if((-105 < robot_angle) && (robot_angle < -75)){
			//goal is at x = -115, y = 0
			if(robotPosition[1] > 0){
				OCR1B = 833;
				OCR1C = 600;
			}else if(robotPosition[1] < 0){
				OCR1B = 600;
				OCR1C = 833;
			}else if(robotPosition[1] == 0){
				OCR1B = 833;
				OCR1C = 833;
			}
		}else if((robot_angle < -105) || (robot_angle > 90)){
			OCR1B = 100; // B6 left wheel
			OCR1C = 833; // B7 right wheel
		}else if((robot_angle > -75) && (robot_angle < 90) ){
			OCR1B = 833; // left wheel
			OCR1C = 100; // right wheel
		}
	}
}

void checkRobotPosition(){
	//Defend Goal B
	if(test == 1 && robotPosition[0] < 75 ){
		defend_num = 1;
	}
	else if(test==1 && robotPosition[0] > 75 ){
		defend_num = 2;
	}
	else if (test ==2 && robotPosition[0] > -75 ){
		defend_num = 1;
	}
	else if (test ==2 && robotPosition[0] < -75){
		defend_num =3 ;
	}
	else {
		defend_num =0;
	}
	switch(defend_num){
		case 0:
			OCR1C = 0;
			OCR1B = 0;
			break;
		case 1:
			goToOurGoal();
			break;
		case 2:
			OCR1B = 833;
			OCR1C = 50;
			if(robot_angle < -60){
				OCR1B = 0;
				OCR1C = 0;
				defend_num = 0;
			}
			break;
		case 3:
			OCR1C = 833;
			OCR1B = 50;
			if (robot_angle > 80){
				OCR1B = 0;
				OCR1C = 0;
				defend_num = 0;
			}
		
	}
}
void Read_ADC(void){
	//photoUpperRight
	//UpperLeft2 input
	// Disable ADC system to change MUX bit
	clear(ADCSRA,ADEN);
	//Set MUX bit to F0
	clear(ADCSRB,MUX5);
	clear(ADMUX,MUX2);
	clear(ADMUX,MUX1);
	clear(ADMUX,MUX0);
	//Enable the system
	set(ADCSRA,ADEN);
	//Start the conversion
	set(ADCSRA,ADSC);
	while(!check(ADCSRA,ADIF)){} //ADC conversion ongoing
	set(ADCSRA,ADIF);// Clear the flag
	
	photoUpperLeft_1 = ADC;
	
	//Upperleft1 input
	clear(ADCSRA,ADEN); // Disable ADC system to change MUX bit
	//Set MUX bit to F1
	clear(ADCSRB,MUX5);
	clear(ADMUX,MUX2);
	clear(ADMUX,MUX1);
	set(ADMUX,MUX0);
	
	set(ADCSRA,ADEN); //Enable the system
	set(ADCSRA,ADSC); //Start the conversion
	
	while(!check(ADCSRA,ADIF)){} //ADC conversion ongoing
	set(ADCSRA,ADIF);// Clear the flag
	
	photoUpperLeft_2 = ADC;
	
	
	//upper right1 input
	clear(ADCSRA,ADEN); // Disable ADC system to change MUX bit
	//Set MUX bit to F4
	clear(ADCSRB,MUX5);
	set(ADMUX,MUX2);
	clear(ADMUX,MUX1);
	clear(ADMUX,MUX0);
	//Enable the system
	set(ADCSRA,ADEN);
	//Start the conversion
	set(ADCSRA,ADSC);
	
	while(!check(ADCSRA,ADIF)){} //ADC conversion ongoing
	set(ADCSRA,ADIF);// Clear the flag
	
	photoLowerLeft_1 = ADC;
	
	//upper right 2 input
	clear(ADCSRA,ADEN); // Disable ADC system to change MUX bit
	//Set MUX bit to F5
	clear(ADCSRB,MUX5);
	set(ADMUX,MUX2);
	clear(ADMUX,MUX1);
	set(ADMUX,MUX0);
	//Enable the system
	set(ADCSRA,ADEN);
	set(ADCSRA,ADSC); //Start the conversion
	
	while(!check(ADCSRA,ADIF)){} //ADC conversion ongoing
	set(ADCSRA,ADIF);// Clear the flag
	
	photoLowerLeft_2 = ADC;
	
	//lower left 2
	clear(ADCSRA,ADEN); // Disable ADC system to change MUX bit
	//Set MUX bit to F6
	clear(ADCSRB,MUX5);
	set(ADMUX,MUX2);
	set(ADMUX,MUX1);
	clear(ADMUX,MUX0);
	//Enable the system
	set(ADCSRA,ADEN);
	set(ADCSRA,ADSC); //Start the conversion
	
	while(!check(ADCSRA,ADIF)){} //ADC conversion ongoing
	set(ADCSRA,ADIF);// Clear the flag
	
	photo_straight_down = ADC;

	//lower left 1 input
	clear(ADCSRA,ADEN); // Disable ADC system to change MUX bit
	//Set MUX bit to F7
	clear(ADCSRB,MUX5);
	set(ADMUX,MUX2);
	set(ADMUX,MUX1);
	set(ADMUX,MUX0);
	//Enable the system
	set(ADCSRA,ADEN);
	set(ADCSRA,ADSC); //Start the conversion
	
	while(!check(ADCSRA,ADIF)){} //ADC conversion ongoing
	set(ADCSRA,ADIF);// Clear the flag
	
	photoLowerRight_2 = ADC;

	//lower right 1 input
	clear(ADCSRA,ADEN); // Disable ADC system to change MUX bit
	//Set MUX bit to D4
	set(ADCSRB,MUX5);
	clear(ADMUX,MUX2);
	clear(ADMUX,MUX1);
	clear(ADMUX,MUX0);
	//Enable the system
	set(ADCSRA,ADEN);
	set(ADCSRA,ADSC); //Start the conversion
	
	while(!check(ADCSRA,ADIF)){} //ADC conversion ongoing
	set(ADCSRA,ADIF);// Clear the flag
	
	photoLowerRight_1 = ADC;

	//lower right 2 input
	clear(ADCSRA,ADEN); // Disable ADC system to change MUX bit
	//Set MUX bit to D6
	set(ADCSRB,MUX5);
	clear(ADMUX,MUX2);
	clear(ADMUX,MUX1);
	set(ADMUX,MUX0);
	//Enable the system
	set(ADCSRA,ADEN);
	set(ADCSRA,ADSC); //Start the conversion
	
	while(!check(ADCSRA,ADIF)){} //ADC conversion ongoing
	set(ADCSRA,ADIF);// Clear the flag
	
	photoCenter = ADC;

	//center input
	clear(ADCSRA,ADEN); // Disable ADC system to change MUX bit
	//Set MUX bit to D7
	set(ADCSRB,MUX5);
	clear(ADMUX,MUX2);
	set(ADMUX,MUX1);
	clear(ADMUX,MUX0);
	//Enable the system
	set(ADCSRA,ADEN);
	set(ADCSRA,ADSC); //Start the conversion
	
	while(!check(ADCSRA,ADIF)){} //ADC conversion ongoing
	set(ADCSRA,ADIF);// Clear the flag
	
	photoUpperRight_2 = ADC;

	//straight down input
	clear(ADCSRA,ADEN); // Disable ADC system to change MUX bit
	//Set MUX bit to B4
	set(ADCSRB,MUX5);
	clear(ADMUX,MUX2);
	set(ADMUX,MUX1);
	set(ADMUX,MUX0);
	//Enable the system
	set(ADCSRA,ADEN);
	set(ADCSRA,ADSC); //Start the conversion
	
	while(!check(ADCSRA,ADIF)){} //ADC conversion ongoing
	set(ADCSRA,ADIF);// Clear the flag
	
	photoUpperRight_1 = ADC;
}

void read_in_terminal(void){
	//enable usb subsystem
	m_red(ON);
	m_usb_init();
	while(!m_usb_isconnected()){};
	m_red(OFF);
}

void whereamI(unsigned int* wii_data){
	
	int max = 0;
	int min = 1024;
	int i = 0;
	int j = 0;
	int count = 0;
	int count2 = 0;
	int count3 = 0;
	int maxPoints[4] ={0,0,0,0};
	int minPoints[4] ={0,0,0,0};
	int centerX = 0;
	int centerY = 0;
	int thetaX = 0;
	int thetaY = 0;
	int cameraCenterX = 512;
	int cameraCenterY = 386;
	double o1[2] = {0,0};
	
	double scale = (11.08/37.795276);
	long wii_data_long[12];
	//tranform wii data
	for (i = 0; i <12 ; i++){
		wii_data_long[i] = (long)wii_data[i];
	}
	//test = (long)wii_data[0]*(long)wii_data[0];
	//use star 2 and 4 to find angle: angle = atan2(x4-x2,y2-y4)
	//Calculate six distances for four stars
	for(i=0; i <= 2; i++){
		for(j = i+1; j <=3 ; j ++){
			distance[count] =(long) sqrt((wii_data[i*3]-wii_data[j*3])*(wii_data[i*3]-wii_data[j*3]) + (wii_data[i*3+1]-wii_data[j*3 +1])*(wii_data[i*3+1]-wii_data[j*3 +1]));
			count += 1;
		}
	}
	// Find the longest between two stars and extract two stars
	//If Four Points are 0, 3, 1 and 4 ------> X1 and X2, Y1 and Y2 -------->Star1 and Star2
	//If Four Points are 0, 6, 1 and 7 ------> X1 and X3, Y1 and Y3 -------->Star1 and Star3
	//If Four Points are 0, 9, 1 and 10 ------> X1 and X4, Y1 and Y4 -------->Star1 and Star4
	//If Four Points are 3, 6, 4 and 7 ------> X2 and X3, Y2 and Y3 -------->Star2 and Star3
	//If Four Points are 3, 9, 4 and 10 ------> X2 and X4, Y2 and Y4 -------->Star2 and Star4
	//If Four Points are 6, 9, 7 and 10 ------> X3 and X4, Y3 and Y4 -------->Star3 and Star4
	for(i=0; i <=2; i++){
		for(j=i+1;j<=3;j++){
			if(distance[count2] > max){
				max = distance[count2];
				maxPoints[0] = i*3;
				maxPoints[1] = j*3;
				maxPoints[2] = i*3 + 1;
				maxPoints[3] = j*3 + 1;
			}
			count2 = count2 +1;
		}
	}
	//Find the shortest distance for the four stars
	//If Four Points are 0, 3, 1 and 4 ------> X1 and X2, Y1 and Y2 -------->Star1 and Star2
	//If Four Points are 0, 6, 1 and 7 ------> X1 and X3, Y1 and Y3 -------->Star1 and Star3
	//If Four Points are 0, 9, 1 and 10 ------> X1 and X4, Y1 and Y4 -------->Star1 and Star4
	//If Four Points are 3, 6, 4 and 7 ------> X2 and X3, Y2 and Y3 -------->Star2 and Star3
	//If Four Points are 3, 9, 4 and 10 ------> X2 and X4, Y2 and Y4 -------->Star2 and Star4
	//If Four Points are 6, 9, 7 and 10 ------> X3 and X4, Y3 and Y4 -------->Star3 and Star4
	for(i=0; i <=2; i++){
		for(j=i+1;j<=3;j++){
			if(distance[count3] < min){
				min = distance[count3];
				minPoints[0] = i*3;
				minPoints[1] = j*3 ;
				minPoints[2] = i*3 + 1;
				minPoints[3] = j*3 + 1;
			}
			count3 = count3 +1;
		}
	}
	//Find the Center Position of the four stars
	centerX = (wii_data[maxPoints[0]] + wii_data[maxPoints[1]])/2;
	centerY = (wii_data[maxPoints[2]] + wii_data[maxPoints[3]])/2;
	
	//Find robot's angle in the rink using atan2
	if (maxPoints[0] == minPoints[0] || maxPoints[0] == minPoints[1]){
		thetaX = wii_data[maxPoints[1]] - wii_data[maxPoints[0]];
		thetaY = wii_data[maxPoints[2]] - wii_data[maxPoints[3]];
		robot_angle = atan2(thetaX,thetaY) * (180.0/pi);
		}else if (maxPoints[1] == minPoints[0] || maxPoints[1] == minPoints[1]){
		thetaX = wii_data[maxPoints[0]] - wii_data[maxPoints[1]];
		thetaY = wii_data[maxPoints[3]] - wii_data[maxPoints[2]];
		robot_angle = atan2(thetaX,thetaY) * (180.0/pi);
	}
	
	
	
	//Expressing Frame0 in frame one by multiplying the Rotation Matrix
	o1[0] = -centerX * cos((robot_angle/(180.0/pi))) - centerY * sin((robot_angle/(180.0/pi)));
	o1[1] = centerX * sin((robot_angle/(180.0/pi))) - centerY * cos((robot_angle/(180.0/pi)));
	
	//Calculating Robot's Posistion in the rink
	robotPosition[0] = (cameraCenterX * cos(-(robot_angle/(180.0/pi))) - cameraCenterY * sin(-(robot_angle/(180.0/pi))) + o1[0])*scale;
	robotPosition[1] = (cameraCenterX * sin(-(robot_angle/(180.0/pi))) + cameraCenterY * cos(-(robot_angle/(180.0/pi))) + o1[1])*scale;

}

int checkWillData(){
	if(wii_data[0]==1023 || wii_data[1]==1023 || wii_data[3] == 1023 || wii_data[4] == 1023 || wii_data[6] ==1023 || wii_data[7] ==1023 || wii_data[9] ==1023 || wii_data[10] ==1023){
		return -1;
	}
	else{
		return 1;
	}
}

void initializeWii(){
	m_bus_init();
	m_wii_open();
	
}

void Determine_max_point(){
	photo[0] = photoCenter;
	photo[1] = photoLowerRight_1;
	photo[2] = photoLowerRight_2;
	photo[3] = photoUpperRight_2;
	photo[4] = photoUpperRight_1;
	photo[5] = photoLowerLeft_1;
	photo[6] = photoLowerLeft_2;
	photo[7] = photoUpperLeft_2;
	photo[8] = photoUpperLeft_1;
	
	//max = 0;
	//max_number = 0;
	//determine which is max
	for (i = 0; i < 9 ; i++){
		if (photo[i] > maxee){
			maxee = photo[i];
			max = maxee;
			max_number = i;
		}
		
	}
	maxee = 0;
}

void Clock_source_1(void){
	//timer mode 15
	set(TCCR1B,WGM13);
	set(TCCR1B,WGM12);
	set(TCCR1A,WGM11);
	set(TCCR1A,WGM10);
	
	//set up the prescaler to 1/64
	clear(TCCR1B,CS12);
	set(TCCR1B,CS11);
	set(TCCR1B,CS10);
	
	//compare output OC1B (clear at OCR1B, set at rollover)
	set(TCCR1A,COM1B1);
	clear(TCCR1A,COM1B0);
	//compare output OC1C (clear at OCR1C, set at rollover)
	set(TCCR1A,COM1C1);
	clear(TCCR1A,COM1C0);
	//300Hz
	OCR1A = 833;
	OCR1B = 0;
	//OCR1C  right motor
	OCR1C = 0;
	//set B6 B7 output (control PWM)
	set(DDRB,6);
	set(DDRB,7);
	//set C7 always high (Motor direction)
	set(DDRB,3);
	set(PORTB,3);
	//enable interrupt
	sei();
}

void ADC_init(void)
{
	/*set voltage reference to default*/
	clear(ADMUX,REFS1);
	set(ADMUX,REFS0);
	
	//set ADC prescaler
	set(ADCSRA,ADPS2); //Prescale ADC to 125 Khz
	set(ADCSRA,ADPS1);
	set(ADCSRA,ADPS0);
	
	//Set free running mode
	set(ADCSRA, ADATE);
	
	/*disabling digital inputs*/
	set(DIDR0,ADC0D);
	set(DIDR0,ADC1D);
	set(DIDR0,ADC4D);
	set(DIDR0,ADC5D);
	set(DIDR0,ADC6D);
	set(DIDR0,ADC7D);
	set(DIDR2,ADC8D);
	set(DIDR2,ADC9D);
	set(DIDR2,ADC10D);
	//let F4-7 be normal pin
	m_disableJTAG();
	 
}

void checkWirelessIsOn(){
	m_red(ON);
	while (!m_rf_open(CHANNEL, RXADDRESS, PACKET_LENGTH)) {
	}
	m_red(OFF);
}

void initializeWireless(){
	m_rf_open(CHANNEL, RXADDRESS, PACKET_LENGTH);
	//m_rf_open(CHANNEL,RXADDRESS,5);
}

void go_to_goal(){
	if (test == 2){
		
		if(75 < robot_angle && robot_angle < 105){
			//goal is at x = 115, y = 0
			if(robotPosition[1] > 0){
				OCR1B = 600;
				OCR1C = 833;
				}else if(robotPosition[1] < 0){
				OCR1B = 833;
				OCR1C = 600;
				}else if(robotPosition[1] == 0){
				OCR1B = 833;
				OCR1C = 833;
			}
			
			}else if(robot_angle < -90 || (robot_angle < 180 && robot_angle > 105)){
			OCR1B = 833; // B6 left wheel
			OCR1C = 100; // B7 right wheel
			}else if(robot_angle < 85 && robot_angle > -90 ){
			OCR1B = 100; // left wheel
			OCR1C = 833; // right wheel
		}
	}
	if (test == 1){
		
		if((-105 < robot_angle) && (robot_angle < -75)){
			//goal is at x = -115, y = 0
			if(robotPosition[1] > 0){
				OCR1B = 833;
				OCR1C = 600;
				}else if(robotPosition[1] < 0){
				OCR1B = 600;
				OCR1C = 833;
				}else if(robotPosition[1] == 0){
				OCR1B = 833;
				OCR1C = 833;
			}
			
			}else if((robot_angle < -105) || (robot_angle > 90)){
			OCR1B = 100; // B6 left wheel
			OCR1C = 833; // B7 right wheel
			}else if((robot_angle > -75) && (robot_angle < 90) ){
			OCR1B = 833; // left wheel
			OCR1C = 100; // right wheel
		}
	}
}

	
	
		





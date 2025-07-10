
#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include "USART.h"


// ========== ??????? PID ==========
float Kp, Ki, Kd;
float error;
float previous_error = 0;
float integral = 0, derivative = 0;
float output = 0;

// ========== ??????? LCD ==========
#define LCD_MODE 4

#define LCD_DATA_DIRECTION   DDRC
#define LCD_CONTROL_DIRECTION DDRC
#define LCD_DATA_PORT        PORTC
#define LCD_CONTROL_PORT     PORTC

#define RS 1
#define EN 2

void lcd_send_fallingEdge();
void lcd_cmd(unsigned char cmd);
void lcd_data(unsigned char data);
void init_lcd();
void display_clear_lcd();
void lcd_send_string(const char *data_string);
void lcd_send_number(uint16_t num);
void lcd_send_float(float value, uint8_t decimal_places);

// ========== ???? PWM ==========
void PWM_init() {
	DDRB |= (1 << PB3); // OC0
	TCCR0 |= (1 << WGM01) | (1 << WGM00);
	TCCR0 |= (1 << COM01);
	TCCR0 |= (1 << CS01) | (1 << CS00); // prescaler = 64
}

void set_pwm(uint8_t duty) {
	OCR0 = duty;
}

// ========== ???? ADC ==========
void ADC_init() {
	ADMUX = (1 << REFS0); // AVCC
	ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0); // Division factor = 8
}

uint16_t ADC_read(uint8_t channel) {
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

// ========== ?????? ?? ??????? ==========
void motor_forward() {
	PORTB |= (1 << PB0);
	PORTB &= ~(1 << PB1);
}

void motor_reverse() {
	PORTB &= ~(1 << PB0);
	PORTB |= (1 << PB1);
}

// ========== Main ==========
int main(void) {
	DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB3); // PB0, PB1: ????? ??????? - PB3: PWM

	PWM_init();
	ADC_init();
	init_lcd();
	UART_vInit(9600);

	char buffer[17];
	uint8_t toggle = 0;

	while (1) {
		uint16_t setpoint ;
		uint16_t feedback = ADC_read(0);   // PA0 -> FEEDBACK
		 
		
		
			UART_vReceiveHandler();  

			if (line_ready) {
				setpoint = atoi(uart_buffer);  
				line_ready = 0;
			
			}

		// ????? PID ?? Potentiometers
		uint16_t kp_adc = ADC_read(2); // PA2
		uint16_t ki_adc = ADC_read(3); // PA3
		uint16_t kd_adc = ADC_read(4); // PA4

		Kp = (kp_adc / 1023.0) * 10.0;
		Ki = (ki_adc / 1023.0) * 10.0;
		Kd = (kd_adc / 1023.0) * 10.0;

		// PID calculations
		error = (int16_t)setpoint - (int16_t)feedback;
		integral += error;
		if (integral > 500) integral = 500;
		if (integral < -500) integral = -500;

		derivative = error - previous_error;
		output = Kp * error + Ki * integral + Kd * derivative;

		if (error > 0) {
			motor_forward();
		} else {
			motor_reverse();
			output = -output;
		}

		if (output > 255) output = 255;
		if (output < 0) output = 0;

		set_pwm((uint8_t)output);
		previous_error = error;

		// === LCD Display ===
		lcd_cmd(0x80); // ????? ?????
		lcd_send_string("S:");
		itoa(setpoint, buffer, 10);
		lcd_send_string(buffer);
		lcd_send_string(" F:");
		itoa(feedback, buffer, 10);
		lcd_send_string(buffer);

		lcd_cmd(0xC0); // ????? ??????
		if (toggle == 0) {
			lcd_send_string("Kp:");
			lcd_send_float(Kp, 1);
			lcd_send_string(" Ki:");
			lcd_send_float(Ki, 1);
		} else {
			lcd_send_string("Kd:");
			lcd_send_float(Kd, 1);
			lcd_send_string(" E:");
			itoa(error, buffer, 10);
			lcd_send_string(buffer);
		}

		toggle = !toggle; // ????? ????? ?? ???
		_delay_ms(500);
		display_clear_lcd();
	}
}

// ========== ???? LCD ==========
void lcd_cmd(unsigned char cmd) {
	#if LCD_MODE == 8
	LCD_DATA_PORT = cmd;
	LCD_CONTROL_PORT &= ~(1 << RS);
	lcd_send_fallingEdge();

	#elif LCD_MODE == 4
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x87) | ((cmd & 0xF0) >> 1);
	LCD_CONTROL_PORT &= ~(1 << RS);
	lcd_send_fallingEdge();
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x87) | ((cmd << 3) & 0x78);
	lcd_send_fallingEdge();
	#endif

	_delay_ms(2);
}

void lcd_data(unsigned char data) {
	#if LCD_MODE == 8
	LCD_DATA_PORT = data;
	LCD_CONTROL_PORT |= (1 << RS);
	lcd_send_fallingEdge();

	#elif LCD_MODE == 4
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x87) | ((data & 0xF0) >> 1);
	LCD_CONTROL_PORT |= (1 << RS);
	lcd_send_fallingEdge();
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x87) | ((data << 3) & 0x78);
	lcd_send_fallingEdge();
	#endif

	_delay_ms(2);
}

void init_lcd() {
	LCD_CONTROL_DIRECTION |= (1 << RS) | (1 << EN);
	#if LCD_MODE == 8
	LCD_DATA_DIRECTION |= 0xFF;
	_delay_ms(40);
	lcd_cmd(0x38);
	_delay_ms(1);
	lcd_cmd(0x0C);
	_delay_ms(1);
	display_clear_lcd();
	lcd_cmd(0x06);
	_delay_ms(1);

	#elif LCD_MODE == 4
	LCD_DATA_DIRECTION |= 0x78;
	_delay_ms(40);
	lcd_cmd(0x02);
	_delay_ms(5);
	lcd_cmd(0x28);
	_delay_ms(1);
	lcd_cmd(0x0C);
	_delay_ms(1);
	display_clear_lcd();
	lcd_cmd(0x06);
	_delay_ms(1);
	#endif
}

void display_clear_lcd() {
	lcd_cmd(0x01);
	_delay_ms(2);
}

void lcd_send_string(const char *data_string) {
	while (*data_string) {
		lcd_data(*data_string++);
	}
}

void lcd_send_fallingEdge() {
	LCD_CONTROL_PORT |= (1 << EN);
	_delay_ms(1);
	LCD_CONTROL_PORT &= ~(1 << EN);
	_delay_ms(1);
}

// ??? ??? ???? ??? ???? LCD
void lcd_send_float(float value, uint8_t decimal_places) {
	char float_buffer[10];
	dtostrf(value, 4, decimal_places, float_buffer);
	lcd_send_string(float_buffer);
}




//////////////////////////////////////////////////////////////////////***********************/////////////////////////////////////
/*
 * USART.c
 *
 * Created: 10/6/2019 1:20:08 PM
 *  Author: Mohamed Zaghlol
 */ 

#define F_CPU 8000000UL
#include <util/delay.h>
#include <avr/io.h>
#include "std_macros.h"

#define BUFFER_SIZE 8
char uart_buffer[BUFFER_SIZE];
uint8_t uart_index = 0;
volatile uint8_t line_ready = 0;


void UART_vInit(unsigned long baud)
{
	/*1 - Choose baud rate that will be used by sender and receiver by writing to UBRRL/UBRRH*/
	unsigned short UBRR ;
	UBRR=(F_CPU/(16*baud))-1 ;
	UBRRH=(unsigned char)(UBRR>>8);
	UBRRL=(unsigned char)UBRR;
	/*2 - Enable USART Sender & Receiver*/
	SET_BIT(UCSRB,TXEN);
	SET_BIT(UCSRB,RXEN);
	/*3 - Choose number of data bits to be sent,parity and stop bits from UCSRC
	, We will work with 8 bits data,1 stop bit and no parity bits*/
	UCSRC=(1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1);
}

void UART_vSendData(char data)
{
	/*Wait for UDR transmit buffer to be empty*/
	while(READ_BIT(UCSRA,UDRE)==0);
	/*Put data to UDR transmit buffer transmit*/
	UDR=data ;
}

char UART_u8Available(void)
{
	return READ_BIT(UCSRA, RXC); 
}

char UART_u8ReceiveData(void)
{
	/*Wait for UDR receive buffer to be filled with data*/
	while(READ_BIT(UCSRA,RXC)==0);
	/*Receive data from UDR receive buffer*/
	return UDR ;
}
void UART_vSendstring( char *ptr)
{
	while(*ptr!=0)
	{
		UART_vSendData(*ptr);
		ptr++;
		_delay_ms(100);
	}
}

void UART_vReadLine(char* buffer, unsigned char maxLength)
{
	char c;
	unsigned char i = 0;

	while (i < maxLength - 1) {
		c = UART_u8ReceiveData();

 		if (c == '\n' || c == '\r') {
			break;
		}

		buffer[i++] = c;
	}

	buffer[i] = '\0';  
}

 


 void UART_vReceiveHandler(void)
 {
	 if (UART_u8Available()) {
		 char c = UART_u8ReceiveData();

		 if (c == '\n' || c == '\r') {
			 uart_buffer[uart_index] = '\0';  
			 uart_index = 0;
			 line_ready = 1;  
			 } else {
			 if (uart_index < BUFFER_SIZE - 1) {
				 uart_buffer[uart_index++] = c;
			 }
		 }
	 }
 }

////////////////////////////////////////////*********************/////////////////////////////////////////////////



#ifndef USART_H_
#define USART_H_

#define BUFFER_SIZE 8

extern volatile uint8_t line_ready;
extern char uart_buffer[BUFFER_SIZE]; 

void UART_vInit(unsigned long baud);
char UART_u8Available(void);
void UART_vReceiveHandler(void);
void UART_vReadLine(char* buffer, unsigned char maxLength);
void UART_vSendData(char data);
char UART_u8ReceiveData(void);

void UART_vSendstring( char *ptr);

#endif /* USART_H_ */



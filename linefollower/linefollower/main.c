#ifndef F_CPU
#define F_CPU 16000000UL
#endif
//avrdude -c usbasp -p m32 -U lfuse:r:-:h -U hfuse:r:-:h
//lfuse: 0xff
//hfuse: 0x99
//default: lfuse:0xe1 hfuse:0x99

#ifndef _BV
#define _BV(bit)				(1<<(bit))
#endif

#ifndef sbi
#define sbi(reg,bit)		reg |= (_BV(bit))
#endif

#ifndef cbi
#define cbi(reg,bit)		reg &= ~(_BV(bit))
#endif

#ifndef tbi
#define tbi(reg,bit)		reg ^= (_BV(bit))
#endif

#define	bit_is_set(sfr, bit)				(_SFR_BYTE(sfr) & _BV(bit))
#define bit_is_clear(sfr, bit)				(!(_SFR_BYTE(sfr) & _BV(bit)))
#define loop_until_bit_is_set(sfr, bit)		do { } while (bit_is_clear(sfr, bit))
#define loop_until_bit_is_clear(sfr, bit)	do { } while (bit_is_set(sfr, bit))
#define motor_left_PWM_pin		OCR1B
#define motor_right_PWM_pin		OCR1A

//define port where encoder is connected
#define ROTPORT PORTB
#define ROTDDR DDRB
#define ROTPIN PINB
//define rotary encoder pins
#define ROTPA PB2
#define ROTPB PB3
//#define ROTPBUTTON PB4
//define macros to check status
#define ROTA !((1<<ROTPA)&ROTPIN)
#define ROTB !((1<<ROTPB)&ROTPIN)
//#define ROTCLICK !((1<<ROTPBUTTON)&ROTPIN)
//


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

uint8_t rotarystatus;
uint8_t rotarystatus1;
int8_t dir_count;

int32_t i_pid = 0;
int32_t prevError = 0;

float kp = 150;
float ki = 0.2;
float kd = 80;

void RotaryInit(void)
{
	//set pins as input
	ROTDDR &= ~((1<<ROTPA)|(1<<ROTPB)); //|(1<<ROTPBUTTON));
	//enable interrnal pullups;
	ROTPORT |= (1<<ROTPA)|(1<<ROTPB); //|(1<<ROTPBUTTON);
	
	rotarystatus = 0;
}

void RotaryInit1(void)
{
	/* set PD2 and PD3 as input */
	DDRD &=~ (1 << 2);				/* PD2 and PD3 as input */
	DDRD &=~ (1 << 3);
	PORTD |= (1 << 3)|(1 << 2);   /* PD2 and PD3 pull-up enabled   */

	GICR |= (1<<INT0)|(1<<INT1);		/* enable INT0 and INT1 */
	MCUCR |= (1<<ISC01)|(1<<ISC11)|(1<<ISC10); /* INT0 - falling edge, INT1 - reising edge */
	
	rotarystatus1 = 0;
}

void RotaryCheckStatus(void)
{
	//reading rotary and button
	//check if rotation is left
	if(ROTA & (!ROTB)){
		loop_until_bit_is_set(ROTPIN, ROTPA);
		if (ROTB)
		rotarystatus=1;
		//check if rotation is right
		}else if(ROTB & (!ROTA)){
		loop_until_bit_is_set(ROTPIN, ROTPB);
		if (ROTA)
		rotarystatus=2;
		}else if (ROTA & ROTB){
		loop_until_bit_is_set(ROTPIN, ROTPA);
		if (ROTB)
		rotarystatus=1;
		else rotarystatus=2;
	}
	//check button status
	/*if (ROTCLICK)
	{
		rotarystatus=3;
	}*/
}

//reset status
void RotaryResetStatus(void)
{
	rotarystatus = 0;
	rotarystatus1 = 0;
}

void Timer2_Init(void)
{
	TCNT2=0x00;
}

void Timer2_Start(void)
{
	TCCR2|=(1<<CS22)|(1<<CS21); //prescaller 256 ~122 interrupts/s
	TIMSK|=(1<<TOIE2);//Enable Timer0 Overflow interrupts
}

ISR(INT0_vect)
{
	if(!bit_is_clear(PIND, PD3))
	{
		rotarystatus1 = 1;
	}
	else
	{
		rotarystatus1 = 0;
	}
}

ISR(INT1_vect)
{
	if(!bit_is_clear(PIND, PD2))
	{
		rotarystatus1 = 2;
	}
	else
	{
		rotarystatus1 = 0;
	}
}

ISR(TIMER2_OVF_vect)
{
	RotaryCheckStatus();
}

void init_motors(){
	
	//set motor_1 direction pins as output
	sbi(DDRD, 0);
	sbi(DDRD, 1);

	//set motor_2 direction pins as output
	sbi(DDRD, 6);
	sbi(DDRD, 7);
	
	//set enable PWM pins OC1B and OC1A as output
	sbi(DDRD, 4);
	sbi(DDRD, 5);
	
	//Fast PWM 8 bit
	TCCR1A |= (1<<WGM10);
	TCCR1B |= (1<<WGM12);
	
	//Clear OC1A/OC1B on Compare Match, set OC1A/OC1B at BOTTOM
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1);
	
	//Prescaler 64 fpwm = 976,5Hz
	//TCCR1B |= (1<<CS10) | (1<<CS11);
	//TCCR1B |= (1<<CS11); //7812.5Hz
	//TCCR1B |= (1<<CS10) | (1<<CS12); //61Hz
	TCCR1B |= (1<<CS10); //62745.1Hz
	
	
	OCR1A = 0;         //kana? A = 0
	OCR1B = 0;         //kana? B = 0
	
}

/*
void count(){
	if(bit_is_set(PINA, PA1)){
		dir_count = -20;
	}
	else if(bit_is_set(PINA, PA4))	{
		dir_count = -10;
	}
	else if(bit_is_set(PINA, PA5)){
		dir_count = 10;
	}
	else{
		dir_count = 20;
	}
}
*/

float readSensors(){
	uint8_t sensor1,sensor2, sensor3, sensor4;
	float sum = 0;
	
	if(bit_is_clear(PINA, PA1)){
		sensor1 = 1;
	}
	else{
		sensor1 = 0;
	}
	
	if(bit_is_clear(PINA, PA4)){
		sensor2 = 1;
	}
	else{
		sensor2 = 0;
	}
	
	if(bit_is_clear(PINA, PA5)){
		sensor3 = 1;
	}
	else{
		sensor3 = 0;
	}
	
	if(bit_is_clear(PINA, PA7)){
		sensor4 = 1;
	}
	else{
		sensor4 = 0;
	}
	
	//if there is no signal
	if(bit_is_set(PINA, PA1) && bit_is_set(PINA, PA4) && bit_is_set(PINA, PA5) && bit_is_set(PINA, PA7)){
		sbi(PORTC, 7);
		return 0;
	}
	
	cbi(PORTC, 7);
	
	//calculate weight
	sum = (float)(sensor1*1 + sensor2*2 + sensor3*3 + sensor4*4);
	sum = (float)(sum / (sensor1 + sensor2 + sensor3 + sensor4));
	
	return sum;
}

//current value will become previous value
//set_value is the point between sensors
float PID_calculate(float current_value, float set_value){
	
	float pid;
	float error;
	float p_pid, d_pid;
	
	error = set_value - current_value;
	
	p_pid =  error * kp;
	i_pid += error * ki;
	d_pid = (error - prevError) * kd;
	
	pid = p_pid + i_pid + d_pid;
	prevError = error;
	
	return pid;
}


int main(void)
{
	//LED:
	//PC7
	//
	//MOTORS:
	//PD0 LEFT BACK
	//PD7 RIGHT BACK
	//PD1 LEFT FORWARD
	//PD6 RIGHT FORWARD
	//
	//PWM:
	//OCR1A - right motor
	//OCR1B - left motor
	
	
	//      sen(PA1/PA2)  sen(PA4/PA3)  sen(PA5)  sen(PA7)
	// sen(PA0)                                       sen(PA6)
	//
	
	//set line A as input [sensors]
	for(int i=0; i<=7;i++){
		cbi(DDRA, i);
	}
	
	//these pins are off
	sbi(DDRA, 2); //set PA2 as output
	sbi(DDRA, 3); //set PA3 as output
	
	//set pin for LED
	sbi(DDRC, 7); //set LED PIN as output
	
	
	init_motors();
	RotaryInit();
	RotaryInit1();
	Timer2_Init();
	Timer2_Start();
	sei();
	
	/*
	//set motor_1 direction pins as output
	sbi(DDRD, 0);
	sbi(DDRD, 1);

	//set motor_2 direction pins as output
	sbi(DDRD, 6);
	sbi(DDRD, 7);
    */
	
	//turn on enable pins
	//sbi(PORTD, 4);
	//sbi(PORTD, 5);
	
	//sbi(PORTC, 7);
	
	int i=0;
	dir_count = 0;
	float cur_value;
	float control_value;
	float prev_value;
	float ruch;
	
	sbi(PORTD, 1);
	sbi(PORTD, 6);
	
    while(1)
    {
		
		
		cur_value = readSensors();
		
		if(cur_value == 0){
			cur_value = prev_value;
		}
		
		control_value = PID_calculate(cur_value, 2.5);
		
		if(control_value < 0){
			ruch = ((-control_value) * 255) / 345;
			OCR1A = ruch;
			OCR1B = 255;
		}
		if(control_value > 0){
			ruch = (control_value * 255) / 345;
			OCR1A = 255;
			OCR1B = ruch;
		}
		
		prev_value = cur_value;

		
		
		/*
		if(control_value > 500)
			control_value = 500;
		if(control_value < -500)
			control_value = -500;
		
		if(control_value > 0){
			if(control_value > 255){
				OCR1A = control_value - 255;
			}
			else{
				OCR1A = 255 - control_value;
			}
			OCR1B = 255;
		}
		else{
			if(control_value < -255){
				OCR1B = -(control_value + 255);
			}
			else{
				OCR1B = 255 + control_value;
			}
			OCR1A = 255;
		}
		*/
		
		
		/*
		count();
		if (dir_count < 0)
		{
			OCR1A = 200;
			OCR1B = 100;
			sbi(PORTD, 1);
			sbi(PORTD, 6);
			_delay_ms(1000);
			cbi(PORTD, 1);
			cbi(PORTD, 6);
			_delay_ms(1000);
		}
		else
		{
			OCR1A = 100;
			OCR1B = 200;
			sbi(PORTD, 1);
			sbi(PORTD, 6);
			_delay_ms(1000);
			cbi(PORTD, 1);
			cbi(PORTD, 6);
			_delay_ms(1000);
		}
		
		*/
		
		
	    /*PORTC = 0xFF;
	    _delay_ms(1000);
	    PORTC = 0x00;
	    _delay_ms(1000);*/
		
			//rotarystatus == 1; do przodu lewy silnik
			
			
			
			
			/*
			if(dir_count < 0)
			{
				OCR1A = 200;
				OCR1B = 100;
				sbi(PORTD, 0);
				sbi(PORTD, 7);
				_delay_ms(1000);
				cbi(PORTD, 0);
				cbi(PORTD, 7);
				_delay_ms(1000);
			}
			else
			{
				OCR1A = 100;
				OCR1B = 200;
				sbi(PORTD, 0);
				sbi(PORTD, 7);
				_delay_ms(1000);
				cbi(PORTD, 0);
				cbi(PORTD, 7);
				_delay_ms(1000);
			}
			
			count();
			*/
			
			if(rotarystatus1 == 1){
				PORTC = 0xFF;
				RotaryResetStatus();
			}
			else
			{
				PORTC = 0x00;
				RotaryResetStatus();
			}
		
		
		
		if(rotarystatus == 2){
			PORTC = 0xFF;
			RotaryResetStatus();
		}
		else{
			PORTC = 0x00;
			RotaryResetStatus();
		}
		
		/*
		if(i==1){
			OCR1A = 250;
			OCR1B = 250;
			sbi(PORTD, 1);
			sbi(PORTD, 6);
			_delay_ms(8000);
			cbi(PORTD, 1);
			cbi(PORTD, 6);
			_delay_ms(1000);
			
			/*OCR1A = 100;
			OCR1B = 100;
			sbi(PORTD, 0);
			sbi(PORTD, 7);
			_delay_ms(1000);
			cbi(PORTD, 0);
			cbi(PORTD, 7);
			_delay_ms(1000);
			*/
		//}
		
    }
}


//led check
/*
		sbi(PORTC, 7);
		_delay_ms(1000);
		cbi(PORTC, 7);
		_delay_ms(1000);
		*/

//motor check
/*
		OCR1A = 250;
		OCR1B = 250;
		sbi(PORTD, 1);
		sbi(PORTD, 7);
		_delay_ms(8000);
		cbi(PORTD, 1);
		cbi(PORTD, 7);
		_delay_ms(1000);
		*/	


//sensor check
/*
		if(bit_is_set(PINA, PA7)){
			//PORTC = 0xFF;
			sbi(PORTC, 7);
		}
		if(bit_is_clear(PINA, PA7)){
			cbi(PORTC, 7);
		}*/
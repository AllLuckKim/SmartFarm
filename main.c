#define _CRT_SECRUE_NO_WARNINGS
#define F_CPU   16000000UL
#define SLA_W   0x70
#define SLA_R   0x71

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void uart0_init(uint32_t baudrate);
void twi0_init(void);
uint8_t twi0_read(uint8_t addr);
void twi0_write(uint8_t addr, uint8_t data);
void twi0_read_multi(int num, uint8_t addr, uint8_t* buf);
void error(uint8_t status);
uint8_t AHT20_Read_Status(void);
void AHT20_SendAC(void);
void AHT20_Read_CTdata(uint8_t *buf);
void ADC_start();
void system_init(void);      //맨 처음에 세팅하는 부분
void cooler_ON(uint8_t speed); //192-1을 speed에 넣으면 duty cycle 75%
void cooler_OFF(void);
void pump_ON(uint8_t speed);   //192-1을 speed에 넣으면 duty cycle 75%
void pump_OFF(void);
void window_open(uint8_t angle); //20주면 90도 돈다
void window_close(void);
void humid_on();
void humid_off();
void heet_on();
void heet_off();
void LED_on();
void LED_off();

uint32_t CT_data[2];
uint16_t adc_data_raw;
volatile uint8_t eoc_flag;
uint16_t adc_data;
uint16_t soil_humid;

int main(void)
{
	uint8_t buf[7];
	volatile int  c1,t1;
	
	uart0_init(9600UL);
	system_init();
	
	int set_LED = 0;
	int set_max_temp = 0;
	int set_min_temp = 0;
	int set_max_humid = 0;
	int set_min_humid = 0;
	int set_max_soil = 0;
	int set_min_soil = 0;

	twi0_init();

	printf("Set LED: on(1) off(0): ");
	scanf("%d", &set_LED);
	printf("Set max temp: ");
	scanf("%d", &set_max_temp);
	printf("Set min temp: ");
	scanf("%d", &set_min_temp);
	printf("Set max soil humid: ");
	scanf("%d", &set_max_soil);
	printf("Set min soil humid: ");
	scanf("%d", &set_min_soil);
	printf("Set max humid: ");
	scanf("%d", &set_max_humid);
	printf("Set min Humid: ");
	scanf("%d", &set_min_humid);
	
	
	while (1)
	{
		ADC_start();
		AHT20_Read_CTdata(buf);
		
		c1 = CT_data[0]*100/1024/1024;
		t1 = CT_data[1]*200/1024/1024-50;
		
		printf("온도 = %d, 습도 = %d\n, 토양 습도 = %d\n", t1, c1, soil_humid);
		
		if (soil_humid < set_min_soil)
		{
			pump_ON(255);
			
		}
		if (soil_humid > set_max_soil)
		{
			pump_OFF();
		}
		
		if (t1 < set_min_temp)
		{
			heet_on();
			if (c1 < set_min_humid)
			{
				humid_on();
				cooler_OFF();
				window_close();
			}
			else if (c1 > set_max_humid)
			{
				humid_off();
				cooler_ON(255);
				window_open(20);
			}
			else
			{
				cooler_OFF();
				window_close();
			}
			
		}
		else if(t1 > set_max_temp)
		{
			heet_off();
			if (c1 > set_max_humid)
			{
				humid_off();
				cooler_ON(255);
				window_open(20);
			}
			else if (c1 < set_min_humid)
			{
				humid_on();
				cooler_OFF();
				window_close();
			}
			else
			{
				cooler_ON(255);
				window_open(20);
			}
			
		}
		else
		{
			if (c1 < set_min_humid)
			{
				humid_on();
				cooler_OFF();
				window_close();
			}
			else if (c1 > set_max_humid)
			{
				humid_off();
				cooler_ON(255);
				window_open(20);
			}
		}
		
		if (set_LED == 1)
		{
			LED_on();
		}
		else if (set_LED == 0)
		{
			LED_off();
		}
	}
}

// Fscl = 100 kHz
void twi0_init(void)
{
	TWSR = 0;      // prescale = 1
	TWBR = 72;      // 16MHz/100kHz=160.
}

uint8_t twi0_read(uint8_t addr)
{
	// START
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
	while ((TWCR & (1 << TWINT)) == 0);
	;
	
	// SLA_W
	TWDR = SLA_W;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);

	// addr
	TWDR = addr;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);

	// Rs
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
	while ((TWCR & (1 << TWINT)) == 0);

	// SLA_R
	TWDR = SLA_R;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);

	// Data in (NAK)
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);

	// STOP
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

	return TWDR;
}

void twi0_write(uint8_t addr, uint8_t data)
{
	// START
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
	while ((TWCR & (1 << TWINT)) == 0);
	
	
	// SLA_W
	TWDR = SLA_W;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);

	// addr
	TWDR = addr;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);

	// data
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);

	// STOP
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

void twi0_read_multi(int num, uint8_t addr, uint8_t* buf)
{
	// START
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
	while ((TWCR & (1 << TWINT)) == 0);
	;
	
	// SLA_W
	TWDR = SLA_W;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);

	// addr
	TWDR = addr;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);

	// Rs
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
	while ((TWCR & (1 << TWINT)) == 0);

	// SLA_R
	TWDR = SLA_R;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);

	for (uint8_t i=0; i<num; i++)
	{
		if (i < num-1)
		{
			// Data in (ACK)
			TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
			while ((TWCR & (1 << TWINT)) == 0);
			buf[i] = TWDR;
		}
		else
		{
			// Data in (NAK)
			TWCR = (1 << TWINT) | (1 << TWEN);
			while ((TWCR & (1 << TWINT)) == 0);
			buf[i] = TWDR;
		}
	}

	// STOP
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

void error(uint8_t status)
{
	printf("Error Code: %02X\n", status & 0xF8);
	while(1);
}

uint8_t AHT20_Read_Status(void)
{
	// START
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
	while ((TWCR & (1 << TWINT)) == 0);
	

	// SLA_R
	TWDR = SLA_R;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);
	

	// Data in (NAK)
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);
	

	// STOP
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

	return TWDR;
}

void AHT20_Read_CTdata(uint8_t *buf)
{
	uint16_t cnt = 0;
	uint32_t data = 0;
	
	AHT20_SendAC();
	_delay_ms(80);
	
	while(((AHT20_Read_Status() & 0x80) == 0x80))
	{
		_delay_us(1000);
		if (cnt++ >= 100)
		{
			break;
		}
	}
	twi0_read_multi(6, SLA_R,buf);
	data = (data|buf[1])<<8;
	data = (data|buf[2])<<8;
	data = (data|buf[3]);
	data = data>>4;
	CT_data[0] = data;
	data = 0;
	data = (data|buf[3])<<8;
	data = (data|buf[4])<<8;
	data = (data|buf[5]);
	data = data&0xfffff;
	CT_data[1] = data;
	
}

void AHT20_SendAC(void)
{
	// START
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
	while ((TWCR & (1 << TWINT)) == 0);
	

	// SLA_W
	TWDR = SLA_W;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);

	TWDR = 0xac;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);
	
	TWDR = 0x33;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);

	TWDR = 0x00;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);
	
	// STOP
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

void ADC_start()
{
	ADCSRA |= (1<<6); //1 적어주면 ad변환 1회 진행, 값은 ADCW에 저장
	while((ADCSRA & (1 << 4)) ==0); // ADIF가 1이 될 때(변환이 끝나는 시점)까지 기다렸다가 넘어감
	ADCSRA |= (1<<4); //ADIF에 1을 적어주어 1이된 FLAG를 clear 시켜줌
	
	adc_data = ADCW; //변환된 데이터는 ADCW에 저장되있으며 이 값을 다른 변수에 복사해준다.
	soil_humid = adc_data*100/1023;
	//printf("ADC value = %d\n", adc_data); // Terminal 프로그램 사용시
	//printf("%d\n", adc_value); // adc_monitor 사용시
}

void system_init(void) //timer setting
{
	DDRB  |= (1 << 1) | (1 << 2);   //OC1A/B output
	DDRD  |= (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2);            //OC2B output
	
	ADMUX   = 0b01000000;
	ADCSRA   = 0b10000111;
	ADCSRB   = 0b011;
	
	TCCR0A   = 0b10;
	TCCR0B   = 0b100;   //divide by 256
	OCR0A   = 250 - 1;
	
	TCCR1A = 0xA1;   //COM1A, COM1B = non-inverting PWM, fast PWM-8bit mode: 5
	TCCR1B = 0x09;   //fast PWM-8bit mode: 5, TC1 Presclae: 1
	
	OCR1A  = 0;      //A block PWM duty cycle : 0
	OCR1B  = 0;      //B block PWM duty cycle : 0
	
	TCCR2A = 0x23;   //COM1B = non-inverting PWM, fast PWM mode: 3
	TCCR2B = 0x07;   //fast PWM mode: 3, TC2 Presclae: 1024
	
	OCR2B  = 0;
	
	TCNT1  = 0;      //TC1 Counter value= 0
	TCNT2  = 0;      //TC2 Counter value= 0;
	
	humid_off();
	heet_off();
	LED_off();
	
	
	sei();
}
void cooler_ON(uint8_t speed)   //TC1-OC1A
{
	OCR1A   = speed;   //A block PWM duty cycle set
}

void cooler_OFF()            //TC1-OC1A
{
	OCR1A  = 0;         //A block PWM duty cycle= 0
}

void pump_ON(uint8_t speed)   //TC1-OC1B
{
	OCR1B   = speed;   //B block PWM duty cycle set
}

void pump_OFF()               //TC1-OC1B
{
	OCR1B  = 0;         //B block PWM duty cycle= 0
}

void window_open(uint8_t angle)   //TC2-OC2B
{
	OCR2B  = angle;
}

void window_close()            //TC2-OC2B
{
	OCR2B  = 39;   //turn to 0
}

void humid_on()
{
	PORTD |= (1 << 2);
}

void humid_off()
{
	PORTD &= ~(1 << 2);
}

void heet_on()
{
	PORTD |= (1 << 5);
}

void heet_off()
{
	PORTD &= ~(1 << 5);
}

void LED_on()
{
	PORTD |= (1 << 4);
}

void LED_off()
{
	PORTD &= ~(1 << 4);
}

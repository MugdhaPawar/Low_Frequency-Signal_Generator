#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include<math.h>
#include<stdlib.h>
#include "tm4c123gh6pm.h"

//#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define PORTB5_LDAC  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))
int count,i,j=0,fieldstart[3],k,field_count;
float frequency,voltage,value,raw_1,raw_2,temp_1,temp_2;
uint32_t table[4096],accumulator=0,delta,higher_frequency,step,lower_frequency,f;
char str[100],type[100],fieldtype[3],functn[10],match_string[10],vtg[10],freq[10],adc_str[100];
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


// Initialize Hardware
void initHw()
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x08;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x08; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x18;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

// Configure UART0 pins
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 0x03;                           // default, added for clarity
	GPIO_PORTA_AFSEL_R |= 0x03;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

 	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure SSI2 pins for SPI configuration
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
    GPIO_PORTB_DIR_R |= 0xB0;                        // make bits 4 and 7 outputs
    GPIO_PORTB_DR2R_R |= 0xB0;                       // set drive strength to 2mA
	GPIO_PORTB_AFSEL_R |= 0xB0;                      // select alternative functions for MOSI, SCLK pins
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK | GPIO_PCTL_PB5_SSI2FSS; // map alt fns to SSI2
    GPIO_PORTB_DEN_R |= 0xB0;                        // enable digital operation on TX, CLK pins
    GPIO_PORTB_PUR_R |= 0x10;

    // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
    SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
    SSI2_CR1_R = 0;                                  // select master mode
    SSI2_CC_R = 0;                                   // select system clock as the clock source
    SSI2_CPSR_R = 20;                                // set bit rate to 2 MHz (if SR=0 in CR0)
    SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
    SSI2_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2

    PORTB5_LDAC=0;

    // Configure AN0 as an analog input
    SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
	GPIO_PORTE_AFSEL_R |= 0x0C;                      // select alternative functions for AN0 (PE3)
    GPIO_PORTE_DEN_R &= ~0x0C;                       // turn off digital operation on pin PE3
    GPIO_PORTE_AMSEL_R |= 0x0C;                      // turn on analog operation on pin PE3
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN2;                // disable sample sequencer 2 (SS2) for programming
    ADC0_EMUX_R = ADC_EMUX_EM2_PROCESSOR;            // select SS2 bit in ADCPSSI as trigger
    ADC0_SSMUX2_R = 0x20;                               // set first sample to AN0
    ADC0_SSCTL2_R = ADC_SSCTL2_END1;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN2;                 // enable SS3 for operation
}

void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}

void putsUart0(char* str)
{
	int i;
    for (i = 0; i < strlen(str); i++)
	  putcUart0(str[i]);
}

char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE);
	return UART0_DR_R & 0xFF;
}

// Blocking functn that returns only when SW1 is pressed
void waitPbPress()
{
	while(PUSH_BUTTON);
}

// waitMicrosecond
void waitMicrosecond(uint32_t us){
	__asm("WMS_LOOP0:   MOV  R1, #6");
    __asm("WMS_LOOP1:   SUB  R1, #1");
    __asm("             CBZ  R1, WMS_DONE1");
    __asm("             NOP");
    __asm("             NOP");
    __asm("             B    WMS_LOOP1");
    __asm("WMS_DONE1:   SUB  R0, #1");
    __asm("             CBZ  R0, WMS_DONE0");
	__asm("             NOP");
    __asm("             B    WMS_LOOP0");
    __asm("WMS_DONE0:");

}

void reset()
{
	NVIC_APINT_R=NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
}
 // step 2
 void GetString_Step1()
 {
	 for(count=0;count<100;count++)
	 {
		char c = getcUart0();

		if(c =='\b')
		{
			if(count > 0)
			{
				 count--;
				 str[count--]=c;
			}
			else
			{
			}
		}
		else if (c =='\r')
			 	 {
			 		 str[count] = 0;
			 		 break;
			 	 }
		else if(c > ' ')
		{
			 str[count]=c;
			 if(str[count] >= 65 && str[count] <= 90)
			 {
			 	str[count]=str[count]+32;
			 }
		}
	 }
 }

// step 3
void String_Step2()
{
	for (count=0;count<100;count++)
	{
		if(str[count] >= 48 && str[count] <= 57 )
		{
			str[count]=str[count];
		}
		else
		{
			if(str[count] >= 97 && str[count] <= 122)
			{
				str[count]=str[count];
			}
			else if(str[count] >= 45 && str[count] <= 46)
			{
				str[count]=str[count];
			}
			else
			{
				str[count]=0;
			}
		}
	}

	for (j=0;j<100;j++)
	{
		if(str[j] >= 97 && str[j] <= 122)
		{
			type[j]='a';
		}
		else if(str[j] >= 48 && str[j] <= 57 )
			{
				type[j]='n';
			}
		else if(str[j] == 45 || str[j] == 46 )
		{
			type[j]='n';
		}
	}

/*for(count=0;count<100;count++)
	{
    	fieldstart[count]=0;
    	fieldtype[count]=0;
    }
*/
	j=0;
	for (count=0;count<100;count++)
	{
		if(type[count]=='a'&& type[count-1]!='a')
		{
			fieldstart[j]=count;
			fieldtype[j]='a';
			j++;
		}
		else if(type[count]=='n'&& type[count-1]!='n')
		{
			fieldstart[j]=count;
			fieldtype[j]='n';
			j++;
		}
		else if(type[count]=='\0')
		{
		}
	}

k=0;
for (i=fieldstart[0];i<100;i++)
{
	if(str[i]!=0)
		{
		functn[k]=str[i];
		k++;
		}
	else
	{
		break;
	}
}

k=0;
for (i=fieldstart[1];i<100;i++)
{
	if(str[i]!=0)
		{
		freq[k]=str[i];
		k++;
		}
	else
		{
			break;
		}
}
k=0;
for (i=fieldstart[2];i<100;i++)
{
	if(str[i]!=0)
		{
		vtg[k]=str[i];
		k++;
		}
	else
		{
			break;
		}
}
}
void Arguement_Step4()
{
	frequency = atof(&freq[0]);
	voltage=atof(&vtg[0]);

}

bool iscommand(char match_string[10])
  {
	  return(strcmp(match_string,functn));
  }


 void dc_test_step_6()
 {
	 voltage=frequency;
	 if(voltage>0)
	 {
		 //SSI2_DR_R=(0x3070+0x800-(0x810-0x000)*voltage/5);
		 SSI2_DR_R=(0x37DB-(0x7DB)*voltage/5.08);
		 while (SSI2_SR_R & SSI_SR_BSY);
	 }
	 else if(voltage<0)
	 {
		 //SSI2_DR_R=(0x3070+0x800-(0x810-0xFFF)*voltage/4.98);
		 SSI2_DR_R=(0x37DB-(0x824)*voltage/4.96);
		 while (SSI2_SR_R & SSI_SR_BSY);
	 }
 }

void delta_()
{
 	delta=frequency*pow(2,32)/100000;
}

 void look_up_table_Step7()

 {

	 for(i=0;i<4096;i++)
	 {
		 value=sin((2*3.14*i)/4096);
		 if(sin(i)<0)
		 {
			 //table[i]=(0x3870-(0xFFF-0x800)*voltage*value/5.02);
			 table[i]=(0x37DB-(0x824)*voltage*value/4.96);
		 }
		 else
		 {
			// table[i]=(0x3870-(0x800-0x000)*voltage*value/4.98);
			 table[i]=(0x37DB-(0x7DB)*voltage*value/5.08);
		 }
	 }
	 delta_();
 }

void Timer1Isr()
{

	accumulator=accumulator+delta;
	SSI2_DR_R=table[accumulator>>20];
	TIMER1_ICR_R= TIMER_ICR_TATOCINT;
}

void Step_8()
{
	    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
	    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer

	    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as

	    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for
		    TIMER1_TAILR_R = 0x190;                      // set load value to

	    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
	    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on

	    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

int16_t read_Adc0_Ss2_step91()
{
    ADC0_PSSI_R |= ADC_PSSI_SS2;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO2_R;
}

int16_t read_Adc0_Ss2_step92()
{
    ADC0_PSSI_R |= ADC_PSSI_SS2;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO2_R;
}



   /* ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
        while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
                                  // get single result from the FIFO
    }

    ADC0_SSFIFO2_R=0;                           // get single result from the FIFO
}

temp_2=ADC0_SSFIFO2_R;
raw_2=((temp_2)/4096)*10;
*/
void look_up_table_Step10(uint32_t i)
{
	uint32_t del_frequency=i;
	for(i=0;i<4096;i++)
	{
		value=sin((2*3.14*i)/4096);
		if(sin(i)<0)
		{
			//table[i]=(0x3870-(0xFFF-0x800)*voltage*value/5.02);
			table[i]=(0x37DB-(0x824)*3*value/4.96);
		}
		else
		{
			// table[i]=(0x3870-(0x800-0x000)*voltage*value/4.98);
			table[i]=(0x37DB-(0x7DB)*3*value/5.08);
		}
	}
	delta=del_frequency*pow(2,32)/100000;
}
void sawtooth_look_up_table_Step11()
{
	for(i=0;i<4096;i++)
 	{
		float sawtooth=((-voltage/2048)*i+voltage);
		//table[i]=(0x3870-(0xFFF-0x800)*voltage*value/5.02);
		table[i]=(0x37DB-((0x824)*sawtooth/4.96));
 	}

 		// table[i]=(0x3870-(0x800-0x000)*voltage*value/4.98);
 		//table[i]=(0x37DB-(0x7DB)*voltage*value/5.08);
	 delta_();
}
void squarewave_look_up_table_Step12()
{
	for(i=0;i<4096;i++)
	{
		if(i<2048)
		{
			table[i]=0x3000+0x800-((0xFFF-0X7FF)*(voltage)/5);
		}
		else if(i>2048)
		{
			table[i]=0x3000+0x800+((0xFFF-0X7FF)*(voltage)/5);
		}

	}
	 delta_();
}

int main()
{
	initHw();
 	GREEN_LED=1;
 	waitMicrosecond(500000);
 	GREEN_LED=0;
 	while(1)
 	{
 		putsUart0("\nEnter valid command and its arguments\n");
 		putsUart0("\n\r");
 		GetString_Step1();              //STEP 1
 		String_Step2();					//STEP 2 ,STEP 3

 		if(iscommand("dc")==0)			//STEP 5
 		{
 			if (fieldtype[0]=='a' && fieldtype[1]=='n'&& fieldtype[2]!='n')
 			{
 				Arguement_Step4();			//STEP 4
 				voltage=frequency;
 				if(voltage >=-5 && voltage <=5)
 				{
 					dc_test_step_6();			//STEP 6
 				}
 				else
 				{
 					putsUart0("\nEnter voltage between -5V to 5V\n");
 				}
 			}
 			else
 			{
 				putsUart0("\nEnter valid number of arguments\n");
 			}
 		}

 		else if(iscommand("sine")==0)	//STEP 5
 		{
 			if (fieldtype[0]=='a' && fieldtype[1]=='n' && fieldtype[2]=='n' && fieldtype[3]!='n' )
 			{
 				Arguement_Step4();			//STEP 4
 				if(frequency>=0 &&frequency<=10000 && voltage >=-5 && voltage <=5)
 				{
 					look_up_table_Step7();		//STEP 7
 					Step_8();					//STEP 8
 				}
 				else
 				{
 				 	putsUart0("\nEnter frequency less than 1000Hz and voltage between -5V to 5V\n");
 				}
 			}
 			else
 			{
 				putsUart0("\nEnter valid number of arguments\n");
 			}
 		}

 		else if(iscommand("sawtooth")==0)	//STEP 5
 		{
 			if (fieldtype[0]=='a' && fieldtype[1]=='n' && fieldtype[2]=='n' && fieldtype[3]!='n' )
 		 	{
 				Arguement_Step4();			//STEP 4
 		 		if(frequency>=0 &&frequency<=10000 && voltage >=-5 && voltage <=5)
 		 		{
 		 			sawtooth_look_up_table_Step11();		//STEP 7
 		 			Step_8();					//STEP 8
 		 		}
 		 		else
 		 		{
 		 			putsUart0("\nEnter frequency less than 1000Hz and voltage between -5V to 5V\n");
 		 		}
 		 	}
 		 	else
 		 	{
 		 		putsUart0("\nEnter valid number of arguments\n");
 		 	}
 		 }

 		else if(iscommand("square")==0)	//STEP 5
 		{
 			if (fieldtype[0]=='a' && fieldtype[1]=='n' && fieldtype[2]=='n' && fieldtype[3]!='n' )
 		 	{
 				Arguement_Step4();			//STEP 4
 		 		if(frequency>=0 &&frequency<=10000 && voltage >=-5 && voltage <=5)
 		 		{
 		 			squarewave_look_up_table_Step12();		//STEP 7
 		 		 	Step_8();					//STEP 8
 		 		}
 		 		else
 		 		{
 		 			putsUart0("\nEnter frequency less than 1000Hz and voltage between -5V to 5V\n");
 		 		}
 		 	}
 		 	else
 		 	{
 		 		putsUart0("\nEnter valid number of arguments\n");
 		 	}
 		}
 		else if(iscommand("sweep")==0)
 		{
 			if (fieldtype[0]=='a' && fieldtype[1]=='n' && fieldtype[2]=='n' && fieldtype[3]!='n' )
 			{
 				Arguement_Step4();
 				lower_frequency=frequency;
 				higher_frequency=voltage;
 				step=(higher_frequency-lower_frequency)/30;

 				//lookup table config
 				putsUart0((char*)"-------------------------------------------------------------------------------------------------------------------------------\r\n");
 				putsUart0((char*)" 						SWEEP FUNCTION STARTS FROM THE FREQUENCY											\r\n");
 				sprintf(str, "range is from %dHz to %dHz \r\n",lower_frequency,higher_frequency);
 				putsUart0((char*)str);
 				putsUart0((char*)"--------------------------------------------------------------------------------------------------------------------------------\r\n");
 				putsUart0((char*)"					   FREQUENCY						VOLTAGE											\r\n");

 				if(lower_frequency>1 && lower_frequency <= higher_frequency && higher_frequency<10000)
 		 		{
 					for(i=lower_frequency;i<higher_frequency;i+=step)
 					{
 						f=i;
 						look_up_table_Step10(i);
 						Step_8();
 						GREEN_LED = 1;
 						waitMicrosecond(600000);
 						GREEN_LED = 0;
// 						read_Adc0_Ss2_step9();
 						//float f=raw_1;                                vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 						waitMicrosecond(600000);
 						if(i<=1000)
 						{
 							temp_1=read_Adc0_Ss2_step91();
 							raw_1=((temp_1)/4096)*10;
 							sprintf(adc_str,"\r\n			%dHz									%0.1f V",i,raw_1);
 							putsUart0((char*)adc_str);
 						}
 						else
 						{
 							temp_2=read_Adc0_Ss2_step92();
 							raw_2=((temp_2)/4096)*10;
 							sprintf(adc_str,"\r\n			%dHz									%0.1f V",i,raw_2);
 							putsUart0((char*)adc_str);
 						}
 					}
 		 		}
 				else
 				{
 					putsUart0("\nEnter the lower frequency greater than 1 KHz and lower than higher frequency. Also enter higher frequency less than 10000 KHz\n");
 				}
 			}
 			else
 			{
 				putsUart0("\nEnter valid number of arguments\n");
 			}
 		}
 		else if(iscommand("reset")==0)
 		{
 			reset();
 		}
 		else
 		{
 			putsUart0("\nThe string you have entered is invalid\n");
 		}
 	}
}

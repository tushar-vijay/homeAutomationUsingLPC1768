#include <stdint.h>
#include "lcd.h"

#define ROW_PINS (0x0F << 4)
#define COL_PINS (0x0F << 0)

#define ALL_LED (0xFF << 19)
#define LED_PIN(X) (1 << X)		
#define BUZZER_PIN (1 << 27)	
#define RELAY_PIN (1 << 28)
#define odd (0x55<<19);
#define switch1 (0x01<< 11)
#define switch2 (0x01<< 12)


#define VREF       3.3 //Reference Voltage at VREFP pin, given VREFN = 0V(GND)
#define ADC_CLK_EN (1<<12)
#define SEL_AD0_2  (1<<2) //Select Channel AD0.1 
#define CLKDIV     (3 << 8)      //ADC clock-divider (ADC_CLOCK=PCLK/CLKDIV+1) = 1Mhz @ 4Mhz PCLK
#define PWRUP      (1<<21) //setting it to 0 will power it down
#define START_CNV  (1<<24) //001 for starting the conversion immediately
#define ADC_DONE   (1U<<31) //define it as unsigned value or compiler will throw #61-D warning int m

#define  T_COEFF 100.0f


#define Year 2024
#define Month 2
#define Day 7
#define Hour 9
#define Min 30
#define Sec 1


	


#define PASSWORD_LENGTH 4	   

int count=0;			

void pwm_config(void);
void pwm_signal(void);
void checkPassword(const char *password);  
void time_config(void);
void time1(char *date,char *time);
void adc_config(void);
void adc(int result,float volts,float temp,char *stemp);
int main()
{
uint8_t  i,j,val,k;
int input;
int result = 0;

	float volts = 0;

	float temp = 0;
  	char stemp[20];
char date[20];
char time[20];
int flag=0;
uint8_t scan[4]= {0xE,0xD,0xB,0x7};
char key[4][4] = {{'0','1','2','3'},
         		   {'4','5','6','7'},
				   {'8','9','A','B'},
				   {'C','D','E','F'}
				  };

char enteredPassword[PASSWORD_LENGTH + 1];
lcd_init();
time_config();
pwm_config();
adc_config();
	


LPC_GPIO1->FIODIR |= BUZZER_PIN;	
LPC_GPIO1->FIODIR |= RELAY_PIN;
LPC_GPIO2->FIODIR = ROW_PINS;
LPC_GPIO1->FIODIR |=odd;

LPC_GPIO2->FIODIR &= ~COL_PINS;
LPC_GPIO2->FIODIR &= ~switch1;
LPC_GPIO2->FIODIR &= ~switch2;







time1(date,time);
adc(result,volts,temp,stemp);
lcd_cmd_write(0x0C); 
lcd_str_write("Enter code: ");




while(1)
{







for(i=0;i<4;i++)
{

	


LPC_GPIO2->FIOCLR |= ROW_PINS; //clear row lines
LPC_GPIO2->FIOSET |= scan[i] << 4; //activate single row at a time
val = LPC_GPIO2->FIOPIN & COL_PINS;//read column lines
for(j=0;j<4;j++)
{
if(val == scan[j]) break; //if any key is pressed stop scanning  
}
if(val != 0x0F) // if key pressed in the scanned row, print key using lookup table
{
  lcd_data_write(key[i][j]);
                enteredPassword[strlen(enteredPassword)] = key[i][j];
                enteredPassword[PASSWORD_LENGTH] = '\0'; 
                delay(100);
}}
if (strlen(enteredPassword) == PASSWORD_LENGTH)
        {
			checkPassword(enteredPassword);
        	for (k = 0; k < PASSWORD_LENGTH; k++)
    		{
        		enteredPassword[k] = '\0';
    		}	 
			enteredPassword[0] = '\0';
		}
		flag=0;

 }

}


   


		 



void checkPassword(const char *password)  
{
int flag1=0;
int flag2=0;
	uint8_t k=0;
	lcd_cmd_write(0x01); 
    if (strcmp(password, "2222") == 0)
	{
        lcd_str_write("Correct Password");
		delay(1000);
		lcd_cmd_write(0x01);   
		LPC_GPIO1->FIOSET = RELAY_PIN;
		lcd_str_write("Door opened!!");
		delay(1000);
		lcd_cmd_write(0x01);
		lcd_str_write("WELCOME TO");
		lcd_cmd_write(0xC0);   
		lcd_str_write("HOME!");
		delay(4000);
		lcd_cmd_write(0x01);
		LPC_GPIO1->FIOCLR = RELAY_PIN;
		lcd_str_write("Door closed");
		
		lcd_cmd_write(0x01);

		lcd_str_write("PRESS SWITCH TO ");
		lcd_cmd_write(0xC0);
		lcd_str_write("ON FAN OR LED ");

		while(1){
		if(flag2==0){
	
		if(((LPC_GPIO2->FIOPIN)&(switch2))!=0){
		pwm_signal();
		lcd_cmd_write(0x01);
		lcd_str_write("MOTOR IS ON");
		flag2=1;
		}	 }

		if(flag1==0){			 
		if(((LPC_GPIO2->FIOPIN)&(switch1))!=0){  

			lcd_cmd_write(0x01);
		lcd_str_write("TURNING ON LEDS");
		LPC_GPIO1->FIOSET=odd;
		
		flag1=1;
		}

		

}
if(flag1==1){
if(((LPC_GPIO2->FIOPIN)&(switch1))!=0){
 		 
			lcd_cmd_write(0x01);
		lcd_str_write("TURNING OFF LEDS");
		
		LPC_GPIO1->FIOCLR=odd;
		
		flag1=0;
 
 }

	}

if(flag2==1){

if(((LPC_GPIO2->FIOPIN)&(switch2))!=0){
	//	pwm_signal();
		lcd_cmd_write(0x01);
		lcd_str_write("MOTOR IS OFF");
		flag2=0;
		}



}

  
  
		

		count=0;
    } 	}
	else
	{
        lcd_str_write("Wrong Password");
		count++;
		LPC_GPIO1 -> FIOSET = BUZZER_PIN;
		delay(100);
		LPC_GPIO1 -> FIOCLR = BUZZER_PIN;
		lcd_cmd_write(0x01);
	
			lcd_str_write("--Door locked--");
			delay(1000);
			count=0;
			lcd_cmd_write(0x01);
		
		
		
			lcd_str_write("Try again!!");
			delay(500);
		
		lcd_cmd_write(0x01);
	}

	}


void pwm_config(void) {
																									  
    LPC_PINCON->PINSEL3 &= ~(1 << 8);// P1.20 as PWM1.2
    LPC_PINCON->PINSEL3 |= (1 << 9);

	LPC_SC->PCONP |= (1 << 6); //PWM1.1 power/clk control enabled
	LPC_SC->PCLKSEL0 &=~(3<< 12); // Make 12 and 13 bit 0 

    LPC_PWM1->PR = 3;    // PR = 3, PCLK / (PR + 1) = 1MHz / (3+1) = 0.25MHz 
    LPC_PWM1->MR0 = 10000;             // Total period PWM cycle = 40ms
    LPC_PWM1->MCR |= (1 << 1);         // Reset on MR0
    LPC_PWM1->LER |= (1 << 0);         // Enable MR0 Latch

    LPC_PWM1->PCR |= (1 << 10);        // Enable PWM1 output

    LPC_PWM1->TCR = (1 << 0) | (1 << 3);  // Enable PWM Timer and PWM Mode

	}



    void pwm_signal(void) {
	     
        LPC_PWM1->MR2 = 10000;   // 100% duty cycle
        LPC_PWM1->LER |= (1 << 2);  // Enable MR2 Latch
        delay(1000);
		return;
    }

void time_config(void){
    LPC_RTC->CIIR = 0;  // Disable all interrupts
    LPC_RTC->CCR = 1;   // Enable RTC

    LPC_RTC->YEAR = Year;
    LPC_RTC->MONTH = Month;
    LPC_RTC->DOM = Day;
    LPC_RTC->HOUR = Hour;
    LPC_RTC->MIN = Min;
    LPC_RTC->SEC = Sec;
	return;
}


void time1(char *date,char *time){


while(1){
        lcd_cmd_write(0x01);
        sprintf(date,"DATE:%d-%d-%d",LPC_RTC->DOM,LPC_RTC->MONTH,LPC_RTC->YEAR);
		lcd_str_write(date);
		lcd_cmd_write(0xC0); // display on cursor off command
		sprintf(time,"Time: %d-%d-%d",LPC_RTC->HOUR,LPC_RTC->MIN,LPC_RTC->SEC);
		lcd_str_write(time);
		delay(200);
		lcd_cmd_write(0x01);
		delay(100);

		if(((LPC_GPIO2->FIOPIN)&(switch1))!=0){
		
		break;	}
		else continue;


	  }
	
		return;
}


void adc_config(void){
    LPC_PINCON->PINSEL1 |= (0x01<<18) ; //select AD0.2 for P0.25
	LPC_SC->PCONP |= ADC_CLK_EN; //Enable ADC clock
	LPC_ADC->ADCR =  PWRUP | CLKDIV | SEL_AD0_2;
	}

void adc(int result,float volts,float temp,char *stemp){

	while(1){
     	LPC_ADC->ADCR |= START_CNV; //Start new Conversion

		while((LPC_ADC->ADDR2 & ADC_DONE) == 0){} //Wait untill conversion is finished
		
		result = (LPC_ADC->ADDR2>>4) & 0xFFF; //12 bit Mask to extract result
		
		volts = (result*VREF)/4096.0; //Convert result to Voltage
		temp = volts * T_COEFF;
		sprintf(stemp,"Temp=%.2f 'C",temp);
		lcd_str_write(stemp);
		delay(500); //Slowing down Updates to 2 Updates per second
	    lcd_cmd_write(0x01);

		
		if(((LPC_GPIO2->FIOPIN)&(switch1))!=0){
		
		break;	}
		else continue;


	  

		} }

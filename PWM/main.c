
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "driverlib/adc.h"

#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/fpu.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "uartstdio.h"

#define N 245
uint32_t COUNT[1];

uint32_t CPU_FREC=16000000;
uint32_t PWM_FREC=20000;
uint32_t pwm_word;
int A;
int AL=0;
int NA=0;
int posicion;
int B;
int BL=0;
int NB=0;


/**
 * main.c
 */


void PORTFHandler(void){
    GPIOIntClear(GPIO_PORTF_BASE,GPIOIntStatus(GPIO_PORTF_BASE,true));//se limpia la bandera de la interrupcion
    A = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0);
        NA=NA+1;
        posicion = NA/N;
        AL = A;
        UARTprintf("%d \n",(int)((float)posicion));

}

int main(void)
{
//-------------------------------------SE CONFIGURA EL RELOJ DEL MICROCONTROLADOR A 16MHZ----------------------------------------------
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
//---------------------------------------------CONGIGURACION DEL ADC-------------------------------------------------------------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeADC(GPIO_PORTB_BASE,GPIO_PIN_5);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR,0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0,ADC_CTL_IE|ADC_CTL_END|ADC_CTL_CH11);
    ADCSequenceEnable(ADC0_BASE,3);
    ADCIntEnable(ADC0_BASE, 3);
//-------------------------------------------ADC PARA ENCODER---------------------------------------------------------------------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOIntRegister(GPIO_PORTF_BASE,PORTFHandler);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);
    IntMasterEnable();
//-----------------------------------------------PWM------------------------------------------------------------------------------------
    pwm_word = (1/PWM_FREC)*CPU_FREC;
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 800);

    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

//---------------------------------------SE INICIALIZAN UART0----------------------------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
        GPIOPinConfigure(GPIO_PA0_U0RX);
        GPIOPinConfigure(GPIO_PA1_U0TX);
        GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
        UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE );
        UARTStdioConfig(0, 115200, SysCtlClockGet());
//----------------------------------------ESPECIFICAMOS LOS CRACTERES QUE VAN A ENTRAR POR EL UART)-------------------

    while(1){

        ADCIntClear(ADC0_BASE, 3);
        ADCProcessorTrigger(ADC0_BASE, 3);
        while(!ADCIntStatus(ADC0_BASE, 3, false));
        ADCSequenceDataGet(ADC0_BASE, 3, COUNT);
        if(COUNT[0]<=450){
            COUNT[0]=450;
        }
        else if(COUNT[0]>=3645){
                    COUNT[0]=3645;
                }
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, COUNT[0]*800/4095);
        A = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0);
        /*A = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0);
        if(A != AL){
            NA=NA+1;
            posicion = NA/N*360;
            AL = A;
        }*/
        B = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_1);
        float adc=(float)COUNT[0];
        UARTprintf("%d \n",(int)adc);
        UARTprintf("%d \n",(int)A);
        /*if(COUNT[0]<1365){
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2, 0X00);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
        }
        else if(COUNT[0]>=1365 && COUNT[0]<2730){
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_1, 0X00);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        }
        else if(COUNT[0]>=2730 && COUNT[0]<4095){
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_1, 0X00);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
        }*/
    }
        return 0;
}
/*
//------------------------------------SE ACTIVA EL PERIFERICO B------------------------------------------------------------------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//------------------------------------SE ACTIVA COMO PIN ADC EL PB5--------------------------------------------------------------------
    GPIOPinTypeADC(GPIO_PORTB_BASE,GPIO_PIN_5);
//------------------------------------SE ACTIVA EL ADC0--------------------------------------------------------------------------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
//------------------------------------SE INDICA QUE EL ADC SE LEERA CUANDO EN EL PROCESADOR HAYA UN TRIGGER----------------------------
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
//------------------------------------SE ACTIVA EL COMPARADOR PARA EL ADC EN ESTE CASO SOLO SE LEE Y SE COMPARA UNA VEZ----------------
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH11|ADC_CTL_IE|ADC_CTL_END);
//------------------------------------SE ACTIVA EL ADC POR COMPLETO--------------------------------------------------------------------
    ADCSequenceEnable(ADC0_BASE,3);

 //------------------------------------------CONFIGURACION DEL PWM---------------------------------------------------------------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 400);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    while(1)
    {
        ADCIntClear(ADC0_BASE, 3);
        ADCProcessorTrigger(ADC0_BASE, 3);
        while(!ADCIntStatus(ADC0_BASE, 3, false));
        ADCSequenceDataGet(ADC0_BASE, 3, COUNT);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,COUNT[0]);
    }
	return 0;
}*/

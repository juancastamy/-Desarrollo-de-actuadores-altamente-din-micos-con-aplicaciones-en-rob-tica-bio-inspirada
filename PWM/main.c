#include "inc/hw_types.h"

#include "inc/hw_gpio.h"

#include "inc/hw_memmap.h"
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
#include "driverlib/qei.h"

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
float po;

/**
 * main.c
 */


/*void PORTFHandler(void){
    GPIOIntClear(GPIO_PORTF_BASE,GPIOIntStatus(GPIO_PORTF_BASE,true));//se limpia la bandera de la interrupcion
    A = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0);
        NA=NA+1;
        posicion = NA/N;
        AL = A;
        UARTprintf("%d \n",(int)((float)posicion));

}*/
uint32_t contador=0;
uint32_t posicio;
uint32_t velocidad;
int32_t direccion;

int main(void)
{
//-------------------------------------SE CONFIGURA EL RELOJ DEL MICROCONTROLADOR A 16MHZ----------------------------------------------
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
//---------------------------------------------CONGIGURACION DEL ADC-------------------------------------------------------------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeADC(GPIO_PORTB_BASE,GPIO_PIN_5);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR,0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0,ADC_CTL_IE|ADC_CTL_END|ADC_CTL_CH11);
    ADCSequenceEnable(ADC0_BASE,3);
    ADCIntEnable(ADC0_BASE, 3);
//-------------------------------------------ENCODER------------------------------------------------------------------------------------
//https://forum.43oh.com/topic/7170-using-harware-qei-on-tiva-launchpad/
    // Enable QEI Peripherals
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

        //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
        HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
        HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
        HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

        //Set Pins to be PHA0 and PHB0
        GPIOPinConfigure(GPIO_PD6_PHA0);
        GPIOPinConfigure(GPIO_PD7_PHB0);

        //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
        GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);

        //DISable peripheral and int before configuration
        QEIDisable(QEI0_BASE);
        QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

        // Configure quadrature encoder, use an arbitrary top limit of 1000
        QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 978);

        QEIVelocityConfigure(QEI0_BASE,QEI_VELDIV_1,SysCtlClockGet());
        QEIVelocityEnable(QEI0_BASE);
        // Enable the quadrature encoder.
        QEIEnable(QEI0_BASE);

        //Set position to a middle value so we can see if things are working
        QEIPositionSet(QEI0_BASE, 489);
//--------------------------------------------PINES DIGITALES PARA EL DRIVER----------------------------------------------------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x00);
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
//------------------------------------------------PUERTOS DIGITALES PARA DRIVER PUENTE H------------------------------------------------
/*    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlDelay(3);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, (GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_4));
*/
//---------------------------------------SE INICIALIZAN UART0----------------------------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE );
    UARTStdioConfig(0, 115200, SysCtlClockGet());

    while(1){

        ADCIntClear(ADC0_BASE, 3);
        ADCProcessorTrigger(ADC0_BASE, 3);
        while(!ADCIntStatus(ADC0_BASE, 3, false));
        ADCSequenceDataGet(ADC0_BASE, 3, COUNT);
        //GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x00);
        if(COUNT[0]<=100){
            COUNT[0]=100;
            //GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_4,GPIO_PIN_4);
            //GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x00);
        }
        else if(COUNT[0]>=3645){
            COUNT[0]=3995;
            //GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7,0x00);
            //GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,GPIO_PIN_5);
        }

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, COUNT[0]*800/4095);
        posicio=QEIPositionGet(QEI0_BASE);
        po=posicio*360/978;
        direccion=QEIDirectionGet(QEI0_BASE);
        velocidad=QEIVelocityGet(QEI0_BASE);
        //float adc=(float)COUNT[0];
        //UARTprintf("%d \n",(int)adc);

        /*float ve=(float)velocidad;
        float di=(float)direccion;
        UARTprintf("posicion: %d \n",(int)po);
        UARTprintf("velocidad: %d \n",(int)ve);
        UARTprintf("direccion: %d \n",(int)di);*/


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

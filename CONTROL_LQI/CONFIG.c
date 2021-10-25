/*
 * CONFIG.c
 *
 *  Created on: 17/09/2021
 *      Author: juan
 */

#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"


#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "driverlib/adc.h"
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

#include "driverlib/qei.h"

void CONFIG(void)
{
    //----------------------------------------INICIALIZACION DEL RELOJ-------------------------------------------
        SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL |  SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    //---------------------------------------INICIALIZACION DE PERIFERICOS---------------------------------------
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //---------------------------------------SE INICIALIZAN UART0----------------------------------
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
        GPIOPinConfigure(GPIO_PA0_U0RX);
        GPIOPinConfigure(GPIO_PA1_U0TX);
        GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
        UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE );
        IntEnable(INT_UART0);
        UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
     //  UARTStdioConfig(0, 115200, SysCtlClockGet());
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

        QEIVelocityConfigure(QEI0_BASE,QEI_VELDIV_1,SysCtlClockGet()/100);
        QEIVelocityEnable(QEI0_BASE);
        // Enable the quadrature encoder.
        QEIEnable(QEI0_BASE);

        //Set position to a middle value so we can see if things are working
        QEIPositionSet(QEI0_BASE, 28);
    //--------------------------------------------PINES DIGITALES PARA EL DRIVER----------------------------------------------------------
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5|GPIO_PIN_6);
        GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x00);
    //-----------------------------------------------PWM------------------------------------------------------------------------------------
        uint32_t PWM_FREC=20000;
        uint32_t pwm_word;
        pwm_word = ((SysCtlClockGet()/1)/PWM_FREC)-1;
        SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

        GPIOPinConfigure(GPIO_PB6_M0PWM0);
        PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, pwm_word);

        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
        PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    //------------------------------------------------PUERTOS DIGITALES PARA DRIVER PUENTE H------------------------------------------------
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        SysCtlDelay(3);
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, (GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_4));
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 100*pwm_word/4095);

    //---------------------------------------SE ACTIVA EL PERIFERICO DE TIMER0---------------------------------
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    //----------------------------------------INICIALIZACION DEL TIMER 0------------------------------------------
        TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);
    //----------------------------------------CONFIGURACION DEL TIEMPO DEL TIMER 0-------------------------------
        TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet()/100) -1);
    //-----------------------------------------SE ACTIVAN LAS INTERRUPCIONES DEL TIMER0------------------------------------

        IntEnable(INT_TIMER0A);
        //----------------------------------------SE ACTIVA LAS INTERRUPCIONES DEL TIMER-----------------------------
        TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);

    //----------------------------------------ACTIVAMOS LAS INTERRUPCIONES--------------------------------------
        IntMasterEnable();
        TimerEnable(TIMER0_BASE, TIMER_A);

        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
        GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
        GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
}

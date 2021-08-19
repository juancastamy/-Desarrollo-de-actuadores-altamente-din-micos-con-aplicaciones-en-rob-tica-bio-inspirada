
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

/**
 * main.c
 */

uint32_t COUNT[2];
float mensaje[2];
int main(void)
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
    UARTStdioConfig(0, 115200, SysCtlClockGet());
//---------------------------------------------CONGIGURACION DEL ADC-------------------------------------------------------------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeADC(GPIO_PORTB_BASE,GPIO_PIN_5);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR,0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0,ADC_CTL_IE|ADC_CTL_END|ADC_CTL_CH11);
    ADCSequenceEnable(ADC0_BASE,3);
    ADCIntEnable(ADC0_BASE, 3);

    while(1)
    {
    //-------------------------------------SE LIMPIA BANDERA DE ADC PARA INICIAR A LEER EL VALOR OBTENIDO--------------------------
        ADCIntClear(ADC0_BASE, 3);
        ADCProcessorTrigger(ADC0_BASE, 3);//SE ACTIVA EL TRIGUER PARA REALIZAR LECTURA
        while(!ADCIntStatus(ADC0_BASE, 3, false));//SE ESPERA QUE EL STATUAS DEL ADC SEA FALSE
        ADCSequenceDataGet(ADC0_BASE, 3, &COUNT[0]);// SE LEE ADC
        COUNT[1] = 1000;
        mensaje[0] = (float)COUNT[0];
        mensaje[1] = (float)COUNT[1];
        UARTprintf("%d",(int)mensaje[0]);
        UARTprintf("%d",(int)mensaje[1]);

    }
	return 0;
}

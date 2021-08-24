
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

uint32_t COUNT;
uint32_t prueba;
unsigned char a0;
unsigned char a1;
unsigned char a2;
unsigned char a3;
unsigned char data[4];
float mensaje1;
float mensaje2;
char n;
int s;

UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPut(UART0_BASE, *pui8Buffer++);
    }
}

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

    n=0;
    while(1)
    {
    //-------------------------------------SE LIMPIA BANDERA DE ADC PARA INICIAR A LEER EL VALOR OBTENIDO--------------------------
        ADCIntClear(ADC0_BASE, 3);
        ADCProcessorTrigger(ADC0_BASE, 3);//SE ACTIVA EL TRIGUER PARA REALIZAR LECTURA
        while(!ADCIntStatus(ADC0_BASE, 3, false));//SE ESPERA QUE EL STATUAS DEL ADC SEA FALSE
        ADCSequenceDataGet(ADC0_BASE, 3, &COUNT);// SE LEE ADC
        prueba = 1000;
        mensaje1 = (float)COUNT;
        mensaje2 = (float)prueba;

        data[0] = (COUNT >> 24) & 0xff;  /* high-order (leftmost) byte: bits 24-31 */
        data[1] = (COUNT >> 16) & 0xff;  /* next byte, counting from left: bits 16-23 */
        data[2] = (COUNT >>  8) & 0xff;  /* next byte, bits 8-15 */
        data[3] = COUNT & 0xff; //(prueba & 0xff);  /* low-order byte: bits 0-7 */

        if(n==0)
        {
            UARTCharPut(UART0_BASE,'1');
            n = UARTCharGet(UART0_BASE);
        }
        else if(n==1)
        {
            int i;
            for(i=0; i<=3; i++)
            {
                UARTCharPut(UART0_BASE,data[i]);
            }
            n=0;
        }


    }
	return 0;
}

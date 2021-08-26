
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
unsigned char motor[4];
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
#define N 245
uint32_t COUNT;

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


uint32_t contador=0;
uint32_t posicio;
float velocidad;
int32_t direccion;
uint32_t vel;
float ref;
float ref2;

float ref22;
float ref11;
float ref_1;
int pulso;

struct PID_values out;


struct PID_values
{
    float dif;//variable para almacenar ek
    float out;//variable para almacenar giro
    float Me;//variable para almacenar ek_1
    float ME;//variable para almacenar Ek_1
};

struct PID_values control_pid (float giro, float ek_1, float Ek_1, float x, float entrada)
{
    struct PID_values resultado;
    float ek;
    float ed;
    float Ek;
    float Kp=0.631;
    float Ki=0.000005;
    float Kd=0.82;
    float uk;
    //if (entrada>x)
    //{
        ek = entrada - x;
    //}
    /*else
    {
        ek = x - entrada;
    }*/

    ed = ek - ek_1;
    Ek = Ek_1+ek;
    Ek_1=Ek;
    ek_1=ek;
    uk = Kp*ek + Ki*Ek + Kd*ed ;
    giro=uk*(3995)/360;//7503;

    resultado.dif = ek;
    resultado.out = giro;
    resultado.Me = ek_1;
    resultado.ME = Ek_1;

    return resultado;


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
        QEIPositionSet(QEI0_BASE, 0);
    //--------------------------------------------PINES DIGITALES PARA EL DRIVER----------------------------------------------------------
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5|GPIO_PIN_6);
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
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        SysCtlDelay(3);
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, (GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_4));
    n=0;
    while(1)
    {
    //-------------------------------------SE LIMPIA BANDERA DE ADC PARA INICIAR A LEER EL VALOR OBTENIDO--------------------------
        ADCIntClear(ADC0_BASE, 3);
        ADCProcessorTrigger(ADC0_BASE, 3);//SE ACTIVA EL TRIGUER PARA REALIZAR LECTURA
        while(!ADCIntStatus(ADC0_BASE, 3, false));//SE ESPERA QUE EL STATUAS DEL ADC SEA FALSE
        ADCSequenceDataGet(ADC0_BASE, 3, &COUNT);// SE LEE ADC
        ref11 = (float)((COUNT-100)*3995/(3995-100));
        ref = ref11*360/(3995);/*7503*/

        prueba = 1000;
        mensaje1 = (float)COUNT;
        mensaje2 = (float)prueba;

        data[0] = ((uint32_t)ref >> 24) & 0xff;  /* high-order (leftmost) byte: bits 24-31 */
        data[1] = ((uint32_t)ref >> 16) & 0xff;  /* next byte, counting from left: bits 16-23 */
        data[2] = ((uint32_t)ref >>  8) & 0xff;  /* next byte, bits 8-15 */
        data[3] = (uint32_t)ref & 0xff; //(prueba & 0xff);  /* low-order byte: bits 0-7 */

        vel=(QEIPositionGet(QEI0_BASE)*360/979.2);
        motor[0]=(vel >> 24) & 0xff;
        motor[1]=(vel >> 16) & 0xff;
        motor[2]=(vel >> 8) & 0xff;
        motor[3]=vel;
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
            for(i=0; i<=3; i++)
            {
                UARTCharPut(UART0_BASE,motor[i]);
            }
            n=0;
        }
        if(COUNT<=100) //EL VALOR DEL ADC NO PUEDE SER MENOR A 100
               {
                           COUNT=100;
               }
               if(COUNT>=3995) //EL VALOR DEL ADC NO PUEDE SER MENOR A 3995
               {
                   COUNT=3995;
               }

       //--------------------------------SE REALIZA MAPEO AL VALOR DEL ADC OBTENIDO PARA MANTENERLO DENTRO DEL VALOR DE 100-3995---------



               velocidad=(float)(QEIPositionGet(QEI0_BASE)*360/979.2);

               out = control_pid (out.out, out.Me, out.ME, velocidad, ref);
               //GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,GPIO_PIN_5);
                //           GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0x00);
               if(out.dif > 2)
               {
                   GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,GPIO_PIN_5);
                   GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0x00);
               }

               else if(out.dif < -5)
              {
                   GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,GPIO_PIN_6);
                   GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5,0x00);
               }

               if(out.dif > -5 && out.dif < 5)
               {
                   GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,0x00);
                   GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0x00);
               }

               pulso = (int)out.out;

               PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pulso*800/4095);



    }
	return 0;
}

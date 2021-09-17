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
#include "CONFIG.h"
#include "driverlib/qei.h"

/**
 * main.c
 */

uint32_t COUNT;
//*******************************************************VARIABLE PARA CAMBIO DE GIRO DEL MOTOR***********************************************
int left;
int right;
//******************************************************VARIABLE PARA ALMACENAR EL MENSAJE POR COMUNICACION UART*******************************
unsigned char data[8];

char n;
int s;

uint32_t PWM_FREC=20000;
uint32_t pwm_word;



float velocidad;
float posicion;

float ref;
float ref11;
float ref2;
float ref22;


float pulso;

int update;

int fl;
int serial;

int BOTON;

struct Filtro pt;
struct PID_values out;

float giro;
float rev;
struct PID_values
{
    float difp;//variable para almacenar ek
    float outp;//variable para almacenar giro
    float Mep;//variable para almacenar ek_1
    float MEp;//variable para almacenar Ek_1
    float difv;//variable para almacenar ek
    float outv;//variable para almacenar giro
    float Mev;//variable para almacenar ek_1
    float MEv;//variable para almacenar Ek_1

};

struct Filtro
{
    float pot;
};

struct Filtro filtrado(float senal, float S, float alpha)
{
    struct Filtro potenciometro;
    s = (alpha*senal)+((1-alpha)*S);
    potenciometro.pot = s;

    return potenciometro;
}
struct PID_values control_pid (float uk, float ek_1, float Ek_1, float x, float entrada, float Kp, float Ki, float Kd, int n)
{
    struct PID_values resultado;
    float ek;
    float ed;
    float Ek;
    float dif;
    /*float Kp=0.00003;//0.2;//21
    float Ki=0.002;//0.007;//500;
    float Kd=0.004;//0.009;//0.25;*/
    if (n==0)
    {
        dif = entrada - x;

        ek=abs(dif);
        ed = ek - ek_1;
        Ek = Ek_1+ek;
        uk = (Kp*ek) + (Ki*Ek) + (Kd*ed);

        resultado.difp = dif;
        resultado.outp = uk;
        resultado.Mep = ek;
        resultado.MEp = Ek;
    }
    else
    {
        ek = entrada - x;
        ed = ek - ek_1;
        Ek = Ek_1+ek;
        uk = (Kp*ek) + (Ki*Ek) + (Kd*ed);
        resultado.difv = ek;
        resultado.outv = uk;
        resultado.Mev = ek;
        resultado.MEv = Ek;
    }

    return resultado;


}


void Timer0IntHandler(void){
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    update = 1;

}

void UART0IntHandler(void){
    UARTIntClear(UART0_BASE,UARTIntStatus(UART0_BASE, true)); //se limpia la bandera de la interrupcion
    n = UARTCharGet(UART0_BASE);//se lee lo que entra al UART0
    serial=1;
}

int main(void)
{

ref = 10;

    CONFIG();
    pwm_word = ((SysCtlClockGet()/1)/PWM_FREC)-1;

    while(1)
    {
        if(update == 1)
        {
            //-------------------------------------SE LIMPIA BANDERA DE ADC PARA INICIAR A LEER EL VALOR OBTENIDO--------------------------
           ADCIntClear(ADC0_BASE, 3);
           ADCProcessorTrigger(ADC0_BASE, 3);//SE ACTIVA EL TRIGUER PARA REALIZAR LECTURA
           while(!ADCIntStatus(ADC0_BASE, 3, false));//SE ESPERA QUE EL STATUAS DEL ADC SEA FALSE
           ADCSequenceDataGet(ADC0_BASE, 3, &COUNT);// SE LEE ADC

           if (fl==0)
           {
               fl=1;
               pt.pot = COUNT;
           }
           pt = filtrado(COUNT, pt.pot, 0.05);

           ref11 = (float)(0.08791*pt.pot);

           if (ref11 >= 350)
           {
               ref = 350;
           }
           else if (ref11 <=10)
           {
               ref = 10;
           }

           else
           {
               ref = ref11;
           }

           posicion = (float)(QEIPositionGet(QEI0_BASE)*360/979.2);

           out = control_pid (out.outp, out.Mep, out.MEp, posicion, ref,0.7,0.002,0.025,0);

           //giro=(float)(abs((out.outp* 10.54166)+200));

           ref22 = (float)(abs(out.outp* 9.583333) + 545);

           //ref22 = (float)(0.131752 * (giro -200));

           velocidad = (float)(QEIVelocityGet(QEI0_BASE)*100*60/979.2);

           out = control_pid (out.outv, out.Mev, out.MEv, velocidad, ref22, /*2*/0.5, /*0.09*/0.09, 0.000, 1);


           update = 0;
        }
     /* if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)==0)
       {
           BOTON=1;
       }
       if(BOTON==1)
       {
           ref=180;
       }*/

        //GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,GPIO_PIN_5);//se enciende pin//el encoder sumara
       // GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0x00);//se apaga pin
        rev=(float)(abs((out.outv* 7.59)+200));
        pulso = ref22;



       if(out.difp > 1 && out.difp > -1)
        {
           right = 1;
           left = 0;
        }
        else if(out.difp < -1 && out.difp < 1)
        {
           right = 0;
           left = 1;
        }
        else
        {
            right=0;
            left=0;
        }

        if(right == 1 && left == 0)
        {
            GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,GPIO_PIN_5);//se enciende pin//el encoder sumara
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0x00);//se apaga pin
        }
        else if(right == 0 && left == 1)
        {
           GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,0x00);//se apaga pin
           GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,GPIO_PIN_6);//se enciende pin
        }
        else if (right == 0 && left == 0)
        {
            GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,0x00);//se apaga pin
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0x00);//se apaga pin
        }

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pulso*pwm_word/4095);


        data[0] = ((uint32_t)ref >> 24) & 0xff;  //high-order (leftmost) byte: bits 24-31
        data[1] = ((uint32_t)ref >> 16) & 0xff;  //next byte, counting from left: bits 16-23
        data[2] = ((uint32_t)ref >>  8) & 0xff;  // next byte, bits 8-15
        data[3] = (uint32_t)ref & 0xff; //(prueba & 0xff);  //low-order byte: bits 0-7
        data[4]=((uint32_t)posicion >> 24) & 0xff;
        data[5]=((uint32_t)posicion >> 16) & 0xff;
        data[6]=((uint32_t)posicion >> 8) & 0xff;
        data[7]=(uint32_t)posicion;
        if(serial==0){
            UARTCharPut(UART0_BASE,'6');
        }
        if (n ==1)
        {
            UARTCharPut(UART0_BASE,'1');

            UARTCharPut(UART0_BASE,data[0]);
            UARTCharPut(UART0_BASE,data[1]);
            UARTCharPut(UART0_BASE,data[2]);
            UARTCharPut(UART0_BASE,data[3]);
            UARTCharPut(UART0_BASE,data[4]);
            UARTCharPut(UART0_BASE,data[5]);
            UARTCharPut(UART0_BASE,data[6]);
            UARTCharPut(UART0_BASE,data[7]);
            n=0;
        }

    }
    return 0;
}

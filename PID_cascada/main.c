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
uint32_t feedfoward;
uint32_t COUNT;
//*******************************************************VARIABLE PARA CAMBIO DE GIRO DEL MOTOR***********************************************
int left;
int right;
//******************************************************VARIABLE PARA ALMACENAR EL MENSAJE POR COMUNICACION UART*******************************
unsigned char data[16];

char n;
int s;

uint32_t PWM_FREC=20000;
uint32_t pwm_word;



float velocidad;
float posicion;
float sentido;

float posref;
float delta_pos;
float pos_pas;
float pos_tot;

float ref;
float ref11;
float ref2;
float ref22;


float pulso;

int update;

int fl;
int serial;
int j;
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
    ek = entrada - x;
    ed = ek - ek_1;
    Ek = Ek_1+ek;
    uk = (Kp*ek) + (Ki*Ek) + (Kd*ed);
    if (n==0)
        //out = control_pid (out.outv, out.Mev, out.MEv, velocidad, posref, 1, 0.01, 0, 1);
    {
        resultado.difp = ek;
        resultado.outp = uk;
        resultado.Mep = ek;
        resultado.MEp = Ek;
    }
    else
    {
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



void direccion(float dir)
{
    if(dir >= 1 )
    {
        GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,GPIO_PIN_5);//se enciende pin//el encoder sumara
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0x00);//se apaga pin
    }
    else if (dir <= -1)
    {
        GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,0x00);//se apaga pin
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,GPIO_PIN_6);//se enciende pin
    }
    else
    {
        GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,0x00);//se apaga pin
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0x00);//se apaga pin
    }
}


int main(void)
{

    ref = 10;
    //out.outp=5;
    CONFIG();
    pwm_word = ((SysCtlClockGet()/1)/PWM_FREC)-1;
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,GPIO_PIN_5);//se enciende pin//el encoder sumara
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0x00);//se apaga pin
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


           if (ref11 >= 350 && BOTON==0)
           {
               ref = 350;
           }
           else if (ref11 <=10 && BOTON==0)
           {
               ref = 10;
           }

           else
           {
               ref = ref11;
           }

           sentido = QEIDirectionGet(QEI0_BASE);
           posicion = (float)(QEIPositionGet(QEI0_BASE)*2*3.1416/979.2);

           //---------------------------------------obtencion de la posicion en valores positivos y negativos--------------------------------------
          delta_pos = posicion - pos_pas;
          pos_pas = posicion;
          if (sentido == 1)
          {

              if (delta_pos < -5)
              {
                  j=1;
              }
              if (j==0)
              {
                  pos_tot = pos_tot + delta_pos;

              }
              if (j==1)
              {
                  pos_tot = pos_tot + posicion;
                  j=0;
              }
          }
          if(sentido == -1)
          {


              if (delta_pos > 5)
              {
                  j=1;
              }
              if (j==0)
              {
                  pos_tot = pos_tot + delta_pos;
              }
              if (j==1)
              {
                  pos_tot = pos_tot - (2*3.1416-posicion);
                  j=0;
              }
          }

           //out = control_pid (out.outp, out.Mep, out.MEp, posicion, ref,5,0.5,1.5,0);//PID para control solo de posicion
           out = control_pid (out.outp, out.Mep, out.MEp, pos_tot, posref,5,0.01,0.001,0);//PID para control en cascada 3.5,0.000005,1.75,0
           giro=(float)(abs(out.outp)*4095)/12;

           ref22 = (float)(giro*52.35988/4095);
           velocidad = (float)(QEIVelocityGet(QEI0_BASE)*100*60/979.2)*0.10472;
           out = control_pid (out.outv, out.Mev, out.MEv, velocidad, ref22, 1.2, 0.005, 0.00001, 1);
           //******************************************FEEDFOWARD***********************************************************
          // feedfoward = (ref22 - 24.957)*8.1967;



          update = 0;
        }
        if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)==0)
        {
            BOTON=1;
        }



       /* if (giro > 3995)
        {
            giro = 3995;
        }
        else if (giro < 0)
        {
            giro = 0;
        }
        rev=(float)((out.outv*8.19) + feedfoward);

        if (rev<10)
        {
            rev = 10;
        }
        else if (rev>3995)
        {
            rev=3995;
        }*/
        pulso = 30 + (abs(out.outv)*4095)/12;

        direccion(out.outp);//se selecciona la direccion en la que el motor gira

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pulso*pwm_word/4095);//se envia la salida del controlador convertido a se�al PWM al pwm.

//****************************************************PREPARACION PARA ENVIO DE DATOS POSICION**********************************************
        data[0] = ((uint32_t)posref >> 24) & 0xff;  //se extrae los bits 24-31 de la se�al de referencia de posicion y se almacenan en la primera
                                                 //posicion del array data

        data[1] = ((uint32_t)posref >> 16) & 0xff;  //se extrae los bits 16-23 de la se�al de referencia de posicion y se almacenan en la segunda
                                                 //posicion del array data

        data[2] = ((uint32_t)posref >>  8) & 0xff;  //se extrae los bits 8-15 de la se�al de referencia de posicion y se almacenan en la tercera
                                                 //posicion del array data

        data[3] = (uint32_t)posref & 0xff; //se extrae los bits 0-7 de la se�al de referencia de posicion y se almacenan en la cuarta
                                        //posicion del array data

        data[4] = ((uint32_t)pos_tot >> 24) & 0xff; //se extrae los bits 24-31 de la se�al de posicion del encoder y se almacenan en la quinta
                                                     //posicion del array data

        data[5] = ((uint32_t)pos_tot >> 16) & 0xff; //se extrae los bits 16-23 de la se�al de posicion del encoder y se almacenan en la sexta
                                                     //posicion del array data

        data[6] = ((uint32_t)pos_tot >> 8) & 0xff; //se extrae los bits 8-15 de la se�al de posicion del encoder y se almacenan en la septima
                                                    //posicion del array data

        data[7] = (uint32_t)pos_tot; //se extrae los bits 0-7 de la se�al de posicion del encoder y se almacenan en la octava
                                      //posicion del array data
//*******************************************************ENVIO DATOS VELOCIDAD*********************************************************
        data[8] = ((uint32_t)ref22 >> 24) & 0xff;  //se extrae los bits 24-31 de la se�al de referencia de velocidad y se almacenan en la novena
                                                   //posicion del array data

        data[9] = ((uint32_t)ref22 >> 16) & 0xff;  //se extrae los bits 16-23 de la se�al de referencia de velocidad y se almacenan en la decima
                                                   //posicion del array data

        data[10] = ((uint32_t)ref22 >>  8) & 0xff;  //se extrae los bits 8-15 de la se�al de referencia de velocidad y se almacenan en la onceaba
                                                    //posicion del array data

        data[11] = (uint32_t)ref22 & 0xff; //se extrae los bits 0-7 de la se�al de referencia de velocidad y se almacenan en la doceaba
                                           //posicion del array data

        data[12]=((uint32_t)velocidad >> 24) & 0xff; //se extrae los bits 24-31 de la se�al de velocidad del encoder y se almacenan en la treceaba
                                                     //posicion del array data
        data[13]=((uint32_t)velocidad >> 16) & 0xff; //se extrae los bits 16-23 de la se�al de velocidad del encoder y se almacenan en la catorceaba
                                                     //posicion del array data
        data[14]=((uint32_t)velocidad >> 8) & 0xff; //se extrae los bits 8-15 de la se�al de velocidad del encoder y se almacenan en la quinceaba
                                                    //posicion del array data
        data[15]=(uint32_t)velocidad; //se extrae los bits 0-8 de la se�al de velocidad del encoder y se almacenan en la diez y seisaba
                                      //posicion del array data

        if(serial==0){
            UARTCharPut(UART0_BASE,'6');
        }
//*******************************SI SE RECIVE EL VALOR DE n=1 DEL UART SE INICIA A ENVIAR TODOS LOS DATOS EN ORDEN*******************************************
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

            UARTCharPut(UART0_BASE,data[8]);
            UARTCharPut(UART0_BASE,data[9]);
            UARTCharPut(UART0_BASE,data[10]);
            UARTCharPut(UART0_BASE,data[11]);
            UARTCharPut(UART0_BASE,data[12]);
            UARTCharPut(UART0_BASE,data[13]);
            UARTCharPut(UART0_BASE,data[14]);
            UARTCharPut(UART0_BASE,data[15]);
            n=0;
        }

    }
    return 0;
}

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
uint32_t corriente;
//*******************************************************VARIABLE PARA CAMBIO DE GIRO DEL MOTOR***********************************************
int left;
int right;
//******************************************************VARIABLE PARA ALMACENAR EL MENSAJE POR COMUNICACION UART*******************************
unsigned char data[20];

char n;
int s;

uint32_t PWM_FREC=40000;
uint32_t pwm_word;

float delta_pos=0;
float pos_tot=0;
float pos_pas=0;
float j=0;

float velocidad;
float posicion;
float sentido;
float current;
float volt_medido;

float ref;
float pos;

float tau;
float tau_e;

float error_tau;
float xI;
float dt;
float u;
/*
const float k1 = 0.1993;
const float k2 = -0.0500;
const float k3 = -0.0808;
const float k4 = -20.0000;
*/

/*
const float k1 = 1;
const float k2 = 1;
const float k3 = -1;
const float k4 = 5;
*/
/*
const float k1 = 2.5;
const float k2 = 1;
const float k3 = -1;
const float k4 = 8;
*/
float G=10;


const float k1 = 1;
const float k2 = 0.05;
const float k3 = -0.08;
const float k4 = 5;

//float Ek = 0;

float pulso;
float posible_I;

float posref = 0.00;

int update;

int fl;
int serial;

int BOTON;

struct Filtro pt;

float giro;
float rev;

float current_filt;
float lambda = 0.7;
float error_pos = 0;

float volts = 0;

float ref22;
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
    if(dir > 0 )
    {
        GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,GPIO_PIN_5);//se enciende pin//el encoder sumara
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0x00);//se apaga pin
        tau_e = abs(tau_e);
    }
    else if (dir < 0)
    {
        GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,0x00);//se apaga pin
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,GPIO_PIN_6);//se enciende pin
        tau_e = -abs(tau_e);
    }
    else
    {
        GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,0x00);//se apaga pin
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0x00);//se apaga pin
    }
}


int main(void)

{
    //1.3
    ref22=0;
    ref = 0;
    xI = 0;
    dt = 0.001;
    pulso = 10;
    ref = 0;
    CONFIG();
    pwm_word = ((SysCtlClockGet()/1)/PWM_FREC)-1;
    //GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,GPIO_PIN_5);//se enciende pin//el encoder sumara
    //GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0x00);//se apaga pin
    while(1)
    {
        if(update == 1)
        {
            //-------------------------------------SE LIMPIA BANDERA DE ADC PARA INICIAR A LEER EL VALOR OBTENIDO--------------------------
           ADCIntClear(ADC0_BASE, 3);
           ADCProcessorTrigger(ADC0_BASE, 3);//SE ACTIVA EL TRIGUER PARA REALIZAR LECTURA
           while(!ADCIntStatus(ADC0_BASE, 3, false));//SE ESPERA QUE EL STATUAS DEL ADC SEA FALSE
           ADCSequenceDataGet(ADC0_BASE, 3, &COUNT);// SE LEE ADC DE LA REFERENCIA

           ADCIntClear(ADC1_BASE, 0);
           ADCProcessorTrigger(ADC1_BASE, 0);//SE ACTIVA EL TRIGUER PARA REALIZAR LECTURA
           while(!ADCIntStatus(ADC1_BASE, 0, false));//SE ESPERA QUE EL STATUAS DEL ADC SEA FALSE
           ADCSequenceDataGet(ADC1_BASE, 0, &corriente);// SE LEE ADC DE RETROALIMENTACION DE LA CORRIENTE

           if (fl==0)
           {
               fl=1;
               pt.pot = COUNT;
           }

           pt = filtrado(COUNT, pt.pot, 0.05);

           //ref = (float)(pt.pot);

           tau = ref * (0.08 / 4095);

           posicion = (float)(QEIPositionGet(QEI0_BASE)*2*3.1416/979.2);//360/979.2);
           velocidad = (float)(QEIVelocityGet(QEI0_BASE)*100*60/979.2)*0.10472;
           sentido = QEIDirectionGet(QEI0_BASE);

           volt_medido = (((float)corriente)*3300)/4095;//esto esta en V
           current = (((float)corriente)/4.5232)*(3.3/4095)*(1000/0.144);//esto esta en mA

           current_filt = lambda * current_filt + (1-lambda) * current;

           tau_e = current_filt*0.065;

           //error_tau = tau - tau_e;

           //xI = xI + error_tau*dt;
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




           error_pos = -posref + pos_tot;
           if (error_pos > 0.01 || error_pos < -0.01)
           {
           xI = xI + error_pos*dt;
           }

           //u = -1*((k1*tau_e) + (k2*posicion) + (k3*velocidad) + (20*k4*xI)); no

          u = -(((G*k1*tau_e)/1000 + (G*k2*pos_tot) + (G*k3*velocidad)) + (G*k4*xI));//si


          //u = -0.2*50*(pos_tot - posref);

          // Ek=(pos_tot - posref);

           update = 0;
        }



        //direccion(u);//se selecciona la direccion en la que el motor gira

        //pulso = ((float)abs(u)*4095)/12;

        direccion(u);
        pulso = 30 +((float)abs(u)*4095)/12;
        //IF PARA REALIZAR LA TOMA DE DATOS CON UART
        /*  if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)==0)
        {
            BOTON = 1;
        }
        if(BOTON==1)
        {
            ref = 25;
            GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,GPIO_PIN_5);//se apaga pin
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0x00);//se apaga pin
        }
        else
        {
            GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,0x00);//se apaga pin
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0x00);//se apaga pin
        }*/
        /*if(pt.pot < 30)
        {
            ref = 30;
        }
        else if (pt.pot > 4055)
        {
            ref = 4055;
        }*/
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pulso*pwm_word/4095);//se envia la salida del controlador convertido a señal PWM al pwm.

        ref = posref * 180/3.1416;
        pos = pos_tot * 180/3.1416;


//****************************************************PREPARACION PARA ENVIO DE DATOS POSICION**********************************************
        data[0] = ((uint32_t)ref >> 24) & 0xff;  //se extrae los bits 24-31 de la señal de referencia de posicion y se almacenan en la primera
                                                 //posicion del array data

        data[1] = ((uint32_t)ref >> 16) & 0xff;  //se extrae los bits 16-23 de la señal de referencia de posicion y se almacenan en la segunda
                                                 //posicion del array data

        data[2] = ((uint32_t)ref >>  8) & 0xff;  //se extrae los bits 8-15 de la señal de referencia de posicion y se almacenan en la tercera
                                                 //posicion del array data

        data[3] = (uint32_t)ref & 0xff; //se extrae los bits 0-7 de la señal de referencia de posicion y se almacenan en la cuarta
                                        //posicion del array data

        data[4] = ((uint32_t)pos >> 24) & 0xff; //se extrae los bits 24-31 de la señal de posicion del encoder y se almacenan en la quinta
                                                     //posicion del array data

        data[5] = ((uint32_t)pos >> 16) & 0xff; //se extrae los bits 16-23 de la señal de posicion del encoder y se almacenan en la sexta
                                                     //posicion del array data

        data[6] = ((uint32_t)pos >> 8) & 0xff; //se extrae los bits 8-15 de la señal de posicion del encoder y se almacenan en la septima
                                                    //posicion del array data

        data[7] = (uint32_t)pos; //se extrae los bits 0-7 de la señal de posicion del encoder y se almacenan en la octava
                                      //posicion del array data
//*******************************************************ENVIO DATOS VELOCIDAD*********************************************************
        data[8] = ((uint32_t)current >> 24) & 0xff;  //se extrae los bits 24-31 de la señal de referencia de velocidad y se almacenan en la novena
                                                   //posicion del array data

        data[9] = ((uint32_t)current >> 16) & 0xff;  //se extrae los bits 16-23 de la señal de referencia de velocidad y se almacenan en la decima
                                                   //posicion del array data

        data[10] = ((uint32_t)current >>  8) & 0xff;  //se extrae los bits 8-15 de la señal de referencia de velocidad y se almacenan en la onceaba
                                                    //posicion del array data

        data[11] = (uint32_t)current & 0xff; //se extrae los bits 0-7 de la señal de referencia de velocidad y se almacenan en la doceaba
                                           //posicion del array data

        data[12]=((uint32_t)velocidad >> 24) & 0xff; //se extrae los bits 24-31 de la señal de velocidad del encoder y se almacenan en la treceaba
                                                     //posicion del array data
        data[13]=((uint32_t)velocidad >> 16) & 0xff; //se extrae los bits 16-23 de la señal de velocidad del encoder y se almacenan en la catorceaba
                                                     //posicion del array data
        data[14]=((uint32_t)velocidad >> 8) & 0xff; //se extrae los bits 8-15 de la señal de velocidad del encoder y se almacenan en la quinceaba
                                                    //posicion del array data
        data[15]=(uint32_t)velocidad; //se extrae los bits 0-8 de la señal de velocidad del encoder y se almacenan en la diez y seisaba
                                      //posicion del array data


        data[16]=((uint32_t)current_filt >> 24) & 0xff; //se extrae los bits 24-31 de la señal de velocidad del encoder y se almacenan en la treceaba
                                                     //posicion del array data
        data[17]=((uint32_t)current_filt >> 16) & 0xff; //se extrae los bits 16-23 de la señal de velocidad del encoder y se almacenan en la catorceaba
                                                     //posicion del array data
        data[18]=((uint32_t)current_filt >> 8) & 0xff; //se extrae los bits 8-15 de la señal de velocidad del encoder y se almacenan en la quinceaba
                                                    //posicion del array data
        data[19]=(uint32_t)current_filt; //se extrae los bits 0-8 de la señal de velocidad del encoder y se almacenan en la diez y seisaba
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
            UARTCharPut(UART0_BASE,data[16]);
            UARTCharPut(UART0_BASE,data[17]);
            UARTCharPut(UART0_BASE,data[18]);
            UARTCharPut(UART0_BASE,data[19]);
            n=0;
        }

    }
    return 0;
}

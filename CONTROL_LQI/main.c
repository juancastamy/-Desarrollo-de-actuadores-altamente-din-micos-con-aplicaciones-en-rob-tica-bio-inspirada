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

uint32_t corriente;//Valirbale para almacenar el valor ADC de corriente
//*******************************************************VARIABLE PARA CAMBIO DE GIRO DEL MOTOR***********************************************
int left;
int right;
//******************************************************VARIABLE PARA ALMACENAR EL MENSAJE POR COMUNICACION UART*******************************
unsigned char data[20]; //array donde se guardan los bytes para ser enviados por UART

char n;
int s;

uint32_t PWM_FREC=40000;//frecuencia a la que trabaja el PWM
uint32_t pwm_word;

//********************************VARIABLE PARA REALIZAR AJUSTE DE LA POSICION DE ENCODER**************************************************
float delta_pos = 0;
float pos_tot = 0;
float pos_pas = 0;
float j=0;



float velocidad;//variable para guardar el valor de la velocidad (rad/s)
float posicion;//variable para guardar el valor leido del encoder
float sentido;//variable para guardar el sentido en el que gira el motor
float current;//varaible para guardar la corriente estimada
float volt_medido;//variable para guardar el voltaje medido en el ADC

float ref_grados;//referencia de la posicion en grados
float salida_posicion;//salida de la posicion en grados

float tau_e;//valor del torque medido

float xI;//error integral
float dt;//periodo de muestreo
float u;//salida del LQI

float G=10;//ganancia para las constantes del controlador

//**************************************************constantes para el control LQI****************************************
const float k1 = 1;
const float k2 = 0.05;
const float k3 = -0.08;
const float k4 = 5;

float pulso;//valor de salida del controlador en un rango de 0 a 4095

float posref = 0.0;//referncia de la posicion

int update;//badnera para la interripcion del TIMER0

int fl;
int serial;

float giro;
float rev;

float current_filt;
float lambda = 0.7;
float error_pos = 0;

//**********************************INTERRIPCION DEL TIMER0 PARA LA TOMA DE MUESTRAS******************************
void Timer0IntHandler(void){
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    update = 1;

}

//******************************INTERRUPCION PARA REALIZAR LA COMUNICACION UART*********************************
void UART0IntHandler(void){
    UARTIntClear(UART0_BASE,UARTIntStatus(UART0_BASE, true)); //se limpia la bandera de la interrupcion
    n = UARTCharGet(UART0_BASE);//se lee lo que entra al UART0
    serial=1;
}

//*****************************FUNCION PARA DECIDIR EL CENTIDO DE GIRO DEL MOTOR*********************************
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
    xI = 0;
    dt = 0.001;//periodo de muestreo
    CONFIG();//funcion que se llama para realizar las configuraciones del microcontrolador
    pwm_word = ((SysCtlClockGet()/1)/PWM_FREC)-1;//factor por el que se tiene que multiplicar el valor que se envia al PWM

    while(1)
    {
        if(update == 1)
        {
            //-------------------------------------SE LIMPIA BANDERA DE ADC PARA INICIAR A LEER EL VALOR OBTENIDO--------------------------
           ADCIntClear(ADC1_BASE, 0);
           ADCProcessorTrigger(ADC1_BASE, 0);//SE ACTIVA EL TRIGUER PARA REALIZAR LECTURA
           while(!ADCIntStatus(ADC1_BASE, 0, false));//SE ESPERA QUE EL STATUAS DEL ADC SEA FALSE
           ADCSequenceDataGet(ADC1_BASE, 0, &corriente);// SE LEE ADC DE RETROALIMENTACION DE LA CORRIENTE

           posicion = (float)(QEIPositionGet(QEI0_BASE)*2*3.1416/979.2);//lectura de la posicion en radiantes
           velocidad = (float)(QEIVelocityGet(QEI0_BASE)*100*60/979.2)*0.10472;//lectura de la velocidad en rad/s
           sentido = QEIDirectionGet(QEI0_BASE);//direccion en la que gira el motor

           volt_medido = (((float)corriente)*3300)/4095;//esto esta en V
           current = (((float)corriente)/4.5232)*(3.3/4095)*(1000/0.144);//esto esta en mA

           current_filt = lambda * current_filt + (1-lambda) * current;//corriente filtrada

           tau_e = current_filt*0.065;//torque experimental medido

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

           error_pos = -posref + pos_tot;//error de posicion
           if (error_pos > 0.01 || error_pos < -0.01)//tolerancia para el error de integracion
           {
               xI = xI + error_pos*dt;
           }

          u = -(((G*k1*tau_e)/1000 + (G*k2*pos_tot) + (G*k3*velocidad)) + (G*k4*xI));//salida del controlador

          //u = -0.2*50*(pos_tot - posref);//control de posicion tipo P

           update = 0;//se limpia la bandera de la interrupcion
        }

        direccion(u);//se decide la direccion del motor
        pulso = 30 +((float)abs(u)*4095)/12;//se calcula el valor para enviar al PWM
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pulso*pwm_word/4095);//se envia la salida del controlador convertido a señal PWM al pwm.

        ref_grados = posref * 180/3.1416;//se convierte el valor de la referencia a grados
        salida_posicion = pos_tot * 180/3.1416;//se convieret a grados el valor leido del encoder


//****************************************************PREPARACION PARA ENVIO DE DATOS POSICION**********************************************
        data[0] = ((uint32_t)ref_grados >> 24) & 0xff;  //se extrae los bits 24-31 de la señal de referencia de posicion y se almacenan en la primera
                                                 //posicion del array data

        data[1] = ((uint32_t)ref_grados >> 16) & 0xff;  //se extrae los bits 16-23 de la señal de referencia de posicion y se almacenan en la segunda
                                                 //posicion del array data

        data[2] = ((uint32_t)ref_grados >>  8) & 0xff;  //se extrae los bits 8-15 de la señal de referencia de posicion y se almacenan en la tercera
                                                 //posicion del array data

        data[3] = (uint32_t)ref_grados & 0xff; //se extrae los bits 0-7 de la señal de referencia de posicion y se almacenan en la cuarta
                                        //posicion del array data

        data[4] = ((uint32_t)salida_posicion >> 24) & 0xff; //se extrae los bits 24-31 de la señal de posicion del encoder y se almacenan en la quinta
                                                     //posicion del array data

        data[5] = ((uint32_t)salida_posicion >> 16) & 0xff; //se extrae los bits 16-23 de la señal de posicion del encoder y se almacenan en la sexta
                                                     //posicion del array data

        data[6] = ((uint32_t)salida_posicion >> 8) & 0xff; //se extrae los bits 8-15 de la señal de posicion del encoder y se almacenan en la septima
                                                    //posicion del array data

        data[7] = (uint32_t)salida_posicion; //se extrae los bits 0-7 de la señal de posicion del encoder y se almacenan en la octava
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

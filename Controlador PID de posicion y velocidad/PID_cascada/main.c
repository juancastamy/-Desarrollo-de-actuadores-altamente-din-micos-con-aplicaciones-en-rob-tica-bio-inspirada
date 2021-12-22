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
uint32_t feedfoward; //Termin ode feedfoward para mejorar el controlador
uint32_t COUNT; //datos del ADC
//*******************************************************VARIABLE PARA CAMBIO DE GIRO DEL MOTOR***********************************************
int left;
int right;
//******************************************************VARIABLE PARA ALMACENAR EL MENSAJE POR COMUNICACION UART*******************************
unsigned char data[16]; //guardado de la data para realizar comunicacion UART

char n;
int s;

uint32_t PWM_FREC=20000; //Frecuencia de la señal del PWM
uint32_t pwm_word;



float velocidad;//variable para guardar la velocidad del motor
float posicion;//variable para guardar la posicion del motor
float sentido;//variable para guardar el centido en el que gira el motor

//*************LAS SIGUENTES VARIABLES SIRVEN PARA CALCULAR LA POSICION DEL MOTOR SIN QUE SE RECETEE EL ANGULO Y QUE PUEDA SER NEGATIVO********
float posref;
float delta_pos;
float pos_pas;
float pos_tot;
//***********************************************************************************************************************************************

float ref_filtrado;//fitrado de la senal de entrada
float lambda = 0.95;//factor de fitro IIR

float ref;//referencia del PID de posicion
float salida_posicion;//Salida de la posicion del encoder en grados
float ref2;//referencia del PID de velocidad
float ref_grados;//referencia de posicion en grados

float pulso;//valor de 0 a 4095 para enviar al PWM

int update;//bandera de la interrupcion del timer

int fl;
int serial;
int j;




float giro;//variable para la salida del PID de posicion en valores para el PWM

//***************************STRUCT PARA REALIZAR CONTROLADORES PID**************************************
struct PID_values out;
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

struct PID_values control_pid (float uk, float ek_1, float Ek_1, float x, float entrada, float Kp, float Ki, float Kd, int n)//Funcion del PID
{
    //uk -> salida del controlador
    //ek_1 -> valor pasado del error
    //Ek_1 -> valor pasado del error integrador
    //x -> valor retornado por el sensor
    //entrada -> valor de referencia
    //Kp -> contante P
    //Ki -> contante I
    //Kd -> contante D
    //n -> seleccion del guardado de los valores para control de posicion o el de velocidad

    struct PID_values resultado;
    float ek;
    float ed;
    float Ek;
    ek = entrada - x;
    ed = ek - ek_1;
    Ek = Ek_1+ek;
    uk = (Kp*ek) + (Ki*Ek) + (Kd*ed);
    if (n==0)
    {
        //se trunco el valor del error intregrador para que no provocase que el controlador se perdiera
        if (Ek >= 10)
        {
            Ek = 10;
        }
        else if (Ek <= -10)
        {
            Ek = -10;
        }

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
    //dir -> variable que decide el centido del giro
    //giro encontra de las manecillas del reloj
    if(dir > 0 )
    {
        GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,GPIO_PIN_5);//se enciende pin//el encoder sumara
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0x00);//se apaga pin
    }
    //giro a favor de las manecillas del reloj
    else if (dir < 0)
    {
        GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,0x00);//se apaga pin
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,GPIO_PIN_6);//se enciende pin
    }
    //el motor se detiene
    else
    {
        GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5,0x00);//se apaga pin
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6,0x00);//se apaga pin
    }
}


int main(void)
{


    CONFIG();//se cargan todas las configuraciones

    pwm_word = ((SysCtlClockGet()/1)/PWM_FREC)-1;//factor por el que se tiene que multiplicar el valor que se envia al PWM
    while(1)
    {
        if(update == 1)
        {
            //-------------------------------------SE LIMPIA BANDERA DE ADC PARA INICIAR A LEER EL VALOR OBTENIDO--------------------------
           ADCIntClear(ADC0_BASE, 3);
           ADCProcessorTrigger(ADC0_BASE, 3);//SE ACTIVA EL TRIGUER PARA REALIZAR LECTURA
           while(!ADCIntStatus(ADC0_BASE, 3, false));//SE ESPERA QUE EL STATUAS DEL ADC SEA FALSE
           ADCSequenceDataGet(ADC0_BASE, 3, &COUNT);// SE LEE ADC

           ref = (float)(COUNT*2*3.1416/4095);
           ref_filtrado = lambda * ref_filtrado + (1-lambda) * ref;//fitro pasa bajas IIR
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

           //out = control_pid (out.outp, out.Mep, out.MEp, posicion, ref,5,0.000,0.001,0);
           out = control_pid (out.outp, out.Mep, out.MEp, pos_tot, ref,10,0.002,0.0001,0);//PID para control solo de posicion
           //out = control_pid (out.outp, out.Mep, out.MEp, pos_tot, ref,5,0.000001,0.0001,0);//PID para control en cascada
           giro=(float)(abs(out.outp)*4095)/(2*3.1416);

           ref2 = (float)(ref_filtrado*52.35988/4095);//(float)(giro*52.35988/4095);
           velocidad = (float)(QEIVelocityGet(QEI0_BASE)*100*60/979.2)*0.10472;
           out = control_pid (out.outv, out.Mev, out.MEv, velocidad, ref2, 1.5, 0.002, 0.00001, 1);//PID para control solo de posicion
           //out = control_pid (out.outv, out.Mev, out.MEv, velocidad, ref2, 1.25, 0.0, 0.00001, 1);//1.5//PID para control en cascada
           //******************************************FEEDFOWARD***********************************************************
          feedfoward = (ref2 - 2.6135)*78.125;
          update = 0;
        }

        //pulso = giro + 30;
        pulso = (float)(30 + (abs(out.outv)*4095)/52.35998);
       // pulso = (float)(30 + (abs(out.outv)*4095)/52.35998)+feedfoward;

        direccion(2);//se selecciona la direccion en la que el motor gira
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pulso*pwm_word/4095);//se envia la salida del controlador convertido a señal PWM al pwm.
        salida_posicion=(180/3.1416)*pos_tot;
        ref_grados=(180/3.1416)*ref;
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
        data[8] = ((uint32_t)ref2 >> 24) & 0xff;  //se extrae los bits 24-31 de la señal de referencia de velocidad y se almacenan en la novena
                                                   //posicion del array data

        data[9] = ((uint32_t)ref2 >> 16) & 0xff;  //se extrae los bits 16-23 de la señal de referencia de velocidad y se almacenan en la decima
                                                   //posicion del array data

        data[10] = ((uint32_t)ref2 >>  8) & 0xff;  //se extrae los bits 8-15 de la señal de referencia de velocidad y se almacenan en la onceaba
                                                    //posicion del array data

        data[11] = (uint32_t)ref2 & 0xff; //se extrae los bits 0-7 de la señal de referencia de velocidad y se almacenan en la doceaba
                                           //posicion del array data

        data[12]=((uint32_t)velocidad >> 24) & 0xff; //se extrae los bits 24-31 de la señal de velocidad del encoder y se almacenan en la treceaba
                                                     //posicion del array data
        data[13]=((uint32_t)velocidad >> 16) & 0xff; //se extrae los bits 16-23 de la señal de velocidad del encoder y se almacenan en la catorceaba
                                                     //posicion del array data
        data[14]=((uint32_t)velocidad >> 8) & 0xff; //se extrae los bits 8-15 de la señal de velocidad del encoder y se almacenan en la quinceaba
                                                    //posicion del array data
        data[15]=(uint32_t)velocidad; //se extrae los bits 0-8 de la señal de velocidad del encoder y se almacenan en la diez y seisaba
                                      //posicion del array data

        if(serial==0){
            UARTCharPut(UART0_BASE,'6');
        }
//*******************************SI SE RECIVE EL VALOR DE n=1 DEL UART SE INICIA A ENVIAR TODOS LOS DATOS EN ORDEN*******************************************
        if (n == 1)
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

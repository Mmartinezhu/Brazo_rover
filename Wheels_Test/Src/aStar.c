/**
 ******************************************************************************
 * @file           : mpu.c
 * @author         : Jhony Aristizabal, Sebastian Gaviria
 * @brief          : Programa que implementa un PID basado en datos de velocidad
 * 					 y rotación del robot para el control de movimiento del mismo
 * 					 con el objetivo de lograr el recorrido automático de una
 * 					 trayectoria definida.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include <stdint.h>
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "stm32f4xx.h"
#include "gpio_driver_hal.h"
#include "timer_driver_hal.h"
#include "usart_driver_hal.h"
#include "systick_driver_hal.h"
#include "pll_driver_hal.h"
#include "pwm_driver_hal.h"





//------------------------------------------VARIABLES GLOBALES------------------------------------------

char buffer[64] = {0}; // Buffer temporal para almacenar la fila sin corchetes
char buffer2[64] = {0}; // Buffer temporal para almacenar la fila sin corchetes

//------------------------------------------------------------------------------------




// Se define un pin para el Blinky
GPIO_Handler_t stateLed = {0}; // PinA5
GPIO_Handler_t stateLedBoard = {0}; // PinC5
// Handler del Timer
Timer_Handler_t Tim_Blinky = {0};
// Handler para el PLL
PLL_Handler_t pllHandler = {0};
//USART
GPIO_Handler_t handlerPinTX		= {0};
GPIO_Handler_t handlerPinRX		= {0};
USART_Handler_t usart2Comm		= {0};






char bufferMsg[128] = {0};
char bufferReceiver[64] = {0};
uint8_t rxData = 0;
uint8_t counterReception = 0;
uint8_t stringComplete = 0;

// Variables para los comandos
char cmd[64] = {0};
float firstParameter = 0;
float secondParameter = 0;
char lastString[64] = {0};


PWM_Handler_t pwmHandler = {0};
GPIO_Handler_t pinPWM = {0};
uint8_t defaultSpeed = 0;
uint8_t flagStop = 0;



// Contadores
uint16_t counterBlinky = 0;
uint32_t counterMicros = 0;
uint16_t counterPeriodTest = 0;
uint16_t LimitBlinky = 50E3;

uint8_t flagTimer = 0;

uint8_t periodBlinky = 0;



char bufferData[64] = "Algoritmo...";

// Funciones privadas
void initSystem(void);
void parseCommands(char  *ptrbufferReception);

void forwardMove(float percDutyL);
void backwardMove(float percDutyL);
void turnOff(void);
void turnOn(void);

void manageCounters(void);




/* ===== Función principal del programa ===== */
int main(void){

	SCB->CPACR |= 0xf<<20;

	initSystem();




	// Se configura inicialmente el MPU
	config_SysTick_ms(HSI_CLOCK_CONFIGURED);
	delay_ms(1);

	sprintf(bufferMsg, "Algoritmo A* \n");
	sprintf(bufferMsg, "Elaborado por Jhony Aristizábal \n");
	usart_WriteMsg(&usart2Comm, bufferMsg);
	sprintf(bufferMsg, "Escribe \"help @\" ");
	usart_WriteMsg(&usart2Comm, bufferMsg);



	/* Loop forever */
	while (1) {





		// Se revisa cual fue el dato recibido por la comunicacion serial
		if(rxData != '\0'){
			bufferReceiver[counterReception] = rxData;
			counterReception++;

			// Se verifica si el último dato es el caracter de finalizacion de un string
			if (rxData == '@'){
				// Se modifica el estado de una variable de control
				stringComplete = 1;

				// Configuramos las variables para guardar el string y esperar uno nuevo
				bufferReceiver[counterReception] = '\0';//el caracter nulo \0 siempre esta al final de un string
				counterReception = 0;
			}

			// Limpiamos la variable que almacena los datos recibidos por el com. serial
			rxData = '\0';
		}


		else{
			// Aun no se han recibido caractéres a traves de la comunicacion serial
			stringComplete = 0;
		}

		if(stringComplete){
			parseCommands(bufferReceiver);
			stringComplete = 0;

		}



	} // Fin del while

	return 0;


}	// Fin del main



// Función para configurar los periféricos iniciales del sistemas
void initSystem(void){

	// 1. ===== PUERTOS Y PINES =====
	/* Configurando el pin para el Blinky */
	stateLed.pGPIOx								= GPIOA;
	stateLed.pinConfig.GPIO_PinNumber			= PIN_5;	// PinA5
	stateLed.pinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	stateLed.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	stateLed.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	stateLed.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	gpio_Config(&stateLed);

	/* Configurando el pin para el Blinky */
	stateLedBoard.pGPIOx							= GPIOC;
	stateLedBoard.pinConfig.GPIO_PinNumber			= PIN_5;	// PinA5
	stateLedBoard.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	stateLedBoard.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	stateLedBoard.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	stateLedBoard.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	gpio_Config(&stateLedBoard);


	// 2. ===== TIMERS =====
	/* Configurando el Timer del Blinky */
	Tim_Blinky.pTIMx								= TIM2;
	Tim_Blinky.TIMx_Config.TIMx_Prescaler			= 100;	// Genera incrementos de 10 us. El micro está a 100MHz
	Tim_Blinky.TIMx_Config.TIMx_Period				= 10;		// De la mano con el pre-scaler, determina cuando se dispara una interrupción (500ms)
	Tim_Blinky.TIMx_Config.TIMx_mode				= TIMER_UP_COUNTER;	// El Timer cuante ascendente
	Tim_Blinky.TIMx_Config.TIMx_InterruptEnable		= TIMER_INT_ENABLE;	// Se activa la interrupción
	timer_Config(&Tim_Blinky);
	timer_SetState(&Tim_Blinky, TIMER_ON);

	/* ==================================== PLL =============================================*/

	pllHandler.clkSpeed = FREQUENCY_100MHz;
	//Calibramos el clock
	RCC->CR &= ~RCC_CR_HSITRIM;			//Limpiamos el registro
	RCC->CR |= (12<<RCC_CR_HSITRIM_Pos); // Numero para calibrar POR DEFECTO ESTABA EN 15!!!!!
	configPLL(&pllHandler);

	/* ==================================== Configurando los USART =============================================*/
	/*USART 1 -> Comunicación serial a través de la antena */
	handlerPinTX.pGPIOx										= GPIOA;
	handlerPinTX.pinConfig.GPIO_PinNumber					= PIN_2;
	handlerPinTX.pinConfig.GPIO_PinMode						= GPIO_MODE_ALTFN;
	handlerPinTX.pinConfig.GPIO_PinAltFunMode				= AF7;
	gpio_Config(&handlerPinTX);



	// 3. ============================== PWM ==================================================

	/* - Usamos el  para el Canal del Timer del PWM */
	pinPWM.pGPIOx								= GPIOA;
	pinPWM.pinConfig.GPIO_PinNumber				= PIN_7;
	pinPWM.pinConfig.GPIO_PinMode				= GPIO_MODE_ALTFN;
	pinPWM.pinConfig.GPIO_PinAltFunMode			= AF2;
	pinPWM.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	pinPWM.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_FAST;

	/* Cargamos la configuración */
	gpio_Config(&pinPWM);


	/* Configurando el PWM para el motor DERECHO */
	pwmHandler.ptrTIMx					= TIM3; // Timer5 usado para el PWM
	pwmHandler.config.channel			= PWM_CHANNEL_2;
	pwmHandler.config.prescaler			= 50E2; 	// 0.05 ms
	pwmHandler.config.periodo			= 1000;		// 50 ms -> Frec. de 20 Hz
	pwmHandler.config.percDuty			= 0;
	pwm_Config(&pwmHandler);

	// ====================================================================================================

	handlerPinRX.pGPIOx										= GPIOA;
	handlerPinRX.pinConfig.GPIO_PinNumber					= PIN_3;
	handlerPinRX.pinConfig.GPIO_PinMode						= GPIO_MODE_ALTFN;
	handlerPinRX.pinConfig.GPIO_PinAltFunMode				= AF7;
	gpio_Config(&handlerPinRX);

	usart2Comm.ptrUSARTx									= USART2;
	usart2Comm.USART_Config.baudrate						= USART_BAUDRATE_115200;
	usart2Comm.USART_Config.datasize						= USART_DATASIZE_8BIT;
	usart2Comm.USART_Config.parity							= USART_PARITY_NONE;
	usart2Comm.USART_Config.stopbits						= USART_STOPBIT_1;
	usart2Comm.USART_Config.mode							= USART_MODE_RXTX;
	usart2Comm.USART_Config.enableIntRX						= USART_RX_INTERRUPT_ENABLE;
	usart2Comm.USART_Config.enableIntTX						= USART_TX_INTERRUPT_DISABLE;
	usart_Config(&usart2Comm);



} // Fin initSystem()





//------------------------------------------------------ALGORITMO PRINCIPAL------------------------------------------------------



/* Función para los comandos */
void parseCommands(char  *ptrbufferReception){

	sscanf(ptrbufferReception,"%s %f %f %s",cmd,&firstParameter,&secondParameter,lastString);
	//Comando para solicitar ayuda
    // Comando para solicitar ayuda
	// Comando para solicitar ayuda
	if (strcmp(cmd, "help") == 0) {
		usart_WriteMsg(&usart2Comm, "Help Menu CMDS: \n");
		usart_WriteMsg(&usart2Comm, "1) Dir 0:forw / 1:back ; dutty(\%) \" Dir # # @\" \n");
		usart_WriteMsg(&usart2Comm, "1) Cuentas dutty(\%) \" Cuentas (#) @\" \n");

		usart_WriteMsg(&usart2Comm, "2) Spd \%leftM 		; \%rightM \" Spd # # @\" \n");
		usart_WriteMsg(&usart2Comm, "3) Rot 0:left 1:right  ; #turns  \" Rot # # @\" \n");
		usart_WriteMsg(&usart2Comm, "4) TestEncoders percDuttyCycle:left \" TestEncoders # @\" \n");

		usart_WriteMsg(&usart2Comm, "5) Test 0:left / 1:right; dutty   \" Test # # @\" \n");
		usart_WriteMsg(&usart2Comm, "1) Ajuste Cuentas (#) deltaDuty (float) @ \n");


		usart_WriteMsg(&usart2Comm, "6) Stop \" Stop @\" \n");
		usart_WriteMsg(&usart2Comm, "7) Resume \" Resume @\" \n");
	}




	else if (strcmp(cmd, "Dir") == 0) {

		// firstParameter indica la direccion, secondParameter es el dutyCycle
		if (firstParameter == 0 && secondParameter >= 0){
			if (defaultSpeed == 0){
				forwardMove(secondParameter);
			}
			else{
				forwardMove(10);
			}
			usart_WriteMsg(&usart2Comm, "Moviéndose hacia adelante \n");
		}
		else if (firstParameter == 1 && secondParameter >= 0){
			if (defaultSpeed == 0){
				backwardMove(secondParameter);
			}
			else{
				backwardMove(10);
			}
			usart_WriteMsg(&usart2Comm, "Moviéndose hacia atrás \n");
		}
		defaultSpeed = 0;
	}

	else if(strcmp(cmd, "Dutty") == 0) {
			if (firstParameter > 0) {

				defaultSpeed = firstParameter;
				updateDutyCycle(&pwmHandler,(uint16_t)firstParameter);

				sprintf(bufferData,"Velocidad actualizada: %.2f, \n",firstParameter);
				usart_WriteMsg(&usart2Comm, bufferData);
			}
			else{
				usart_WriteMsg(&usart2Comm, "Porcentaje debe ser positivo.\n Ingresa \"help @\" para ver la lista de comandos.\n");
			}
	}


	else if(strcmp(cmd, "Freq") == 0) {
			if (firstParameter > 0) {

				updateFrequency(&pwmHandler, firstParameter);



				sprintf(bufferMsg,"Frecuencia actualizado: %.2f \n",firstParameter);
				usart_WriteMsg(&usart2Comm, bufferMsg);
			}
			else{
				usart_WriteMsg(&usart2Comm, "La Frecuencia debe ser positiva.\n Ingresa \"help @\" para ver la lista de comandos.\n");
			}
	}

	else if(strcmp(cmd, "Period") == 0) {
			if (firstParameter > 0) {

				updatePeriod(&pwmHandler, firstParameter);


				sprintf(bufferMsg,"Periodo actualizado: %.2f \n",firstParameter);
				usart_WriteMsg(&usart2Comm, bufferMsg);
			}
			else{
				usart_WriteMsg(&usart2Comm, "Periodo debe ser positivo.\n Ingresa \"help @\" para ver la lista de comandos.\n");
			}
	}


/* ==================== PID ==================== */




	else if (strcmp(cmd, "Stop") == 0) {
		flagStop = 1;
		turnOff();
		usart_WriteMsg(&usart2Comm, "Detiene del sistema \n");
	}

	else if (strcmp(cmd, "Resume") == 0) {
		turnOn();
		usart_WriteMsg(&usart2Comm, "Reanuda del sistema \n");
	}

	else{
		usart_WriteMsg(&usart2Comm, "\nComando erroneo.\n Ingresa \"help @\" para ver la lista de comandos.\n");
	}


}


void forwardMove(float dutyPercentage){

	stopPwmSignal(&pwmHandler);

	pwmHandler.config.polarity	= PWM_POLARITY_DIRECT;
	pwm_Config(&pwmHandler);
	selectPolarity(&pwmHandler);

	updateDutyCycle(&pwmHandler,dutyPercentage);

	// Encendemos el PWM para mover el motor derecho
	startPwmSignal(&pwmHandler);

}

void backwardMove(float percDutyR){

	stopPwmSignal(&pwmHandler);

	pwmHandler.config.polarity	= PWM_POLARITY_INVERSE;
	pwm_Config(&pwmHandler);
	selectPolarity(&pwmHandler);

	updateDutyCycle(&pwmHandler,percDutyR);

	// Encendemos el PWM para mover el motor derecho
	startPwmSignal(&pwmHandler);

}

//----------------------------------------------------------------

/* Función para manejar los diferentes conteos de tiempo (Periodos) */
void manageCounters(void){
	if (counterBlinky > LimitBlinky){//cada 500 ms revisamos los contadores

		gpio_TogglePin(&stateLed);//cambiamos el estado del led
		gpio_TogglePin(&stateLedBoard);

		if (gpio_ReadPin(&stateLed)) {//si el pin esta en alto contamos
			if (periodBlinky > 10) {//contamos cada ciclo de encendido-apagado de led
				periodBlinky = 0;//reiniciamos la variable
			}
			periodBlinky++;//aumentamos el contador del periodo
		}


		flagTimer ^= 1;
		counterBlinky = 0;
		counterPeriodTest++;

	}

}


void turnOff(void){

	// Apaga el puente H para los motores
	gpio_WritePin(&pinPWM, RESET);

	// Apaga los PWM
	stopPwmSignal(&pwmHandler);

}

void turnOn(void){

	// Enciente el puente H de los motores
	gpio_WritePin(&pinPWM, RESET);

	// Enciente los PWM
	startPwmSignal(&pwmHandler);

}


/* Callback de Timer 2 */
void Timer2_Callback(void){
	counterBlinky++;
	counterMicros++;
	// Función que maneja todos los conteos de tiempo basados en interrupciones del Timer2
	manageCounters();
}


/* Interrupciones por recepcion a traves de transmision serial */
void usart2_RxCallback(void){
	rxData = usart2_getRxData();

}




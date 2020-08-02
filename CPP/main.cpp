/*
 * cppmain.h
 *
 *  Created on: 2020. 8. 2.
 *      Author: junwoo
 */

//essential
#include "main.h"
#include "usart.h"
#include "i2c.h"
#include "tim.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "timers.h"

//additional
#include "stdio.h"
#include "stdlib.h"

//selfmade
#include "MPU9250.h"
#include "sbus.h"
#include "tm_stm32_gps.h"
#include "IST8310.h"
#include "bme280.h"

//
#include "PeripheralInterface/SensorAccel.hpp"

using namespace FC;

#define USE_IST8310
#define USE_GPS
#define USE_MPU9250
#define USE_BME280
#define USE_SBUS

//we can use printf
int _write(int file, unsigned char* p, int len) // for debug through uart3
{
	HAL_UART_Transmit(&huart3, p, len, 10);
	return len;
}


#ifdef USE_GPS
void GPS_main(){
	taskENTER_CRITICAL();
    TM_GPS_Init(&huart8);
	taskEXIT_CRITICAL();

	while(1){
		GPS_calHz();
		printf("%u\r\n", gps.hz);
		osDelay(1000);
	}
}
#endif

#ifdef USE_IST8310
void IST8310_main(){
	taskENTER_CRITICAL();
	IST8310(&hi2c2);
	taskEXIT_CRITICAL();

	while(1){

		IST8310_updataIT();

		osDelay(10);
	}
}
void IST8310_timer(TimerHandle_t pxTimer){
	IST8310_updataIT();
}
#endif


#ifdef USE_BME280
void BME280_main(){
	taskENTER_CRITICAL();
	BME280(&hi2c2);

	/*
	 * recommended mode : gaming
	 * Sensor mode : normal mode, standby = 0.5ms
	 * oversampling : pressureX4, temperatureX1, humidityX0
	 * IIR filter coefficient : 16
	 * RMS Noise : 0.3Pa/2.5cm
	 * Data output rate : 83hz
	 * Filter bandwidth : 1.75 Hz
	 * response time : 0.3s
	 */
	BME280_init(P_OSR_04, H_OSR_00, T_OSR_01, normal, BW0_021ODR,t_00_5ms);
	taskEXIT_CRITICAL();

	while(1){
		BME280_updateIT();
		osDelay(20);
	}
}
void BME280_timer(TimerHandle_t pxTimer){
	BME280_updateIT();
}
#endif

#ifdef USE_MPU9250
void MPU9250_main(void* param){
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 5;

	taskENTER_CRITICAL();
	MPU9250(&hi2c1);
	taskEXIT_CRITICAL();

	xLastWakeTime = xTaskGetTickCount();
	while(1){
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		MPU9250_updateDMA();
	}
}

void MPU9250_timer(TimerHandle_t pxTimer){
	MPU9250_updateDMA();
}
#endif

void debug_main(void* param){

	while(1){
		osDelay(1000);
	}
}

#define MPU9250_UPDATE_HZ 200
#define BME280_UPDATE_HZ 50
#define IST8310_UPDATE_HZ 100

void cppMain(){
    setvbuf(stdout, NULL, _IONBF, 0);

    /* micro second timer start */
	HAL_TIM_Base_Start_IT(&htim2);

	TimerHandle_t timerResult;
#ifdef USE_MPU9250
	MPU9250(&hi2c1);
	timerResult = xTimerCreate("MPU9250_timer",
							   pdMS_TO_TICKS(1000/MPU9250_UPDATE_HZ),
							   pdTRUE,
							   NULL,
							   MPU9250_timer);
	if(timerResult == NULL) {
		while(1);
	}
#endif

#ifdef USE_BME280
	BME280(&hi2c2);
	BME280_init(P_OSR_04, H_OSR_00, T_OSR_01, normal, BW0_021ODR,t_00_5ms);
	timerResult = xTimerCreate("BME280_timer",
							   pdMS_TO_TICKS(1000/BME280_UPDATE_HZ),
							   pdTRUE,
							   NULL,
							   BME280_timer);
	if(timerResult == NULL) {
		while(1);
	}
#endif

#ifdef USE_IST8310
	IST8310(&hi2c2);
	timerResult = xTimerCreate("IST8310_timer",
							   pdMS_TO_TICKS(1000/IST8310_UPDATE_HZ),
							   pdTRUE,
							   NULL,
							   IST8310_timer);
    if(timerResult == NULL) {
		while(1);
	}
#endif


#ifdef USE_GPS
    TM_GPS_Init(&huart8);
#endif

#ifdef USE_SBUS
	SBUS_init(&huart7);
#endif

    printf("boot complete\r\n");


//    cMain();
	xTaskCreate(debug_main,
				"debug_main",
				configMINIMAL_STACK_SIZE,
				NULL,
				5,
				NULL);


//#ifdef USE_BME280
//	xTaskCreate(BME280_main,
//				"BME280_main",
//				configMINIMAL_STACK_SIZE,
//				NULL,
//				4,
//				NULL);
//#endif
//
//#ifdef USE_IST8310
//	xTaskCreate(IST8310_main,
//				"IST8310_main",
//				configMINIMAL_STACK_SIZE,
//				NULL,
//				4,
//				NULL);
//#endif
//
//#ifdef USE_GPS
//	xTaskCreate(GPS_main,
//				"GPS_main",
//				configMINIMAL_STACK_SIZE,
//				NULL,
//				4,
//				NULL);
//#endif
//
//#ifdef USE_MPU9250
//    xTaskCreate(MPU9250_main,
//    		    "MPU9250_main",
//				configMINIMAL_STACK_SIZE,
//				NULL,
//				4,
//				NULL);
//#endif


}

//callback
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
#ifdef USE_MPU9250
	if(hi2c->Instance == mpu9250.hi2c->Instance){
		MPU9250_i2cRxCpltCallback();
		sensorAccel.setAccel(mpu9250.accel[0], mpu9250.accel[1], mpu9250.accel[2]);
	}
#endif

#ifdef USE_IST8310
	if(hi2c->Instance == ist8310.hi2c->Instance){
		IST8310_i2cRxCpltCallback();
	}
#endif

#ifdef USE_BME280
	if(hi2c->Instance == bme280.hi2c->Instance){
		BME280_i2cRxCpltCallback();
	}
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef USE_SBUS
	if(huart->Instance == sbus.huart->Instance){
		SBUS_uartRxCpltCallback();
	}
#endif
#ifdef USE_GPS
	if(huart->Instance == UART5){
		TM_GPS_Update();
	}
#endif

	if(huart->Instance == USART2){
		// telemetry
	}
}

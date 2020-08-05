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
#include "cstdio"
#include "cstdlib"

//selfmade
#include "Driver/MPU9250.h"
#include "Driver/sbus.h"
#include "Driver/tm_stm32_gps.h"
#include "Driver/IST8310.h"
#include "Driver/bme280.h"
#include "Driver/Lidar1D.h"
//
#include "PeripheralInterface/SensorAccel.hpp"
#include "PeripheralInterface/SensorGyro.hpp"
#include "PeripheralInterface/SensorMag.hpp"
#include "PeripheralInterface/SensorGPS.hpp"
#include "PeripheralInterface/SensorBaro.hpp"
#include "PeripheralInterface/RC.hpp"

#include "Module/ModuleAHRS.hpp"
#include "Module/ModuleCommander.h"


using namespace FC;

#define USE_MPU9250
#define USE_IST8310
#define USE_GPS
#define USE_BME280
#define USE_SBUS
#define USE_AHRS


SensorMag sensorMag;
SensorAccel sensorAccel;
SensorGyro sensorGyro;
SensorBaro sensorBaro;
SensorGPS sensorGPS;
RC rc;

ModuleAHRS moduleAHRS;

//we can use printf
int _write(int file, unsigned char* p, int len) // for debug through uart3
{
	HAL_UART_Transmit(&huart3, p, len, 10);
	return len;
}


#ifdef USE_GPS
//void GPS_main(){
//	while(1){
//		GPS_calHz();
//		printf("%u\r\n", gps.hz);
//		osDelay(1000);
//	}
//}
#endif

#ifdef USE_IST8310
void IST8310_timer(TimerHandle_t pxTimer){
	IST8310_updataIT();
}
#endif


#ifdef USE_BME280
void BME280_timer(TimerHandle_t pxTimer){
	BME280_readIT();
}
#endif

#ifdef USE_MPU9250
void MPU9250_timer(TimerHandle_t pxTimer){
	MPU9250_updateDMA();
}
#endif

#ifdef USE_AHRS
void moduleAHRS_timer(TimerHandle_t pxTimer){
	moduleAHRS.main();
}
#endif

void moduleCommanderMain(void* param){
	while(1){
		moduleCommander.main();
	}
}

uint8_t test = 0;
uint8_t testUsr = 0;
uint8_t check = 0;
/*
 *  Switch
 *  Click : High
 *  GPIO_PinState result = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7);
 */
/*
 * 	buzzer
 *  if(HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) != HAL_OK){
 *		error
 *	}
 */
/*
 *  LED signal
 *  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
 */

void debug_main(void* param){
    char buf[256];
    uint32_t bw;
    int len = 0;
    retSD=f_mount(&SDFatFS ,&SDPath[0],1);
	if(retSD==FR_OK){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	}
	else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

	while(1){
		/* open */
		f_open(&SDFile,"log.txt", FA_OPEN_APPEND | FA_WRITE );
		if(retSD==FR_OK){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		}
		else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

		/* data read */
//		struct Attitude attitude;
//		struct NedAccel nedAccel;
//		struct GPS gps;
//		struct Barometer baro;
////		if(msgBus.getAttitude(&attitude)){
//		msgBus.getAttitude(&attitude);
//			len = sprintf(buf, "att %u %d %d %d\n", attitude.timestamp, (int)(attitude.roll*1000000),
//													(int)(attitude.pitch*1000000),
//													(int)(attitude.yaw*1000000));
//			f_write(&SDFile, buf, len, (UINT*)&bw);
////		}
//		if(msgBus.getAttitude(&attitude)){
//					len = sprintf(buf, "att %u %d %d %d\r\n", attitude.timestamp, (int)(attitude.roll*1000000),
//															(int)(attitude.pitch*1000000),
//															(int)(attitude.yaw*1000000));
//					f_write(&SDFile, buf, len, (UINT*)&bw);
//		}
//		msgBus.getNedAccel(&nedAccel);
//		msgBus.getGPS(&gps);
//		msgBus.getBarometer(&baro);

		len = sprintf(buf,"sexxxxx\n");
		f_write(&SDFile, buf, len, (UINT*)&bw);

		if(retSD==FR_OK){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		}
		else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

		f_close(&SDFile);
		if(retSD==FR_OK){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		}
		else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		osDelay(5);
	}


}

#define MPU9250_UPDATE_HZ 200
#define BME280_UPDATE_HZ 50
#define IST8310_UPDATE_HZ 100

#define AHRS_UPDATE_HZ 200

void cppMain(){
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("test\r\n");
    /* micro second timer start */
	HAL_TIM_Base_Start_IT(&htim2);

//	HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_1);
//	HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_2);
	Lidar1D_init(&htim9, TIM_CHANNEL_1, TIM_CHANNEL_2);
	Lidar1D_run();
#ifdef USE_MPU9250
	MPU9250(&hi2c1);
	TimerHandle_t thMPU9250 = xTimerCreate("MPU9250_timer",
										   pdMS_TO_TICKS(1000/MPU9250_UPDATE_HZ),
										   pdTRUE,
										   NULL,
										   MPU9250_timer);
	if(thMPU9250 == NULL) {
		/* timer heap error */
		while(1){

		}
	}
	/* timer start */
	if( xTimerStart( thMPU9250, 0 ) != pdPASS )
	{
		// The timer could not be set into the Active state.
	}
#endif

#ifdef USE_BME280
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
	BME280_init(&hi2c2, P_OSR_04, H_OSR_00, T_OSR_01, normal, BW0_021ODR,t_00_5ms);
	TimerHandle_t thBME280 = xTimerCreate("BME280_timer",
										  pdMS_TO_TICKS(1000/BME280_UPDATE_HZ),
										  pdTRUE,
										  NULL,
										  BME280_timer);
	if(thBME280 == NULL) {
		while(1){
			/* timer heap error */
		}
	}
	/* timer start */
	if( xTimerStart( thBME280, 0 ) != pdPASS )
	{
		// The timer could not be set into the Active state.
	}
#endif

#ifdef USE_IST8310
	IST8310(&hi2c2);
	TimerHandle_t thIST8310 = xTimerCreate("IST8310_timer",
										   pdMS_TO_TICKS(1000/IST8310_UPDATE_HZ),
										   pdTRUE,
										   NULL,
										   IST8310_timer);
	if(thIST8310 == NULL) {
		while(1){
			/* timer heap error */
		}
	}
	/* timer start */
	if( xTimerStart( thIST8310, 0 ) != pdPASS )
	{
		// The timer could not be set into the Active state.
	}
#endif

#ifdef USE_GPS
	/*
	 *  GPS using DMA circular mode
	 */
    TM_GPS_Init(&huart8);
#endif

#ifdef USE_SBUS
	SBUS_init(&huart7);
#endif

#ifdef USE_AHRS
	TimerHandle_t thModuleAHRS = xTimerCreate("moduleAHRS_timer",
							   	   	   	   	  pdMS_TO_TICKS(1000/AHRS_UPDATE_HZ),
											  pdTRUE,
											  NULL,
											  moduleAHRS_timer);
	if(thModuleAHRS == NULL) {
		/* timer heap error */
		while(1){

		}
	}
	/* timer start */
	if( xTimerStart( thModuleAHRS, 0 ) != pdPASS )
	{
		// The timer could not be set into the Active state.
	}
#endif

    std::printf("boot complete\r\n");

	xTaskCreate(debug_main,
				"debug_main",
				1024,
				NULL,
				5,
				NULL);
//	xTaskCreate(moduleCommanderMain,
//				"moduleCommanderMain",
//				configMINIMAL_STACK_SIZE,
//				NULL,
//				5,
//				NULL);

}

//callback
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
#ifdef USE_MPU9250
	if(hi2c->Instance == mpu9250.hi2c->Instance){
		MPU9250_i2cRxCpltCallback();
		sensorAccel.setAccel(mpu9250.accel[0], mpu9250.accel[1], mpu9250.accel[2]);
		sensorGyro.setGyro(mpu9250.gyro[0], mpu9250.gyro[1], mpu9250.gyro[2]);
		sensorMag.setMag(mpu9250.mag[0], mpu9250.mag[1], mpu9250.mag[2]);
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
		sensorBaro.setBaro(bme280.P, bme280.T);
	}
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef USE_SBUS
	if(huart->Instance == sbus.huart->Instance){
		if(SBUS_uartRxCpltCallback() == SBUS_Result_NewData){
			rc.setRC(SBUS_getChannel(2),	/* roll */
					 SBUS_getChannel(3),	/* pitch */
					 SBUS_getChannel(3), 	/* yaw */
					 SBUS_getChannel(1),	/* throttle */
					 SBUS_getChannel(11));
		}
	}
#endif

#ifdef USE_GPS
	if(huart->Instance == UART8){
		if(TM_GPS_Update() == TM_GPS_Result_NewData){
			sensorGPS.setGPS(gpsUart.gpsData.Latitude, gpsUart.gpsData.Longitude, gpsUart.gpsData.Altitude,
							 TM_GPS_ConvertSpeed(gpsUart.gpsData.Speed, TM_GPS_Speed_MeterPerSecond), gpsUart.gpsData.Direction, gpsUart.gpsData.HDOP, gpsUart.gpsData.VDOP,
							 gpsUart.gpsData.Satellites, gpsUart.gpsData.FixMode, 0/* UTC in microsecond */);
		}
	}
#endif

	if(huart->Instance == USART2){
		// telemetry
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	Lidar1D_CaptureCallback(htim);
}

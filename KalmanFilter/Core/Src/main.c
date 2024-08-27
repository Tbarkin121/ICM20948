/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>

#define INV_MSG_ENABLE
#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/DataConverter.h"
#include "Invn/Devices/DeviceIcm20948.h"
#include "Invn/DynamicProtocol/DynProtocol.h"
#include "Invn/DynamicProtocol/DynProtocolTransportUart.h"

#include "Invn/Devices/Drivers/Icm20948/Icm20948Defs.h"
#include "Invn/Devices/Drivers/Icm20948/Icm20948Setup.h"

#include "idd_io_hal.h"
#include "delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAIN_UART_ID UART1 // Through FTDI cable
#define LOG_UART_ID  UART2 // Through ST-Link
#define DELAY_TIMER  TIM4
#define TIMEBASE_TIMER TIM2

#define USE_IDDWRAPPER 0   /* This allows to control the sensors from sensor-cli host application and send sensor events to it */
#define ODR_NONE       0   /* Asynchronous sensors don't need to have a configured ODR */


/* Define msg level */
#define MSG_LEVEL INV_MSG_LEVEL_MAX

#define USE_RAW_ACC    0
#define USE_RAW_GYR    0
#define USE_GRV        0
#define USE_CAL_ACC    0
#define USE_CAL_GYR    0
#define USE_CAL_MAG    0
#define USE_UCAL_GYR   0
#define USE_UCAL_MAG   0
#define USE_RV         0    /* requires COMPASS*/
#define USE_GEORV      0    /* requires COMPASS*/
#define USE_ORI        0    /* requires COMPASS*/
#define USE_STEPC      0
#define USE_STEPD      0
#define USE_SMD        0
#define USE_BAC        0
#define USE_TILT       0
#define USE_PICKUP     0
#define USE_GRAVITY    0
#define USE_LINACC     0
#define USE_B2S        0

static const struct {
	uint8_t  type;
	uint32_t period_us;
} sensor_list[] = {
#if USE_RAW_ACC
	{ INV_SENSOR_TYPE_RAW_ACCELEROMETER, 10000 /* 20 Hz */ }, //Not exceeding 200 hz (ish)
#endif
#if USE_RAW_GYR
	{ INV_SENSOR_TYPE_RAW_GYROSCOPE,     10000 /* 20 Hz */ },
#endif
#if USE_CAL_ACC
	{ INV_SENSOR_TYPE_ACCELEROMETER, 50000 /* 20 Hz */ },
#endif
#if USE_CAL_GYR
	{ INV_SENSOR_TYPE_GYROSCOPE, 50000 /* 20 Hz */ },
#endif
#if USE_CAL_MAG
	{ INV_SENSOR_TYPE_MAGNETOMETER, 50000 /* 20 Hz */ },
#endif
#if USE_UCAL_GYR
	{ INV_SENSOR_TYPE_UNCAL_GYROSCOPE, 50000 /* 20 Hz */ },
#endif
#if USE_UCAL_MAG
	{ INV_SENSOR_TYPE_UNCAL_MAGNETOMETER, 50000 /* 20 Hz */ },
#endif
#if USE_GRV
	{ INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_RV
	{ INV_SENSOR_TYPE_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_GEORV
	{ INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_ORI
	{ INV_SENSOR_TYPE_ORIENTATION, 50000 /* 20 Hz */ },
#endif
#if USE_STEPC
	{ INV_SENSOR_TYPE_STEP_COUNTER, ODR_NONE },
#endif
#if USE_STEPD
	{ INV_SENSOR_TYPE_STEP_DETECTOR, ODR_NONE},
#endif
#if USE_SMD
	{ INV_SENSOR_TYPE_SMD, ODR_NONE},
#endif
#if USE_BAC
	{ INV_SENSOR_TYPE_BAC, ODR_NONE},
#endif
#if USE_TILT
	{ INV_SENSOR_TYPE_TILT_DETECTOR, ODR_NONE},
#endif
#if USE_PICKUP
	{ INV_SENSOR_TYPE_PICK_UP_GESTURE, ODR_NONE},
#endif
#if USE_GRA
	{ INV_SENSOR_TYPE_GRAVITY, 50000 /* 20 Hz */},
#endif
#if USE_LINACC
	{ INV_SENSOR_TYPE_LINEAR_ACCELERATION, 50000 /* 20 Hz */},
#endif
#if USE_B2S
	{ INV_SENSOR_TYPE_B2S, ODR_NONE},
#endif
};
const char * activityName(int act);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint16_t irq_from_device;
/* 
 * WHOAMI value for 20948
 */
static const uint8_t EXPECTED_WHOAMI[] = { 0xEA };
static const uint8_t dmp3_image[] = {
//	0
	 #include "icm20948_img.dmp3a.h"
};
/*
 * States for icm20948 device object
 */
static inv_device_icm20948_t device_icm20948;

/* 
 * Just a handy variable to keep the handle to device object
 */
static inv_device_t * device; 



/*
 * Last time at which 20948 IRQ was fired
 */
static volatile uint32_t last_irq_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ext_interrupt_cb(void * context, int int_num);
static void sensor_event_cb(const inv_sensor_event_t * event, void * arg);
void inv_icm20948_sleep_us(int us);
void inv_icm20948_sleep(int us);
uint64_t inv_icm20948_get_time_us(void);
uint64_t inv_icm20948_get_dataready_interrupt_time_us(void);
static void check_rc(int rc);
static void msg_printer(int level, const char * str, va_list ap);

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

#ifndef TO_MASK
//#define TO_MASK(a) (1U << (unsigned)(a))
#define TO_MASK(a) ((unsigned)(a))
#endif

void modify_register(struct inv_icm20948 * s, uint16_t reg, unsigned char mask, unsigned char value);
/*
 * A listener object will handle sensor events
 */
static const inv_sensor_listener_t sensor_listener = {
	sensor_event_cb, /* callback that will receive sensor events */
	0                /* some pointer passed to the callback */
};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  int rc = 0;
  unsigned i = 0;
  uint8_t whoami = 0xff;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Init(&huart2);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
  delay_init(DELAY_TIMER);
  timer_configure_timebase(&htim2, 1000000);
  HAL_TIM_Base_Start_IT(&htim2);

  /*
  * Setup message facility to see internal traces from IDD
  */
  INV_MSG_SETUP(MSG_LEVEL, msg_printer);

  /*
  * Welcome message
  */
  INV_MSG(INV_MSG_LEVEL_INFO, "###################################");
  INV_MSG(INV_MSG_LEVEL_INFO, "#          20948 example          #");
  INV_MSG(INV_MSG_LEVEL_INFO, "###################################");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  rc += inv_host_serif_open(idd_io_hal_get_serif_instance_spi());

  /*
	 * Create icm20948 Device 
	 * Pass to the driver:
	 * - reference to serial interface object,
	 * - reference to listener that will catch sensor events,
	 * - a static buffer for the driver to use as a temporary buffer
	 * - various driver option
	 */
	inv_device_icm20948_init(&device_icm20948, idd_io_hal_get_serif_instance_spi(),
			&sensor_listener, dmp3_image, sizeof(dmp3_image));

  /*
	 * Simply get generic device handle from icm20948 Device
	 */
	device = inv_device_icm20948_get_base(&device_icm20948);

	/*
	 * Just get the whoami
	 */
	rc = inv_device_whoami(device, &whoami);
	INV_MSG(INV_MSG_LEVEL_INFO, "ICM WHOAMI=%02x", whoami);
	check_rc(rc);

	/*
	 * Check if WHOAMI value corresponds to any value from EXPECTED_WHOAMI array
	 */
	for(i = 0; i < sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0]); ++i) {
		if(whoami == EXPECTED_WHOAMI[i])
			break;
	}

	if(i == sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0])) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Bad WHOAMI value. Got 0x%02x. Expected @EXPECTED_WHOAMI@.", whoami);
		check_rc(-1);
	}

	/*
	 * Configure and initialize the icm20948 device
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Setting-up ICM device");
	rc = inv_device_setup(device);
	check_rc(rc);

	uint64_t available_sensor_mask; /* To keep track of available sensors*/
	/*
		* Check sensor availibitlity
		* if rc value is 0, it means sensor is available,
		* if rc value is INV_ERROR or INV_ERROR_BAD_ARG, sensor is NA
		*/
	available_sensor_mask = 0;
	for(i = 0; i < sizeof(sensor_list)/sizeof(sensor_list[0]); ++i) {
		const int rc = inv_device_ping_sensor(device, sensor_list[i].type);
		INV_MSG(INV_MSG_LEVEL_INFO, "Ping %s %s", inv_sensor_2str(sensor_list[i].type), (rc == 0) ? "OK" : "KO");
		if(rc == 0) {
			available_sensor_mask |= (1ULL << sensor_list[i].type);
		}
	}

	/*
		* Start all available sensors from the sensor list
		*/
	for(i = 0; i < sizeof(sensor_list)/sizeof(sensor_list[0]); ++i) {
		if(available_sensor_mask & (1ULL << sensor_list[i].type)) {
			INV_MSG(INV_MSG_LEVEL_INFO, "Starting %s @ %u us", inv_sensor_2str(sensor_list[i].type), sensor_list[i].period_us);
			rc  = inv_device_set_sensor_period_us(device, sensor_list[i].type, sensor_list[i].period_us);
			check_rc(rc);
			rc += inv_device_start_sensor(device, sensor_list[i].type);
			check_rc(rc);
		}
	}

	/*
	* Poll device for data
	*/
	unsigned char test_data[6];
	unsigned char mask;
	unsigned char val;

	/*-------------------*/
	// Disable DMP INT
	mask = 0b10001111;
	val = 0;
	modify_register(&device_icm20948.icm20948_states, REG_INT_ENABLE, mask, val);
	/*-------------------*/
	// Enable Raw Data INT
	mask = 0b1;
	val = 0b1;
	modify_register(&device_icm20948.icm20948_states, REG_INT_ENABLE_1, mask, val);


	/*-------------------*/
	mask = 0b111111;
	val = 0;
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "REG_ACCEL_CONFIG");
	modify_register(&device_icm20948.icm20948_states, REG_ACCEL_CONFIG, mask, val);
	device_icm20948.icm20948_states.base_state.accel_fullscale = 0;

	mask = 0b11111;
	val = 0;
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "REG_ACCEL_CONFIG_2");
	modify_register(&device_icm20948.icm20948_states, REG_ACCEL_CONFIG_2, mask, val);

	mask = 0b1111;
	val = 0;
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "REG_ACCEL_SMPLRT_DIV_1");
	modify_register(&device_icm20948.icm20948_states, REG_ACCEL_SMPLRT_DIV_1, mask, val);

	mask = 0b11111111;
	val = 0;
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "REG_ACCEL_SMPLRT_DIV_2");
	modify_register(&device_icm20948.icm20948_states, REG_ACCEL_SMPLRT_DIV_2, mask, val);
	/*-------------------*/
	inv_icm20948_read_mems_reg(&device_icm20948.icm20948_states, REG_ACCEL_XOUT_H_SH, 6, test_data);
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Accel Meas (XH,XL,YH,YL,ZH,ZL) : %d, %d, %d, %d, %d, %d", test_data[0],test_data[1],test_data[2],test_data[3],test_data[4],test_data[5]);


	/*-------------------*/
	mask = 0b111111;
	val = 0;
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "REG_GYRO_CONFIG_1");
	modify_register(&device_icm20948.icm20948_states, REG_GYRO_CONFIG_1, mask, val);
	device_icm20948.icm20948_states.base_state.gyro_fullscale = 0;

	mask = 0b111111;
	val = 0;
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "REG_GYRO_CONFIG_2");
	modify_register(&device_icm20948.icm20948_states, REG_GYRO_CONFIG_2, mask, val);

	mask = 0b11111111;
	val = 0;
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "REG_GYRO_SMPLRT_DIV");
	modify_register(&device_icm20948.icm20948_states, REG_GYRO_SMPLRT_DIV, mask, val);
	/*-------------------*/
	inv_icm20948_read_mems_reg(&device_icm20948.icm20948_states, REG_GYRO_XOUT_H_SH, 6, test_data);
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Gyro Meas (XH,XL,YH,YL,ZH,ZL) : %d, %d, %d, %d, %d, %d", test_data[0],test_data[1],test_data[2],test_data[3],test_data[4],test_data[5]);



	/*-------------------*/

//	mask = 0b00100000;
//	val = 0b00000000;
//	INV_MSG(INV_MSG_LEVEL_VERBOSE, "REG_PWR_MGMT_1");
//	modify_register(&device_icm20948.icm20948_states, REG_PWR_MGMT_1, mask, val);
	inv_icm20948_read_mems_reg(&device_icm20948.icm20948_states, REG_PWR_MGMT_1, 1, test_data);
	INV_MSG(INV_MSG_LEVEL_INFO, "REG_PWR_MGMT_1 : %d", test_data[0]);
	/*-------------------*/
	inv_icm20948_read_mems_reg(&device_icm20948.icm20948_states, REG_PWR_MGMT_2, 1, test_data);
	INV_MSG(INV_MSG_LEVEL_INFO, "REG_PWR_MGMT_2 : %d", test_data[0]);
	/*-------------------*/
	// Disable DMP
	mask = 0b10000000;
	val = 0b00000000;
	modify_register(&device_icm20948.icm20948_states, REG_USER_CTRL, mask, val);
	/*-------------------*/
//	inv_icm20948_read_mems_reg(&device_icm20948.icm20948_states, REG_LP_CONFIG, 1, test_data);
//	INV_MSG(INV_MSG_LEVEL_INFO, "REG_LP_CONFIG : %d", test_data[0]);
	mask = 0b01110000;
	val = 0;
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "REG_LP_CONFIG");
	modify_register(&device_icm20948.icm20948_states, REG_LP_CONFIG, mask, val);

	/*-------------------*/
	inv_icm20948_read_mems_reg(&device_icm20948.icm20948_states, REG_INT_STATUS_1, 1, test_data);
	INV_MSG(INV_MSG_LEVEL_INFO, "REG_INT_STATUS_1 : %d", test_data[0]);


	inv_icm20948_read_mems_reg(&device_icm20948.icm20948_states, REG_PWR_MGMT_1, 1, test_data);
	INV_MSG(INV_MSG_LEVEL_INFO, "REG_PWR_MGMT_1 : %d", test_data[0]);
	/*-------------------*/
	// Enable All Accel and Gyro Axis
	inv_icm20948_read_mems_reg(&device_icm20948.icm20948_states, REG_PWR_MGMT_2, 1, test_data);
	INV_MSG(INV_MSG_LEVEL_INFO, "REG_PWR_MGMT_2 : %d", test_data[0]);
	mask = 0b111111;
	val = 0b000111;
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "REG_PWR_MGMT_2");
	modify_register(&device_icm20948.icm20948_states, REG_PWR_MGMT_2, mask, val);

    float accel_float[3];
    int16_t gyro_x, gyro_y, gyro_z;
    signed long  long_data[3] = {0};
	int accel_accuracy;
	float scale;

	accel_accuracy = inv_icm20948_get_accel_accuracy();
	scale = (1 << inv_icm20948_get_accel_fullscale(&device_icm20948.icm20948_states)) * 2.f / (1L<<15); // Convert from raw units to g's
  while (1)
  {
//    INV_MSG(INV_MSG_LEVEL_INFO, "Polling Sensor");
	  // This is the DMP Polling Stuff
//	if (irq_from_device & TO_MASK(ACCEL_INT_Pin)) {
//		rc = inv_device_poll(device);
//		check_rc(rc);
//
//		if(rc >= 0) {
//			__disable_irq();
//			irq_from_device &= ~TO_MASK(ACCEL_INT_Pin);
//			__enable_irq();
//		}
//	}
	  if (irq_from_device & TO_MASK(ACCEL_INT_Pin)) {
		  // Read INT REG
		  inv_icm20948_read_mems_reg(&device_icm20948.icm20948_states, REG_INT_STATUS_1, 1, test_data);
		  // Check INT

		  if(test_data[0])
		  {
			inv_icm20948_read_mems_reg(&device_icm20948.icm20948_states, REG_ACCEL_XOUT_H_SH, 6, test_data);
			long_data[0] = (int16_t)((test_data[0] << 8) | test_data[1]);
			long_data[1] = (int16_t)((test_data[2] << 8) | test_data[3]);
		    long_data[2] = (int16_t)((test_data[4] << 8) | test_data[5]);
		    // inv_icm20948_convert_dmp3_to_body(&device_icm20948.icm20948_states, long_data, scale, accel_float);
			accel_float[0] = long_data[0]*scale;
			accel_float[1] = long_data[1]*scale;
			accel_float[2] = long_data[2]*scale;

			inv_icm20948_read_mems_reg(&device_icm20948.icm20948_states, REG_GYRO_XOUT_H_SH, 6, test_data);
		    gyro_x = (int16_t)((test_data[0] << 8) | test_data[1]);
		    gyro_y = (int16_t)((test_data[2] << 8) | test_data[3]);
		    gyro_z = (int16_t)((test_data[4] << 8) | test_data[5]);

		    INV_MSG(INV_MSG_LEVEL_VERBOSE, "Accel Measurements: X=%f, Y=%f, Z=%f", accel_float[0], accel_float[1], accel_float[2]);
//		    INV_MSG(INV_MSG_LEVEL_VERBOSE, "Gyro Measurements: X=%d, Y=%d, Z=%d", gyro_x, gyro_y, gyro_z);
		  }
	  }
//	if (int_read_back & 0x8)
//	{
//		INV_MSG(INV_MSG_LEVEL_INFO, "Raw Data Int");
//	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*
 * Callback called upon sensor event reception
 * This function is called in the same context as inv_device_poll()
 */
static void sensor_event_cb(const inv_sensor_event_t * event, void * arg)
{
	/* arg will contained the value provided at init time */
	(void)arg;
	/*
	 * In normal mode, display sensor event over UART messages
	 */

	if(event->status == INV_SENSOR_STATUS_DATA_UPDATED) {

		switch(INV_SENSOR_ID_TO_TYPE(event->sensor)) {
		case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
		case INV_SENSOR_TYPE_RAW_GYROSCOPE:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (lsb): %llu %d %d %d",
					inv_sensor_str(event->sensor),
					event->timestamp,
					(int)event->data.raw3d.vect[0],
					(int)event->data.raw3d.vect[1],
					(int)event->data.raw3d.vect[2]);

			break;
		case INV_SENSOR_TYPE_ACCELEROMETER:
		case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
		case INV_SENSOR_TYPE_GRAVITY:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (mg): %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.acc.vect[0]*1000),
					(int)(event->data.acc.vect[1]*1000),
					(int)(event->data.acc.vect[2]*1000));
			break;
		case INV_SENSOR_TYPE_GYROSCOPE:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (mdps): %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.gyr.vect[0]*1000),
					(int)(event->data.gyr.vect[1]*1000),
					(int)(event->data.gyr.vect[2]*1000));
			break;
		case INV_SENSOR_TYPE_MAGNETOMETER:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (nT): %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.mag.vect[0]*1000),
					(int)(event->data.mag.vect[1]*1000),
					(int)(event->data.mag.vect[2]*1000));
			break;
		case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (mdps): %d %d %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.gyr.vect[0]*1000),
					(int)(event->data.gyr.vect[1]*1000),
					(int)(event->data.gyr.vect[2]*1000),
					(int)(event->data.gyr.bias[0]*1000),
					(int)(event->data.gyr.bias[1]*1000),
					(int)(event->data.gyr.bias[2]*1000));
			break;
		case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (nT): %d %d %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.mag.vect[0]*1000),
					(int)(event->data.mag.vect[1]*1000),
					(int)(event->data.mag.vect[2]*1000),
					(int)(event->data.mag.bias[0]*1000),
					(int)(event->data.mag.bias[1]*1000),
					(int)(event->data.mag.bias[2]*1000));
			break;
		case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
		case INV_SENSOR_TYPE_ROTATION_VECTOR:
		case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (e-3): %d %d %d %d ", inv_sensor_str(event->sensor),
					(int)(event->data.quaternion.quat[0]*1000),
					(int)(event->data.quaternion.quat[1]*1000),
					(int)(event->data.quaternion.quat[2]*1000),
					(int)(event->data.quaternion.quat[3]*1000));
			break;
		case INV_SENSOR_TYPE_ORIENTATION:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (e-3): %d %d %d ", inv_sensor_str(event->sensor),
					(int)(event->data.orientation.x*1000),
					(int)(event->data.orientation.y*1000),
					(int)(event->data.orientation.z*1000));
			break;
		case INV_SENSOR_TYPE_BAC:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s : %d %s", inv_sensor_str(event->sensor),
					event->data.bac.event, activityName(event->data.bac.event));
			break;
		case INV_SENSOR_TYPE_STEP_COUNTER:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s : %lu", inv_sensor_str(event->sensor),
					(unsigned long)event->data.step.count);
			break;
		case INV_SENSOR_TYPE_PICK_UP_GESTURE:
		case INV_SENSOR_TYPE_STEP_DETECTOR:
		case INV_SENSOR_TYPE_SMD:
		case INV_SENSOR_TYPE_B2S:
		case INV_SENSOR_TYPE_TILT_DETECTOR:
		default:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s : ...", inv_sensor_str(event->sensor));
			break;
		}
	}
}

static void check_rc(int rc)
{
	if(rc == -1) {
		INV_MSG(INV_MSG_LEVEL_INFO, "BAD RC=%d", rc);
		while(1);
	}
}

static void msg_printer(int level, const char * str, va_list ap)
{
#ifdef INV_MSG_ENABLE
	static char out_str[256]; /* static to limit stack usage */
	unsigned idx = 0;
	const char * ptr = out_str;
	const char * s[INV_MSG_LEVEL_MAX] = {
		"",    // INV_MSG_LEVEL_OFF
		"[E] ", // INV_MSG_LEVEL_ERROR
		"[W] ", // INV_MSG_LEVEL_WARNING
		"[I] ", // INV_MSG_LEVEL_INFO
		"[V] ", // INV_MSG_LEVEL_VERBOSE
		"[D] ", // INV_MSG_LEVEL_DEBUG
	};

	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
	if(idx >= (sizeof(out_str)))
		return;
	idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
	if(idx >= (sizeof(out_str)))
		return;
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
	if(idx >= (sizeof(out_str)))
		return;

	// Transmit the message over UART
	HAL_UART_Transmit(&huart2, (uint8_t*)out_str, idx, HAL_MAX_DELAY);

#else
	(void)level, (void)str, (void)ap;
#endif
}

const char * activityName(int act)
{
	switch(act) {
	case INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_BEGIN:          return "BEGIN IN_VEHICLE";
	case INV_SENSOR_BAC_EVENT_ACT_WALKING_BEGIN:             return "BEGIN WALKING";
	case INV_SENSOR_BAC_EVENT_ACT_RUNNING_BEGIN:             return "BEGIN RUNNING";
	case INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_BEGIN:          return "BEGIN ON_BICYCLE";
	case INV_SENSOR_BAC_EVENT_ACT_TILT_BEGIN:                return "BEGIN TILT";
	case INV_SENSOR_BAC_EVENT_ACT_STILL_BEGIN:               return "BEGIN STILL";
	case INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_END:            return "END IN_VEHICLE";
	case INV_SENSOR_BAC_EVENT_ACT_WALKING_END:               return "END WALKING";
	case INV_SENSOR_BAC_EVENT_ACT_RUNNING_END:               return "END RUNNING";
	case INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_END:            return "END ON_BICYCLE";
	case INV_SENSOR_BAC_EVENT_ACT_TILT_END:                  return "END TILT";
	case INV_SENSOR_BAC_EVENT_ACT_STILL_END:                 return "END STILL";
	default:                                                 return "unknown activity!";
	}
}

void inv_icm20948_sleep_us(int us)
{
	delay_us(us);
}

uint64_t inv_icm20948_get_time_us(void) {
//	return timer_get_counter(TIMEBASE_TIMER);
	return TIMEBASE_TIMER->CNT;
}

void ext_interrupt_cb(void * context, int int_num)
{
	(void)context;
	last_irq_time = inv_icm20948_get_time_us();
	irq_from_device = TO_MASK(int_num);
}

// EXTI Line9 External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	ext_interrupt_cb(0, ACCEL_INT_Pin);
}

void modify_register(struct inv_icm20948 * s, uint16_t reg, unsigned char mask, unsigned char value) {
    unsigned char data[6];  // Adjust the size based on maximum data size needed
    unsigned char originalValue;

    // Read the current register value
    inv_icm20948_read_mems_reg(s, reg, 1, data);
    INV_MSG(INV_MSG_LEVEL_INFO, "Read from %u : %u", reg, data[0]);

    // Modify the register value based on mask and provided value
    originalValue = data[0];
    data[0] = (originalValue & ~mask) | (value & mask);  // Apply mask: clear bits using ~mask, then set from value

    // Write the modified value back to the register
    inv_icm20948_write_single_mems_reg(s, reg, data[0]);
    INV_MSG(INV_MSG_LEVEL_INFO, "Wrote to %u : %u", reg, data[0]);

    // Confirm the write by reading back the register
    inv_icm20948_read_mems_reg(s, reg, 1, data);
    INV_MSG(INV_MSG_LEVEL_INFO, "Confirmed from %u : %u", reg, data[0]);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

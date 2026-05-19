/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "string.h"
#include <float.h>

#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

//Calibration
#define ERROR_CHECK_TIMER 5000
#define M 72
#define Z 9
#define Delta 45
#define ARRAY_SIZE 10
#define N 80
#define SIGNIFICANT_CHANGE 0.05
#define WINDOW_SIZE 10
extern uint8_t number_bridge;
#define ANGLE_RED 26.83

#define NUM_SYSTEM_TYPES 7

//Calculation current, speed, angle
extern uint16_t adcValBuf[6];
extern float discrete_time, encoder_presc;

//CAN
extern CAN_TxHeaderTypeDef TxAngles;
extern CAN_TxHeaderTypeDef TxSpeeds;
extern CAN_TxHeaderTypeDef TxCurrents;
extern CAN_RxHeaderTypeDef RxHeader, RxHeader1;
extern uint8_t RxData[8], RxData1[8];

extern uint8_t TxDataAngles[8], TxDataSpeeds[8], TxDataCurrents[8];
extern uint8_t RxAngles[8], RxOther[8], RxCoeffs1[8], RxCoeffs2[8],
		RxCoeffs3[8], RxCoeffs4[8], RxCoeffs5[8], RxCoeffs6[8], RxCoeffs7[8];

extern uint32_t error_can, received_call, mailbox, transmitting,
		last_received_call;

extern uint8_t flag_of_rx, flag_of_tx, queue;

extern int rc_angle1, rc_angle2, rc_kp_angle1, rc_kp_angle2, rc_ki_angle1,
		rc_ki_angle2, rc_kd_angle1, rc_kd_angle2, rc_kp_speed1, rc_kp_speed2,
		rc_ki_speed1, rc_ki_speed2, rc_kp_curr1, rc_kp_curr2, rc_ki_curr1,
		rc_ki_curr2;
extern uint8_t rc_other1_obstacle, rc_other2_obstacle, rc_other1_flag_phase,
		rc_other2_flag_phase, rc_other1_adaptive_phase,
		rc_other2_adaptive_phase, rc_other_set_to_zero, rc_other_reset,
		rc_other1_system_type, rc_other2_system_type, rc_speed_limit1,
		rc_speed_limit2;

extern volatile uint8_t pid_flag;
extern volatile uint8_t adcFlag;
extern volatile uint8_t enc_flag;

extern uint16_t adcValueCh1, adcValueCh2;

extern float tasking_1, tasking_2;
extern float g_fs_hz, g_fc_hz;

struct Motor {

	// Control system
	volatile int counter_pid;
	float task, task_real, err_angle;

	float out_angle, P_angle, I_angle, D_angle, err_speed, P_speed, I_speed,
			out_speed, err_current, P_current, I_current;

	float out_current, out_current_limited, out_pwm, kp_angle, ki_angle,
			out_speed_limited, limit_speed, kd_angle, kp_speed;

	float ki_speed, kp_current, ki_current, angle, speed, current;
	float pwm_difference, hyst_error;

	float previous_pwm;
//	int angle_smooth, counter_pid_smooth;
//
//	int osc_counter;
	float angle_reduction;

	float out_limited;
	float scale;

	float desired_speed;
	float speed_ramp_rate;

	float speed_scale;          // 1.0f by default
	float speed_filtered;       // init = 0.0f
	float prev_err_speed;       // init = 0.0f
	float D_speed;              // internal storage (init = 0.0f)
	int final_mode;             // init = 0
	float angle_final_threshold; // e.g. 5.0f (deg)
	float final_speed_max;       // e.g. 30.0f (deg/s)
	float settle_angle;          // e.g. 0.5f (deg)
	float settle_speed;          // e.g. 1.0f (deg/s)
	float speed_filter_alpha;    // e.g. 0.3f (0..1)
	float kd_speed;              // base kd for speed (e.g. 0.01f)
	float speed_limit;
	float curr_angle;

	void (*set_forward)(void);
	void (*set_backward)(void);
	void (*set_pwm)(struct Motor *motor, uint32_t pwm_value);

	// Phase system
//	int phase_a, task_phase, initial_task, angle_phase_counter, direction,
//			phase_start;
//	int obstacle, threshold, phase_counter, phase_stop;
//	int prev_task;
//	float speed_limit_min, speed_limit_max, speed_reduction, phase_difference,
//			scaled_difference;
//	float scale_red;
//	float add_counter;
//
//	// Current algorithm
//	float detected_obstacle;

	// Find zero
	int iminA, iminB, imaxA, imaxB;
	int AHigh[2], ALow[2], BHigh[2], BLow[2];
	uint32_t A, B;
	float Result;
	float Result_prev;

	int maxA;
	int maxB;
	int minA;
	int minB;

	uint32_t CMap[92][2];

	int counter_find_zero;
	int flag_current_pos_calc;
	int counter_current_pos_calc;
	int flag_find_zero;
};

void BlindZoneDesignation(struct Motor *motor);
void Read(struct Motor *motor);
int map();
void Calibration(struct Motor *motor);
void CurrentPositionCalculation(struct Motor *motor);
void CMapPrint();
void current_voltage(uint16_t adcValueCh1, uint16_t adcValueCh2);

void pid_angle_speed_curr(struct Motor *motor, float task);
void pid_angle_speed(struct Motor *motor, float task);
void pid_angle(struct Motor *motor, float task);
void pid_speed_curr(struct Motor *motor, float task);
void pid_angle_curr(struct Motor *motor, float task);
void pid_speed(struct Motor *motor, float task);
void pid_curr(struct Motor *motor, float task);

void filters_init(float adc_fs_hz, float cutoff_fc_hz);
void koef_pid();

void speed_angle();
void init_motor_1();
void init_motor_2();
void init_adc_base();
void motor1_backward();
void motor1_forward();
void motor2_forward();
void motor2_backward();
void set_struct_motors();
void motor1_set_pwm();
void motor2_set_pwm();
void pid_enable();
float map_angle(float x, float in_min, float in_max, float out_min,
		float out_max);
void calibration_maps();
//void phase_angle_support(struct Motor *motor, int task);
void init_all();

void init_CAN_TX();
void pack_CAN_data();
void send_info_CAN();
void DecodePart1();
void DecodePart2();
int32_t ConvertBytesToInt32(uint8_t *data);
int32_t ConvertBytesToInt32_LittleEndian(uint8_t *data);

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern struct Motor motor_1;
extern struct Motor motor_2;

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN_2_Pin GPIO_PIN_1
#define IN_2_GPIO_Port GPIOB
#define LED_CHECK_Pin GPIO_PIN_10
#define LED_CHECK_GPIO_Port GPIOA
#define IN_1_Pin GPIO_PIN_6
#define IN_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define _ENABLE_MOTOR_1 1
#define _ENABLE_MOTOR_2 1

#define _CS1_CPU_POWER 3.3
#define _CS1_QUANT_LEVEL 4096.0
#define _CS1_PRESCAL (_CS1_CPU_POWER / _CS1_QUANT_LEVEL)
#define _CS1_VOLTAGE_MID_POINT 1.6
#define _CS1_SENS 5

#define _CS2_CPU_POWER 3.3
#define _CS2_QUANT_LEVEL 4096.0
#define _CS2_PRESCAL (_CS2_CPU_POWER / _CS2_QUANT_LEVEL)
#define _CS2_VOLTAGE_MID_POINT 1.6
#define _CS2_SENS 5
#define current_thresh 0.03

#define _SPI_ENABLE 1

//CAN
extern CAN_TxHeaderTypeDef TxAngles;
extern CAN_TxHeaderTypeDef TxSpeeds;
extern CAN_TxHeaderTypeDef TxCurrents;
extern CAN_TxHeaderTypeDef TxOther;

extern CAN_TxHeaderTypeDef TxCoeffs1;
extern CAN_TxHeaderTypeDef TxCoeffs2;
extern CAN_TxHeaderTypeDef TxCoeffs3;
extern CAN_TxHeaderTypeDef TxCoeffs4;
extern CAN_TxHeaderTypeDef TxCoeffs5;
extern CAN_TxHeaderTypeDef TxCoeffs6;
extern CAN_TxHeaderTypeDef TxCoeffs7;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

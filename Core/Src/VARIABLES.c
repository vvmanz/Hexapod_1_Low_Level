#include "main.h"

struct Motor motor_1;
struct Motor motor_2;

//CAN
CAN_TxHeaderTypeDef TxAngles;
CAN_TxHeaderTypeDef TxSpeeds;
CAN_TxHeaderTypeDef TxCurrents;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8] = { 0, };
CAN_RxHeaderTypeDef RxHeader1;
uint8_t RxData1[8] = { 0, };

uint8_t TxDataAngles[8], TxDataSpeeds[8], TxDataCurrents[8];
uint8_t RxAngles[8] = { 0, };
uint8_t RxOther[8] = { 0, };
uint8_t RxCoeffs1[8] = { 0, };
uint8_t RxCoeffs2[8] = { 0, };
uint8_t RxCoeffs3[8] = { 0, };
uint8_t RxCoeffs4[8] = { 0, };
uint8_t RxCoeffs5[8] = { 0, };
uint8_t RxCoeffs6[8] = { 0, };
uint8_t RxCoeffs7[8] = { 0, };

uint32_t error_can;
uint32_t received_call;
uint8_t flag_of_rx = 0;
uint8_t flag_of_tx = 0;
uint32_t mailbox = 0;
uint8_t queue;
uint32_t transmitting = 0;
uint32_t last_received_call = 0;

int rc_angle1 = 0;
int rc_angle2 = 0;
uint8_t rc_other1_obstacle = 0;
uint8_t rc_other2_obstacle = 0;

uint8_t rc_other1_flag_phase = 0; // 0-ый бит RxOther[2]
uint8_t rc_other2_flag_phase = 0; // 1-ый бит RxOther[2]
uint8_t rc_other_set_to_zero = 0; // 2 бит RxOther[2]

uint8_t rc_other1_system_type = 0; //3-5 биты RxOther[2]
uint8_t rc_other2_system_type = 0; //

uint8_t rc_speed_limit1, rc_speed_limit2;

int rc_kp_angle1 = 4 * 1000;
int rc_kp_angle2 = 4 * 1000;

int rc_ki_angle1 = 4 * 1000;
int rc_ki_angle2 = 4 * 1000;
int rc_kd_angle1 = 10;
int rc_kd_angle2 = 10;
int rc_kp_speed1 = 5;

int rc_kp_speed2 = 5;
int rc_ki_speed1 = 5;
int rc_ki_speed2 = 5;

int rc_kp_curr1 = 50;
int rc_kp_curr2 = 50;

int rc_ki_curr1 = 3000;
int rc_ki_curr2 = 3000;

// ADC (Current, absolute encoders)

extern uint16_t adcValBuf[6];
extern uint16_t adcValueCh1, adcValueCh2;
extern uint16_t adc_angle_1_1, adc_angle_1_2, adc_angle_2_1, adc_angle_2_2;

extern float cs1Voltage, cs2Voltage;
extern float cs1Voltage_prev, cs2Voltage_prev;
extern float cs1Buffer[WINDOW_SIZE], cs2Buffer[WINDOW_SIZE];
extern int cs1Index, cs2Index;

// Speed calculation

extern float encoder_presc;
extern float motor_1_speed_prev, motor_2_speed_prev;
extern uint32_t current_encoder_value_1, current_encoder_value_2;
extern int32_t total_encoder_ticks_1, total_encoder_ticks_2;
extern int32_t encoder_ticks_1, encoder_ticks_2;
extern int32_t prev_encoder_value_1, prev_encoder_value_2;

// PID (Slave control system)

float tasking_1 = 0;
float tasking_2 = 0;
extern float discrete_time, limit_angle, limit_speed, limit_curr, pwm_step,
		percentage, discrete_time_angle_velocity, discrete_time_current;

float com_mot1, com_mot2;

// Calibration
uint8_t number_bridge = 0;

uint8_t flag_position = 0;
int position_counter = 0;

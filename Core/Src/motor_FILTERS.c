#include "main.h"
#include "math.h"
#include "tim.h"
#include "gpio.h"
#include "adc.h"

float cs1Voltage, cs2Voltage;
float cs1Voltage_prev = 0.001;
float cs2Voltage_prev = 0.001;

float cs1Buffer[WINDOW_SIZE] = { 0 };
float cs2Buffer[WINDOW_SIZE] = { 0 };
int cs1Index = 0;
int cs2Index = 0;

float motor_1_speed_prev = 0.0;
float motor_2_speed_prev = 0.0;
uint32_t current_encoder_value_1, current_encoder_value_2;
int32_t total_encoder_ticks_1, total_encoder_ticks_2;
int32_t encoder_ticks_1, encoder_ticks_2;
int32_t prev_encoder_value_1, prev_encoder_value_2;
float alpha_1 = 0.1;
float alpha_2 = 0.1;

float updateMovingAverage(float buffer[], int *index, float newValue) {
	buffer[*index] = newValue;
	*index = (*index + 1) % WINDOW_SIZE;

	float sum = 0.0;
	for (int i = 0; i < WINDOW_SIZE; i++) {
		sum += buffer[i];
	}
	return sum / WINDOW_SIZE;
}
float g_fs_hz = 16700.0f;
float g_fc_hz = 10.0f;

typedef struct {
	float y;
	float alpha;
	int initialized;
} IIRFilter;

static void IIR_init(IIRFilter *f, float alpha) {
	f->alpha = alpha;
	f->initialized = 0;
	f->y = 0.0f;
}
static float IIR_update(IIRFilter *f, float x) {
	if (!f->initialized) {
		f->y = x;
		f->initialized = 1;
		return f->y;
	}
	f->y += f->alpha * (x - f->y);
	return f->y;
}
static float compute_alpha_from_fc_dt(float fc, float dt) {
	if (fc <= 0.0f || dt <= 0.0f)
		return 1.0f;
	float tau = 1.0f / (2.0f * 3.14159265358979323846f * fc);
	return dt / (tau + dt);
}
static inline float median3(float a, float b, float c) {
	if (a > b) {
		float t = a;
		a = b;
		b = t;
	}
	if (b > c) {
		float t = b;
		b = c;
		c = t;
	}
	if (a > b) {
		float t = a;
		a = b;
		b = t;
	}
	return b;
}
static IIRFilter cs1_iir;
static IIRFilter cs2_iir;

static float cs1_prev1 = 0.0f;
static float cs1_prev2 = 0.0f;
static float cs2_prev1 = 0.0f;
static float cs2_prev2 = 0.0f;
void filters_init(float adc_fs_hz, float cutoff_fc_hz) {
	if (adc_fs_hz > 0.0f)
		g_fs_hz = adc_fs_hz;
	if (cutoff_fc_hz > 0.0f)
		g_fc_hz = cutoff_fc_hz;
	float dt = 1.0f / g_fs_hz;
	float alpha = compute_alpha_from_fc_dt(g_fc_hz, dt);
	IIR_init(&cs1_iir, alpha);
	IIR_init(&cs2_iir, alpha);
	cs1_prev1 = cs1_prev2 = 0.0f;
	cs2_prev1 = cs2_prev2 = 0.0f;
}
void current_voltage(uint16_t adcValueCh1, uint16_t adcValueCh2) {
	float raw_cs1 = (float) adcValueCh1 * _CS1_PRESCAL;
	float raw_cs2 = (float) adcValueCh2 * _CS2_PRESCAL;
	float med_cs1 = median3(cs1_prev2, cs1_prev1, raw_cs1);
	float med_cs2 = median3(cs2_prev2, cs2_prev1, raw_cs2);
	cs1_prev2 = cs1_prev1;
	cs1_prev1 = raw_cs1;
	cs2_prev2 = cs2_prev1;
	cs2_prev1 = raw_cs2;
	cs1Voltage = IIR_update(&cs1_iir, med_cs1);
	cs2Voltage = IIR_update(&cs2_iir, med_cs2);
	if (fabs(
			-(cs1Voltage - _CS1_VOLTAGE_MID_POINT) / _CS1_SENS) > current_thresh) {
		motor_1.current = -(cs1Voltage - _CS1_VOLTAGE_MID_POINT) / _CS1_SENS;
	} else {
		motor_1.current = 0;
	}
	if (fabs(
			+(cs2Voltage - _CS2_VOLTAGE_MID_POINT) / _CS2_SENS) > current_thresh) {
		motor_2.current = +(cs2Voltage - _CS2_VOLTAGE_MID_POINT) / _CS2_SENS;
	} else {
		motor_2.current = 0;
	}
}
void speed_angle() {
	current_encoder_value_1 = TIM1->CNT;
	current_encoder_value_2 = TIM2->CNT;
	encoder_ticks_1 =
			(int32_t) (current_encoder_value_1 - prev_encoder_value_1);
	encoder_ticks_1 = encoder_ticks_1;

	encoder_ticks_2 =
			(int32_t) (current_encoder_value_2 - prev_encoder_value_2);

	encoder_ticks_2 = encoder_ticks_2;

	if (encoder_ticks_1 > 500) {
		encoder_ticks_1 -= 1000;
	} else if (encoder_ticks_1 < -500) {
		encoder_ticks_1 += 1000;
	}
	if (encoder_ticks_2 > 500) {
		encoder_ticks_2 -= 1000;
	} else if (encoder_ticks_2 < -500) {
		encoder_ticks_2 += 1000;
	}
	total_encoder_ticks_1 += encoder_ticks_1;
	total_encoder_ticks_2 += encoder_ticks_2;

	motor_1.angle = total_encoder_ticks_1 * encoder_presc;
	motor_1.speed = (encoder_ticks_1 / 1000.0) * 60.0 / (discrete_time);
	motor_1.angle_reduction = motor_1.angle / ANGLE_RED;

	motor_2.angle = -total_encoder_ticks_2 * encoder_presc;
	motor_2.speed = -(encoder_ticks_2 / 1000.0) * 60.0 / (discrete_time);
	motor_2.angle_reduction = motor_2.angle / ANGLE_RED;

	motor_1.speed = alpha_1 * motor_1.speed
			+ (1 - alpha_1) * motor_1_speed_prev;
	motor_1_speed_prev = motor_1.speed;
	motor_2.speed = alpha_2 * motor_2.speed
			+ (1 - alpha_2) * motor_2_speed_prev;
	motor_2_speed_prev = motor_2.speed;
	prev_encoder_value_1 = current_encoder_value_1;
	prev_encoder_value_2 = current_encoder_value_2;
}

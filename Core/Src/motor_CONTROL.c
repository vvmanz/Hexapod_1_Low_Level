#include "main.h"
#include "math.h"
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "adc.h"

float pwm_step;
float percentage;

float discrete_time = 0.00016666666;
float limit_angle = 8000;
float limit_speed = 4.0;
float limit_curr = 1;

float discrete_time_angle_velocity = 0.005;
float discrete_time_current = 0.00016666666;
float encoder_presc = 0.36;

static void apply_direction_and_set_pwm(struct Motor *motor, float out_value,
		float used_limit) {

	if (out_value > 0.0f) {
		if (motor == &motor_1)
			motor->set_forward();
		if (motor == &motor_2)
			motor->set_backward();
	} else if (out_value < 0.0f) {
		if (motor == &motor_1)
			motor->set_backward();
		if (motor == &motor_2)
			motor->set_forward();
	}

	motor->out_limited = out_value;
	if (motor->out_limited > used_limit)
		motor->out_limited = used_limit;
	if (motor->out_limited < -used_limit)
		motor->out_limited = -used_limit;

	motor->scale = 1000.0f / used_limit;
	motor->out_pwm = fabsf(motor->out_limited * motor->scale);

	if (motor->out_pwm > 1000.0f)
		motor->out_pwm = 1000.0f;

	motor->pwm_difference = motor->out_pwm - motor->previous_pwm;
	if (fabsf(motor->pwm_difference) > pwm_step) {
		if (motor->pwm_difference > 0.0f)
			motor->out_pwm = motor->previous_pwm + pwm_step;
		else
			motor->out_pwm = motor->previous_pwm - pwm_step;
	}

	motor->set_pwm(motor, motor->out_pwm);
	motor->previous_pwm = motor->out_pwm;
	motor->out_pwm = motor->out_pwm;
	motor->out_current_limited = motor->out_limited;
}

void pid_angle_speed_curr(struct Motor *motor, float task) {
	motor->counter_pid++;
	motor->task = task * ANGLE_RED;
	motor->task_real = task;

	motor->err_angle = motor->task - motor->angle;
	if (fabsf(motor->err_angle) <= motor->hyst_error) {
		motor->err_angle = 0.0f;
		motor->I_angle = 0.0f;
		motor->D_angle = 0.0f;
	}
	motor->P_angle = motor->err_angle * motor->kp_angle;
	if (motor->counter_pid >= 30) {
		motor->I_angle += motor->err_angle * motor->ki_angle
				* discrete_time_angle_velocity;
		motor->D_angle = motor->err_angle * motor->kd_angle
				/ discrete_time_angle_velocity;
		if (motor->I_angle > limit_angle)
			motor->I_angle = limit_angle;
		if (motor->I_angle < -limit_angle)
			motor->I_angle = -limit_angle;
	}
	motor->out_angle = motor->P_angle + motor->I_angle + motor->D_angle;
	if (motor->out_angle > limit_angle)
		motor->out_angle = limit_angle;
	if (motor->out_angle < -limit_angle)
		motor->out_angle = -limit_angle;

	float alpha = motor->speed_filter_alpha;
	if (alpha < 0.0f)
		alpha = 0.0f;
	if (alpha > 1.0f)
		alpha = 1.0f;
	motor->speed_filtered = alpha * motor->speed
			+ (1.0f - alpha) * motor->speed_filtered;
	float abs_err_angle = fabsf(motor->err_angle);
	if (abs_err_angle <= motor->angle_final_threshold) {
		if (!motor->final_mode) {
			motor->final_mode = 1;
			motor->I_speed *= 0.0f;
			motor->prev_err_speed = 0.0f;
		}
	} else {
		motor->final_mode = 0;
	}
	float raw_target_speed = motor->out_angle
			* (isfinite(motor->speed_scale) ? motor->speed_scale : 1.0f);
	if (motor->final_mode) {
		float Kpos2vel = motor->final_speed_max
				/ (motor->angle_final_threshold > 0.0001f ?
						motor->angle_final_threshold : 1.0f);
		float scaled_target = motor->err_angle * Kpos2vel;
		if (scaled_target > motor->final_speed_max)
			scaled_target = motor->final_speed_max;
		if (scaled_target < -motor->final_speed_max)
			scaled_target = -motor->final_speed_max;
		raw_target_speed = scaled_target;
	}
	if (!isfinite(motor->desired_speed))
		motor->desired_speed = 0.0f;

	float dt = discrete_time_angle_velocity;
	if (dt <= 0.0f)
		dt = 0.001f;
	float max_delta = motor->speed_ramp_rate * dt;
	float delta = raw_target_speed - motor->desired_speed;
	if (delta > max_delta)
		delta = max_delta;
	else if (delta < -max_delta)
		delta = -max_delta;
	motor->desired_speed += delta;
	motor->err_speed = motor->desired_speed - motor->speed_filtered;
	if (fabsf(motor->err_speed) <= motor->hyst_error) {
		motor->err_speed = 0.0f;
	}
	float kp_speed = motor->kp_speed;
	float ki_speed = motor->ki_speed;
	float kd_speed = motor->kd_speed;

	if (motor->final_mode) {
		kp_speed *= 0.3f;
		ki_speed *= 0.0f;
		kd_speed *= 3.0f;
	}
	motor->P_speed = motor->err_speed * kp_speed;
	motor->D_speed = 0.0f;
	if (dt > 0.0f) {
		motor->D_speed = (motor->err_speed - motor->prev_err_speed) / dt
				* kd_speed;
	}
	motor->prev_err_speed = motor->err_speed;
	if (motor->counter_pid >= 30) {
		float tentative_I = motor->I_speed + motor->err_speed * ki_speed * dt;
		if (tentative_I > limit_speed)
			tentative_I = limit_speed;
		if (tentative_I < -limit_speed)
			tentative_I = -limit_speed;

		float tentative_out = motor->P_speed + tentative_I + motor->D_speed;
		if ((tentative_out > limit_speed && motor->err_speed > 0.0f)
				|| (tentative_out < -limit_speed && motor->err_speed < 0.0f)) {
		} else {
			motor->I_speed = tentative_I;
		}
		motor->counter_pid = 0;
	}
	motor->out_speed = motor->P_speed + motor->I_speed + motor->D_speed;
	if (motor->out_speed > limit_speed)
		motor->out_speed = limit_speed;
	if (motor->out_speed < -limit_speed)
		motor->out_speed = -limit_speed;
	motor->err_current = motor->out_speed - motor->current;

	if (fabsf(motor->err_angle) <= motor->hyst_error) {
		motor->err_current = 0.0f;
		motor->I_current = 0.0f;
	}

	motor->P_current = motor->err_current * motor->kp_current;
	motor->I_current += motor->err_current * motor->ki_current
			* discrete_time_current;
	if (motor->I_current > limit_curr)
		motor->I_current = limit_curr;
	if (motor->I_current < -limit_curr)
		motor->I_current = -limit_curr;
	motor->out_current = motor->P_current + motor->I_current;
	if (motor->out_current > limit_curr)
		motor->out_current = limit_curr;
	if (motor->out_current < -limit_curr)
		motor->out_current = -limit_curr;
	if (fabsf(motor->err_angle) <= motor->settle_angle
			&& fabsf(motor->speed_filtered) <= motor->settle_speed) {
		motor->I_speed = 0.0f;
		motor->I_current = 0.0f;
		motor->out_current = 0.0f;
		motor->desired_speed = 0.0f;
		motor->final_mode = 0;
		apply_direction_and_set_pwm(motor, 0.0f, limit_curr);
		return;
	}
	apply_direction_and_set_pwm(motor, motor->out_current, limit_curr);
}
void pid_angle_speed(struct Motor *motor, float task) {
	motor->counter_pid++;
	motor->task = task * ANGLE_RED;
	motor->task_real = task;

	// ANGLE PID (как в предыдущей реализации)
	motor->err_angle = motor->task - motor->angle;
	if (fabsf(motor->err_angle) <= motor->hyst_error) {
		motor->err_angle = 0.0f;
		motor->I_angle = 0.0f;
		motor->D_angle = 0.0f;
	}
	motor->P_angle = motor->err_angle * motor->kp_angle;
	if (motor->counter_pid >= 30) {
		motor->I_angle += motor->err_angle * motor->ki_angle
				* discrete_time_angle_velocity;
		motor->D_angle = motor->err_angle * motor->kd_angle
				/ discrete_time_angle_velocity;
		if (motor->I_angle > limit_angle)
			motor->I_angle = limit_angle;
		if (motor->I_angle < -limit_angle)
			motor->I_angle = -limit_angle;
	}
	motor->out_angle = motor->P_angle + motor->I_angle + motor->D_angle;
	if (motor->out_angle > limit_angle)
		motor->out_angle = limit_angle;
	if (motor->out_angle < -limit_angle)
		motor->out_angle = -limit_angle;

	// SPEED FILTER
	float alpha = motor->speed_filter_alpha;
	if (alpha < 0.0f)
		alpha = 0.0f;
	if (alpha > 1.0f)
		alpha = 1.0f;
	motor->speed_filtered = alpha * motor->speed
			+ (1.0f - alpha) * motor->speed_filtered;

	// FINAL MODE: находимся близко к углу -> мягкая посадка по скорости
	float abs_err_angle = fabsf(motor->err_angle);
	if (abs_err_angle <= motor->angle_final_threshold) {
		if (!motor->final_mode) {
			motor->final_mode = 1;
			motor->I_speed *= 0.0f; // можно 0.2 для мягкости
			motor->prev_err_speed = 0.0f;
		}
	} else {
		motor->final_mode = 0;
	}

	// RAW TARGET: используем только speed_scale как управляющий фактор
	float raw_target_speed = motor->out_angle
			* (isfinite(motor->speed_scale) ? motor->speed_scale : 1.0f);

	// Если финал — сделать целевую скорость пропорциональной ошибке угла (soft capture)
	if (motor->final_mode) {
		float Kpos2vel = motor->final_speed_max
				/ (motor->angle_final_threshold > 0.0001f ?
						motor->angle_final_threshold : 1.0f);
		float scaled_target = motor->err_angle * Kpos2vel;
		if (scaled_target > motor->final_speed_max)
			scaled_target = motor->final_speed_max;
		if (scaled_target < -motor->final_speed_max)
			scaled_target = -motor->final_speed_max;
		raw_target_speed = scaled_target;
	}

	// Инициализация desired_speed если надо
	if (!isfinite(motor->desired_speed))
		motor->desired_speed = 0.0f;

	// rate limiter (slew)
	float dt = discrete_time_angle_velocity;
	if (dt <= 0.0f)
		dt = 0.001f;
	float max_delta = motor->speed_ramp_rate * dt;
	float delta = raw_target_speed - motor->desired_speed;
	if (delta > max_delta)
		delta = max_delta;
	else if (delta < -max_delta)
		delta = -max_delta;
	motor->desired_speed += delta;

	// SPEED PID (P, I, D with scheduling)
	motor->err_speed = motor->desired_speed - motor->speed_filtered;
	if (fabsf(motor->err_speed) <= motor->hyst_error)
		motor->err_speed = 0.0f;

	float kp_speed = motor->kp_speed;
	float ki_speed = motor->ki_speed;
	float kd_speed = motor->kd_speed;

	if (motor->final_mode) {
		kp_speed *= 0.3f;
		ki_speed *= 0.0f;
		kd_speed *= 3.0f;
	}

	motor->P_speed = motor->err_speed * kp_speed;
	motor->D_speed = 0.0f;
	if (dt > 0.0f)
		motor->D_speed = (motor->err_speed - motor->prev_err_speed) / dt
				* kd_speed;
	motor->prev_err_speed = motor->err_speed;

	if (motor->counter_pid >= 30) {
		float tentative_I = motor->I_speed + motor->err_speed * ki_speed * dt;
		if (tentative_I > limit_speed)
			tentative_I = limit_speed;
		if (tentative_I < -limit_speed)
			tentative_I = -limit_speed;
		float tentative_out = motor->P_speed + tentative_I + motor->D_speed;
		if (!((tentative_out > limit_speed && motor->err_speed > 0.0f)
				|| (tentative_out < -limit_speed && motor->err_speed < 0.0f))) {
			motor->I_speed = tentative_I;
		}
		motor->counter_pid = 0;
	}

	motor->out_speed = motor->P_speed + motor->I_speed + motor->D_speed;
	if (motor->out_speed > limit_speed)
		motor->out_speed = limit_speed;
	if (motor->out_speed < -limit_speed)
		motor->out_speed = -limit_speed;

	// CURRENT/TORQUE part: здесь out_speed — желаемый ток/команда
	motor->err_current = motor->out_speed - motor->current;
	if (fabsf(motor->err_angle) <= motor->hyst_error) {
		motor->err_current = 0.0f;
		motor->I_current = 0.0f;
	}

	motor->P_current = motor->err_current * motor->kp_current;
	motor->I_current += motor->err_current * motor->ki_current
			* discrete_time_current;
	if (motor->I_current > limit_curr)
		motor->I_current = limit_curr;
	if (motor->I_current < -limit_curr)
		motor->I_current = -limit_curr;
	motor->out_current = motor->P_current + motor->I_current;
	if (motor->out_current > limit_curr)
		motor->out_current = limit_curr;
	if (motor->out_current < -limit_curr)
		motor->out_current = -limit_curr;

	// SETTLED: если достигли, мягко остановиться
	if (fabsf(motor->err_angle) <= motor->settle_angle
			&& fabsf(motor->speed_filtered) <= motor->settle_speed) {
		motor->I_speed = 0.0f;
		motor->I_current = 0.0f;
		motor->out_current = 0.0f;
		motor->desired_speed = 0.0f;
		motor->final_mode = 0;
		apply_direction_and_set_pwm(motor, 0.0f, limit_curr);
		return;
	}

	apply_direction_and_set_pwm(motor, motor->out_current, limit_curr);
}
void pid_angle_curr(struct Motor *motor, float task) {
	motor->counter_pid++;
	motor->task = task * ANGLE_RED;
	motor->task_real = task;
	motor->err_angle = motor->task - motor->angle;

	if (fabsf(motor->err_angle) <= motor->hyst_error) {
		motor->err_angle = 0.0f;
		motor->I_angle = 0.0f;
	}
	motor->P_angle = motor->err_angle * motor->kp_angle;

	if (motor->counter_pid >= 30) {
		motor->I_angle += motor->err_angle * motor->ki_angle
				* discrete_time_angle_velocity;
		motor->D_angle = motor->err_angle * motor->kd_angle
				/ discrete_time_angle_velocity;

		if (motor->I_angle > limit_angle) {
			motor->I_angle = limit_angle - motor->P_angle * percentage;
		}
		if (motor->I_angle < -limit_angle) {
			motor->I_angle = -limit_angle + motor->P_angle * percentage;
		}
		motor->counter_pid = 0;
	}

	motor->out_angle = motor->P_angle + motor->I_angle + motor->D_angle;
	if (motor->out_angle > limit_speed)
		motor->out_angle = limit_speed;
	if (motor->out_angle < -limit_speed)
		motor->out_angle = -limit_speed;

	motor->err_current = motor->out_angle - motor->current;

	if (fabsf(motor->err_angle) <= motor->hyst_error) {
		motor->err_current = 0.0f;
		motor->I_current = 0.0f;
	}

	motor->P_current = motor->err_current * motor->kp_current;
	motor->I_current += motor->err_current * motor->ki_current
			* discrete_time_current;

	if (motor->I_current >= limit_curr) {
		motor->I_current = limit_curr - motor->P_current * percentage;
	}
	if (motor->I_current <= -limit_curr) {
		motor->I_current = -limit_curr + motor->P_current * percentage;
	}

	motor->out_current = motor->P_current + motor->I_current;

	apply_direction_and_set_pwm(motor, motor->out_current, limit_curr);
}
void pid_angle(struct Motor *motor, float task) {
	motor->counter_pid++;
	motor->task = task * ANGLE_RED;
	motor->task_real = task;
	motor->err_angle = motor->task - motor->angle;

	motor->P_angle = motor->err_angle * motor->kp_angle;
	if (motor->counter_pid >= 30) {
		motor->I_angle += motor->err_angle * motor->ki_angle
				* discrete_time_angle_velocity;
		motor->D_angle = motor->err_angle * motor->kd_angle
				/ discrete_time_angle_velocity;

		if (motor->I_angle > limit_angle) {
			motor->I_angle = limit_angle;
		}
		if (motor->I_angle < -limit_angle) {
			motor->I_angle = -limit_angle;
		}
		motor->counter_pid = 0;
	}
	motor->out_angle = motor->P_angle + motor->I_angle + motor->D_angle;
	if (motor->out_angle > limit_angle)
		motor->out_angle = limit_angle;
	if (motor->out_angle < -limit_angle)
		motor->out_angle = -limit_angle;
	apply_direction_and_set_pwm(motor, motor->out_angle, limit_angle);
}

void pid_speed_curr(struct Motor *motor, float task) {
	motor->counter_pid++;
	motor->task = task;
	motor->task_real = task;

	float raw_target_speed = motor->task
			* (isfinite(motor->speed_scale) ? motor->speed_scale : 1.0f);
	if (!isfinite(motor->desired_speed))
		motor->desired_speed = 0.0f;
	float alpha = motor->speed_filter_alpha;
	if (alpha < 0.0f)
		alpha = 0.0f;
	if (alpha > 1.0f)
		alpha = 1.0f;
	motor->speed_filtered = alpha * motor->speed
			+ (1.0f - alpha) * motor->speed_filtered;
	motor->err_speed = raw_target_speed - motor->speed_filtered;
	if (fabsf(motor->err_speed) <= motor->settle_speed * 5.0f) {
		if (!motor->final_mode) {
			motor->final_mode = 1;
			motor->I_speed *= 0.0f;
			motor->prev_err_speed = 0.0f;
		}
	} else {
		motor->final_mode = 0;
	}
	float dt = discrete_time_angle_velocity;
	if (dt <= 0.0f)
		dt = 0.001f;
	float max_delta = motor->speed_ramp_rate * dt;
	float delta = raw_target_speed - motor->desired_speed;
	if (delta > max_delta)
		delta = max_delta;
	else if (delta < -max_delta)
		delta = -max_delta;
	motor->desired_speed += delta;
	motor->err_speed = motor->desired_speed - motor->speed_filtered;
	if (fabsf(motor->err_speed) <= motor->hyst_error)
		motor->err_speed = 0.0f;
	float kp = motor->kp_speed;
	float ki = motor->ki_speed;
	float kd = motor->kd_speed;
	if (motor->final_mode) {
		kp *= 0.4f;
		ki *= 0.0f;
		kd *= 3.0f;
	}
	motor->P_speed = motor->err_speed * kp;
	motor->D_speed = 0.0f;
	if (dt > 0.0f)
		motor->D_speed = (motor->err_speed - motor->prev_err_speed) / dt * kd;
	motor->prev_err_speed = motor->err_speed;
	if (motor->counter_pid >= 30) {
		float tentative_I = motor->I_speed + motor->err_speed * ki * dt;
		if (tentative_I > limit_speed)
			tentative_I = limit_speed;
		if (tentative_I < -limit_speed)
			tentative_I = -limit_speed;
		float tentative_out = motor->P_speed + tentative_I + motor->D_speed;
		if (!((tentative_out > limit_speed && motor->err_speed > 0.0f)
				|| (tentative_out < -limit_speed && motor->err_speed < 0.0f))) {
			motor->I_speed = tentative_I;
		}
		motor->counter_pid = 0;
	}
	motor->out_speed = motor->P_speed + motor->I_speed + motor->D_speed;
	if (motor->out_speed > limit_speed)
		motor->out_speed = limit_speed;
	if (motor->out_speed < -limit_speed)
		motor->out_speed = -limit_speed;
	motor->err_current = motor->out_speed - motor->current;
	motor->P_current = motor->err_current * motor->kp_current;
	motor->I_current += motor->err_current * motor->ki_current
			* discrete_time_current;
	if (motor->I_current > limit_curr)
		motor->I_current = limit_curr;
	if (motor->I_current < -limit_curr)
		motor->I_current = -limit_curr;
	motor->out_current = motor->P_current + motor->I_current;
	if (motor->out_current > limit_curr)
		motor->out_current = limit_curr;
	if (motor->out_current < -limit_curr)
		motor->out_current = -limit_curr;
	if (fabsf(motor->desired_speed - motor->speed_filtered)
			<= motor->settle_speed
			&& fabsf(motor->speed_filtered) <= motor->settle_speed) {
		motor->I_speed = 0.0f;
		motor->I_current = 0.0f;
		motor->out_current = 0.0f;
		motor->desired_speed = motor->desired_speed;
		apply_direction_and_set_pwm(motor, 0.0f, limit_curr);
		return;
	}

	apply_direction_and_set_pwm(motor, motor->out_current, limit_curr);
}

void pid_speed(struct Motor *motor, float task) {
	motor->counter_pid++;
	motor->task = task;
	motor->task_real = task;

	// raw target (scaled only by speed_scale)
	float raw_target = motor->task
			* (isfinite(motor->speed_scale) ? motor->speed_scale : 1.0f);

	// speed filter
	float alpha = motor->speed_filter_alpha;
	if (alpha < 0.0f)
		alpha = 0.0f;
	if (alpha > 1.0f)
		alpha = 1.0f;
	motor->speed_filtered = alpha * motor->speed
			+ (1.0f - alpha) * motor->speed_filtered;

	// initialize desired_speed if needed
	if (!isfinite(motor->desired_speed))
		motor->desired_speed = 0.0f;

	// final mode detection based on speed error
	motor->err_speed = raw_target - motor->speed_filtered;
	if (fabsf(motor->err_speed) <= motor->settle_speed * 5.0f) {
		if (!motor->final_mode) {
			motor->final_mode = 1;
			motor->I_speed *= 0.0f;
			motor->prev_err_speed = 0.0f;
		}
	} else {
		motor->final_mode = 0;
	}

	// rate limiter
	float dt = discrete_time_angle_velocity;
	if (dt <= 0.0f)
		dt = 0.001f;
	float max_delta = motor->speed_ramp_rate * dt;
	float delta = raw_target - motor->desired_speed;
	if (delta > max_delta)
		delta = max_delta;
	else if (delta < -max_delta)
		delta = -max_delta;
	motor->desired_speed += delta;

	// speed PID
	motor->err_speed = motor->desired_speed - motor->speed_filtered;
	if (fabsf(motor->err_speed) <= motor->hyst_error)
		motor->err_speed = 0.0f;

	float kp = motor->kp_speed;
	float ki = motor->ki_speed;
	float kd = motor->kd_speed;
	if (motor->final_mode) {
		kp *= 0.4f;
		ki *= 0.0f;
		kd *= 3.0f;
	}

	motor->P_speed = motor->err_speed * kp;
	motor->D_speed = 0.0f;
	if (dt > 0.0f)
		motor->D_speed = (motor->err_speed - motor->prev_err_speed) / dt * kd;
	motor->prev_err_speed = motor->err_speed;

	if (motor->counter_pid >= 30) {
		float tentative_I = motor->I_speed + motor->err_speed * ki * dt;
		if (tentative_I > limit_speed)
			tentative_I = limit_speed;
		if (tentative_I < -limit_speed)
			tentative_I = -limit_speed;
		float tentative_out = motor->P_speed + tentative_I + motor->D_speed;
		if (!((tentative_out > limit_speed && motor->err_speed > 0.0f)
				|| (tentative_out < -limit_speed && motor->err_speed < 0.0f))) {
			motor->I_speed = tentative_I;
		}
		motor->counter_pid = 0;
	}

	motor->out_speed = motor->P_speed + motor->I_speed + motor->D_speed;
	if (motor->out_speed > limit_speed)
		motor->out_speed = limit_speed;
	if (motor->out_speed < -limit_speed)
		motor->out_speed = -limit_speed;

	// apply PWM directly (speed controller outputs PWM scaled to limit_speed)
	apply_direction_and_set_pwm(motor, motor->out_speed, limit_speed);
}

void pid_curr(struct Motor *motor, float task) {
	motor->counter_pid++;
	motor->task = task;

	motor->err_current = motor->task - motor->current;
	motor->P_current = motor->err_current * motor->kp_current;
	motor->I_current += motor->err_current * motor->ki_current
			* discrete_time_current;

	if (motor->I_current >= limit_curr) {
		motor->I_current = limit_curr - motor->P_current * percentage;
	}
	if (motor->I_current <= -limit_curr) {
		motor->I_current = -limit_curr + motor->P_current * percentage;
	}

	motor->out_current = motor->P_current + motor->I_current;

	apply_direction_and_set_pwm(motor, motor->out_current, limit_curr);

}

void motor1_forward(void) {
	HAL_GPIO_WritePin(GPIOB, IN_1_Pin, GPIO_PIN_SET);
}

void motor1_backward(void) {
	HAL_GPIO_WritePin(GPIOB, IN_1_Pin, GPIO_PIN_RESET);
}

void motor2_forward(void) {
	HAL_GPIO_WritePin(GPIOB, IN_2_Pin, GPIO_PIN_SET);
}
void motor2_backward(void) {
	HAL_GPIO_WritePin(GPIOB, IN_2_Pin, GPIO_PIN_RESET);
}

void init_motor_1() {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
//	motor_1.phase_start = 0;
}
void init_motor_2() {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
//	motor_2.phase_start = 0;
}

void motor1_set_pwm(struct Motor *motor, uint32_t pwm_value) {
	TIM3->CCR2 = pwm_value;
}

void motor2_set_pwm(struct Motor *motor, uint32_t pwm_value) {
	TIM3->CCR3 = pwm_value;
}
void init_adc_base() {
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adcValBuf, 6);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
}

void set_struct_motors() {

	motor_1.set_forward = motor1_forward;
	motor_1.set_backward = motor1_backward;

	motor_2.set_forward = motor2_forward;
	motor_2.set_backward = motor2_backward;

	motor_1.set_pwm = motor1_set_pwm;
	motor_2.set_pwm = motor2_set_pwm;

//	motor_1.obstacle = rc_other1_obstacle;
//	motor_1.phase_a = 0;
//	motor_1.threshold = 10;
//	motor_1.initial_task = 0;
//	motor_1.phase_counter = 0;
//	motor_1.angle_phase_counter = 0;
//	motor_1.phase_difference = 0;
//	motor_1.scaled_difference = 0;
//	motor_1.phase_stop = 3500;
//	motor_1.task_phase = 0;
//	motor_1.scale_red = 0.8;
//	motor_1.speed_limit_min = 0.4;
//	motor_1.speed_limit_max = 1.0;
	motor_1.task = 0;

	motor_1.speed_ramp_rate = 20000;
	motor_1.speed_filter_alpha = 0.9f;
	motor_1.settle_speed = 1;
	motor_1.settle_angle = 1;
	motor_1.kd_speed = 0;
	motor_1.final_speed_max = 35000;
	motor_1.angle_final_threshold = 10;

//	motor_1.direction = 1;
//
//	motor_2.obstacle = rc_other2_obstacle;
//	motor_2.phase_a = 0;
//	motor_2.threshold = 10;
//	motor_2.initial_task = 0;
//	motor_2.phase_counter = 0;
//	motor_2.angle_phase_counter = 0;
//	motor_2.phase_difference = 0;
//	motor_2.scaled_difference = 0;
//	motor_2.phase_stop = 3500;
//	motor_2.task_phase = 0;
//	motor_2.scale_red = 0.8;
//	motor_2.speed_limit_min = 0.4;
//	motor_2.speed_limit_max = 1.0;
	motor_2.task = 0;

	motor_2.speed_ramp_rate = 20000;
	motor_2.speed_filter_alpha = 0.9f;
	motor_2.settle_speed = 1;
	motor_2.settle_angle = 1;
	motor_2.kd_speed = 0;
	motor_2.final_speed_max = 35000;
	motor_2.angle_final_threshold = 10;

//	motor_2.direction = 1;
}

//void phase_angle_support(struct Motor *motor, int task) {
//	if (motor->phase_a == 0) {
//		if (task != motor->prev_task) {
//			motor->phase_a = 1;
//			motor->speed_limit = motor->speed_limit_min;
//			motor->angle_phase_counter++;
//			if (task > motor->angle_reduction) {
//				motor->initial_task = task - 360;
//				motor->direction = 1;
//			}
//			if (task < motor->angle_reduction) {
//				motor->initial_task = task + 360;
//				motor->direction = 0;
//			}
//		}
//		motor->prev_task = task;
//	}
//	if (motor->phase_a == 1 && motor->direction == 1) {
//		if (motor->angle_reduction < motor->initial_task + 30) {
//			motor->speed_limit = motor->speed_limit_min;
//		}
//		if (motor->angle_reduction
//				>= (motor->initial_task + 30 - motor->threshold)) {
//			motor->phase_a = 2;
//		}
//	}
//	if (motor->phase_a == 2 && motor->direction == 1) {
//		motor->speed_limit = motor->speed_limit_min
//				+ motor->speed_limit_min * 0.1;
//		if (motor->speed_limit >= motor->speed_limit_max + 0.3) {
//			motor->speed_limit = motor->speed_limit_max + 0.3;
//		}
//		if (motor->angle_reduction
//				>= (motor->initial_task + 180 - (motor->obstacle - 30)
//						- motor->threshold)) {
//			motor->phase_a = 3;
//		}
//	}
//	if (motor->phase_a == 3 && motor->direction == 1) {
//
//		if (motor->obstacle - 30 == 0) {
//			motor->phase_difference = abs(
//					motor->angle_reduction - (motor->initial_task + 360));
//			motor->speed_reduction = pow(
//					(1 - (motor->phase_difference / 180.0)), 3);
//
//			motor->speed_limit = motor->speed_limit_max
//					- motor->speed_reduction
//							* (motor->speed_limit_max - motor->speed_limit_min)
//					- motor->scale_red;
//
//			if (motor->speed_limit >= motor->speed_limit_max) {
//				motor->speed_limit = motor->speed_limit_max;
//			} else if (motor->speed_limit <= motor->speed_limit_min) {
//				motor->speed_limit = motor->speed_limit_min;
//			}
//			if (motor->angle_reduction
//					>= (motor->initial_task + 360 - motor->threshold)) {
//				motor->phase_a = 4;
//			}
//		} else {
//			motor->phase_difference = abs(
//					motor->angle_reduction
//							- (motor->initial_task + 360
//									- (motor->obstacle - 30)));
//			motor->speed_reduction = pow(
//					(1 - (motor->phase_difference / 180.0)), 3);
//			motor->speed_limit = motor->speed_limit_max
//					- motor->speed_reduction
//							* (motor->speed_limit_max - motor->speed_limit_min)
//					- motor->scale_red;
//			if (motor->speed_limit >= motor->speed_limit_max) {
//				motor->speed_limit = motor->speed_limit_max;
//			} else if (motor->speed_limit <= motor->speed_limit_min) {
//				motor->speed_limit = motor->speed_limit_min;
//			}
//			if (motor->angle_reduction
//					>= (motor->initial_task + 360 - (motor->obstacle - 30)
//							- motor->threshold)) {
//				motor->phase_a = 5;
//			}
//		}
//
//	}
//	if (motor->phase_a == 5 && motor->direction == 1) {
//		if (motor->angle_reduction < motor->initial_task + 360) {
//			motor->speed_limit = motor->speed_limit_min + 0.2;
//		}
//		if (motor->angle_reduction
//				>= (motor->initial_task + 360 - motor->threshold)) {
//			motor->phase_a = 4;
//		}
//	}
//	if (motor->phase_a == 4 && motor->direction == 1) {
//		motor->speed_limit = 1;
//		motor->phase_a = 0;
//	}
//
////direction --
//	if (motor->phase_a == 1 && motor->direction == 0) {
//		if (motor->angle_reduction > motor->initial_task - 30) {
//			motor->speed_limit = motor->speed_limit_min;
//		}
//		if (motor->angle_reduction
//				<= (motor->initial_task - 30 + motor->threshold)) {
//			motor->phase_a = 2;
//			motor->phase_start = 0;
//		}
//	}
//	if (motor->phase_a == 2 && motor->direction == 0) {
//		motor->speed_limit = motor->speed_limit_min
//				+ motor->speed_limit_min * 0.1;
//		if (motor->speed_limit >= motor->speed_limit_max + 0.3) {
//			motor->speed_limit = motor->speed_limit_max + 0.3;
//		}
//		if (motor->angle_reduction
//				<= (motor->initial_task - 180 + (motor->obstacle - 30)
//						+ motor->threshold)) {
//			motor->phase_a = 3;
//		}
//	}
//
//	if (motor->phase_a == 3 && motor->direction == 0) {
//		if (motor->obstacle - 30 == 0) {
//			motor->phase_difference = abs(
//					motor->angle_reduction + (motor->initial_task - 360));
//			motor->speed_reduction = pow(
//					(1 - (motor->phase_difference / 180.0)), 3);
//
//			motor->speed_limit = (motor->speed_limit_max
//					- motor->speed_reduction
//							* (motor->speed_limit_max - motor->speed_limit_min)
//					- motor->scale_red) / 2;
//
//			if (motor->speed_limit >= motor->speed_limit_max) {
//				motor->speed_limit = motor->speed_limit_max;
//			} else if (motor->speed_limit <= motor->speed_limit_min) {
//				motor->speed_limit = motor->speed_limit_min;
//			}
//			if (motor->angle_reduction
//					<= (motor->initial_task - 360 + motor->threshold)) {
//				motor->phase_a = 4;
//			}
//		} else {
//			motor->phase_difference = abs(
//					motor->angle_reduction
//							- (motor->initial_task - 360
//									+ (motor->obstacle - 30)));
//			motor->speed_reduction = pow(
//					(1 - (motor->phase_difference / 180.0)), 3);
//			motor->speed_limit = (motor->speed_limit_max
//					- motor->speed_reduction
//							* (motor->speed_limit_max - motor->speed_limit_min)
//					- motor->scale_red) / 2;
//			if (motor->speed_limit >= motor->speed_limit_max) {
//				motor->speed_limit = motor->speed_limit_max;
//			} else if (motor->speed_limit <= motor->speed_limit_min) {
//				motor->speed_limit = motor->speed_limit_min;
//			}
//			if (motor->angle_reduction
//					<= (motor->initial_task - 360 + (motor->obstacle - 30)
//							+ motor->threshold)) {
//				motor->phase_a = 5;
//			}
//		}
//
//	}
//	if (motor->phase_a == 5 && motor->direction == 0) {
//		if (motor->angle_reduction > motor->initial_task - 360) {
//			motor->speed_limit = motor->speed_limit_min + 0.2;
//		}
//		if (motor->angle_reduction
//				<= (motor->initial_task - 360 + motor->threshold)) {
//			motor->phase_a = 4;
//		}
//	}
//	if (motor->phase_a == 4 && motor->direction == 0) {
//		motor->speed_limit = 1;
//		motor->phase_a = 0;
//	}
//}

void koef_pid() {
	motor_1.speed_limit = 1;
	motor_2.speed_limit = 1;

	motor_1.limit_speed = 4.0;
	motor_2.limit_speed = 4.0;

	pwm_step = 20;
	percentage = 0.3;

	motor_1.hyst_error = 27;
	motor_2.hyst_error = 27;

	motor_1.kp_angle = 4.0;
	motor_1.ki_angle = 4.0;
	motor_1.kd_angle = 0.01;
	motor_1.kp_speed = 0.005;
	motor_1.ki_speed = 0.005;
	motor_1.kp_current = 0.05;
	motor_1.ki_current = 5.0;

	motor_2.kp_angle = 4.0;
	motor_2.ki_angle = 4.0;
	motor_2.kd_angle = 0.01;
	motor_2.kp_speed = 0.005;
	motor_2.ki_speed = 0.005;
	motor_2.kp_current = 0.05;
	motor_2.ki_current = 5.0;
}


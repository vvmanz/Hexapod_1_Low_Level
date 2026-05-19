#include "main.h"

int32_t ConvertBytesToInt32(uint8_t *data) {
	return (int32_t) ((data[0] << 24) | (data[1] << 16) | (data[2] << 8)
			| data[3]);
}

int32_t ConvertBytesToInt32_LittleEndian(uint8_t *data) {
	return (int32_t) ((data[3] << 24) | (data[2] << 16) | (data[1] << 8)
			| data[0]);
}

void DecodePart1() {
	rc_angle1 = ConvertBytesToInt32_LittleEndian(RxAngles);
	rc_angle2 = ConvertBytesToInt32_LittleEndian(&RxAngles[4]);

	rc_kp_angle1 = ConvertBytesToInt32_LittleEndian(RxCoeffs1);
	rc_kp_angle2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs1[4]);

	rc_kd_angle1 = ConvertBytesToInt32_LittleEndian(RxCoeffs3);
	rc_kd_angle2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs3[4]);

	rc_ki_speed1 = ConvertBytesToInt32_LittleEndian(RxCoeffs5);
	rc_ki_speed2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs5[4]);

	rc_ki_curr1 = ConvertBytesToInt32_LittleEndian(RxCoeffs7);
	rc_ki_curr2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs7[4]);

	motor_1.kp_angle = (float) rc_kp_angle1 / 1000.0f;
	motor_2.kp_angle = (float) rc_kp_angle2 / 1000.0f;

	motor_1.kd_angle = (float) rc_kd_angle1 / 1000.0f;
	motor_2.kd_angle = (float) rc_kd_angle2 / 1000.0f;

	motor_1.ki_speed = (float) rc_ki_speed1 / 1000.0f;
	motor_2.ki_speed = (float) rc_ki_speed2 / 1000.0f;

	motor_1.ki_current = (float) rc_ki_curr1 / 1000.0f;
	motor_2.ki_current = (float) rc_ki_curr2 / 1000.0f;
}
void DecodePart2() {
	rc_other1_obstacle = RxOther[0];
	rc_other2_obstacle = RxOther[1];

	rc_other1_flag_phase = RxOther[2] & 0x01;  // бит 0
	rc_other2_flag_phase = (RxOther[2] >> 1) & 0x01;   // бит 1
	rc_other_set_to_zero = (RxOther[2] >> 2) & 0x01;   // бит 2

	rc_ki_angle1 = ConvertBytesToInt32_LittleEndian(RxCoeffs2);
	rc_ki_angle2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs2[4]);

	rc_kp_speed1 = ConvertBytesToInt32_LittleEndian(RxCoeffs4);
	rc_kp_speed2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs4[4]);

	rc_kp_curr1 = ConvertBytesToInt32_LittleEndian(RxCoeffs6);
	rc_kp_curr2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs6[4]);

	motor_1.ki_angle = (float) rc_ki_angle1 / 1000.0f;
	motor_2.ki_angle = (float) rc_ki_angle2 / 1000.0f;

	motor_1.kp_speed = (float) rc_kp_speed1 / 1000.0f;
	motor_2.kp_speed = (float) rc_kp_speed2 / 1000.0f;

	motor_1.kp_current = (float) rc_kp_curr1 / 1000.0f;
	motor_2.kp_current = (float) rc_kp_curr2 / 1000.0f;
}

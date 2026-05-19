#include "main.h"


float map_angle(float x, float in_min, float in_max, float out_min,
		float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void Read(struct Motor *motor) {
	motor->A = 0;
	motor->B = 0;
	for (int i = 0; i < 100; i++) {
		HAL_Delay(1);
		if (motor == &motor_1) {
			motor->A = motor->A + adcValBuf[3];
			motor->B = motor->B + adcValBuf[2];
		}
		if (motor == &motor_2) {
			motor->A = motor->A + adcValBuf[4];
			motor->B = motor->B + adcValBuf[5];
		}
	}
	motor->A = motor->A / 100;
	motor->B = motor->B / 100;

	if (motor == &motor_1) {
		motor->A = adcValBuf[3];
		motor->B = adcValBuf[2];
	}
	if (motor == &motor_2) {
		motor->A = adcValBuf[4];
		motor->B = adcValBuf[5];
	}
}

void BlindZoneDesignation(struct Motor *motor) {
	int currDelta = 0;
	motor->maxA = 0;
	motor->maxB = 0;
	motor->minA = 10000;
	motor->minB = 10000;
	for (int i = Z; i < M + Z; i++) {
		if (motor->CMap[i][0] > motor->maxA) {
			motor->maxA = motor->CMap[i][0];
			motor->imaxA = i;
		}
		if (motor->CMap[i][0] < motor->minA) {
			motor->minA = motor->CMap[i][0];
			motor->iminA = i;
		}
		if (motor->CMap[i][1] > motor->maxB) {
			motor->maxB = motor->CMap[i][1];
			motor->imaxB = i;
		}
		if (motor->CMap[i][1] < motor->minB) {
			motor->minB = motor->CMap[i][1];
			motor->iminB = i;
		}
	}
	printf("Max A: %d;\nMax B: %d;\nMin A: %d;\nMin B: %d.\n", motor->maxA,
			motor->maxB, motor->minA, motor->minB);
	printf(
			"Max A angle: %d;\nMax B angle: %d;\nMin A angle: %d;\nMin B angle: %d.\n",
			motor->imaxA, motor->imaxB, motor->iminA, motor->iminB);

//Верхняя граница A
	int i = motor->imaxA;
	currDelta = 0;
	while (currDelta <= Delta) {
		currDelta = abs(motor->CMap[i][0] - motor->CMap[i - 1][0]);
		i--;
	}
	motor->AHigh[0] = i + 1;
	i = motor->imaxA;
	currDelta = 0;
	while (currDelta <= Delta) {
		currDelta = abs(motor->CMap[i][0] - motor->CMap[i + 1][0]);
		i++;
	}
	motor->AHigh[1] = i - 1;

//Нижняя граница A
	i = motor->iminA;
	currDelta = 0;
	while (currDelta <= Delta) {
		currDelta = abs(motor->CMap[i][0] - motor->CMap[i - 1][0]);
		i--;
	}
	motor->ALow[0] = i + 1;
	i = motor->iminA;
	currDelta = 0;
	while (currDelta <= Delta) {
		currDelta = abs(motor->CMap[i][0] - motor->CMap[i + 1][0]);
		i++;
	}
	motor->ALow[1] = i - 1;

//Верхняя граница B
	i = motor->imaxB;
	currDelta = 0;
	while (currDelta <= Delta) {
		currDelta = abs(motor->CMap[i][1] - motor->CMap[i - 1][1]);
		i--;
	}
	motor->BHigh[0] = i + 1;
	i = motor->imaxB;
	currDelta = 0;
	while (currDelta <= Delta) {
		currDelta = abs(motor->CMap[i][1] - motor->CMap[i + 1][1]);
		i++;
	}
	motor->BHigh[1] = i - 1;

//Нижняя граница B
	i = motor->iminB;
	currDelta = 0;
	while (currDelta <= Delta) {
		currDelta = abs(motor->CMap[i][1] - motor->CMap[i - 1][1]);
		i--;
	}
	motor->BLow[0] = i + 1;
	i = motor->iminB;
	currDelta = 0;
	while (currDelta <= Delta) {
		currDelta = abs(motor->CMap[i][1] - motor->CMap[i + 1][1]);
		i++;
	}
	motor->BLow[1] = i - 1;
	printf("\nHigh limit A: from %d, to %d;\n", motor->AHigh[0],
			motor->AHigh[1]);
	printf("Low limit A: from %d, to %d;\n", motor->ALow[0], motor->ALow[1]);
	printf("High limit B: from %d, to %d;\n", motor->BHigh[0], motor->BHigh[1]);
	printf("Low limit B: from %d, to %d.\n", motor->BLow[0], motor->BLow[1]);
}
float Result_prev;
void CurrentPositionCalculation(struct Motor *motor) {
	Read(motor);
	if ((motor->A > motor->CMap[motor->AHigh[0]][0])
			|| (motor->A > motor->CMap[motor->AHigh[1]][0])) {
		for (int i = motor->AHigh[0] - 1; i < motor->AHigh[1] + 1; i++) {
			if (((motor->B >= motor->CMap[i][1])
					&& (motor->B <= motor->CMap[i + 1][1]))
					|| ((motor->B <= motor->CMap[i][1])
							&& (motor->B >= motor->CMap[i + 1][1]))) {
				if (i > (M + Z - 1)) {
					motor->Result = (i - Z) * (360 / M) - 360
							+ map(motor->B, motor->CMap[i][1],
									motor->CMap[i + 1][1], 0, 50) * 0.1;
//					motor->Result_prev = motor->Result;
				} else {
					motor->Result = (i - Z) * (360 / M)
							+ map(motor->B, motor->CMap[i][1],
									motor->CMap[i + 1][1], 0, 50) * 0.1;
//					motor->Result_prev = motor->Result;
				}
				return;
			}
		}
	} else if ((motor->A < motor->CMap[motor->ALow[0]][0])
			|| (motor->A < motor->CMap[motor->ALow[1]][0])) {
		for (int i = motor->ALow[0] - 1; i < motor->ALow[1] + 1; i++) {
			if (((motor->B >= motor->CMap[i][1])
					&& (motor->B <= motor->CMap[i + 1][1]))
					|| ((motor->B <= motor->CMap[i][1])
							&& (motor->B >= motor->CMap[i + 1][1]))) {
				if (i > (M + Z - 1)) {
					motor->Result = (i - Z) * (360 / M) - 360
							+ map(motor->B, motor->CMap[i][1],
									motor->CMap[i + 1][1], 0, 50) * 0.1;
//					motor->Result_prev = motor->Result;
				} else {
					motor->Result = (i - Z) * (360 / M)
							+ map(motor->B, motor->CMap[i][1],
									motor->CMap[i + 1][1], 0, 50) * 0.1;
//					motor->Result_prev = motor->Result;
				}
				return;
			}
		}
	} else if ((motor->B > motor->CMap[motor->BHigh[0]][1])
			|| (motor->B > motor->CMap[motor->BHigh[1]][1])) {
		for (int i = motor->BHigh[0] - 1; i < motor->BHigh[1] + 1; i++) {
			if (((motor->A >= motor->CMap[i][0])
					&& (motor->A <= motor->CMap[i + 1][0]))
					|| ((motor->A <= motor->CMap[i][0])
							&& (motor->A >= motor->CMap[i + 1][0]))) {
				if (i > (M + Z - 1)) {
					motor->Result = (i - Z) * (360 / M) - 360
							+ map(motor->A, motor->CMap[i][0],
									motor->CMap[i + 1][0], 0, 50) * 0.1;
//					motor->Result_prev = motor->Result;
				} else {
					motor->Result = (i - Z) * (360 / M)
							+ map(motor->A, motor->CMap[i][0],
									motor->CMap[i + 1][0], 0, 50) * 0.1;
//					motor->Result_prev = motor->Result;
				}
				return;
			}
		}
	} else if ((motor->B < motor->CMap[motor->BLow[0]][1])
			|| (motor->B < motor->CMap[motor->BLow[1]][1])) {
		for (int i = motor->BLow[0] - 1; i < motor->BLow[1] + 1; i++) {
			if (((motor->A >= motor->CMap[i][0])
					&& (motor->A <= motor->CMap[i + 1][0]))
					|| ((motor->A <= motor->CMap[i][0])
							&& (motor->A >= motor->CMap[i + 1][0]))) {
				if (i > (M + Z - 1)) {
					motor->Result = (i - Z) * (360 / M) - 360
							+ map(motor->A, motor->CMap[i][0],
									motor->CMap[i + 1][0], 0, 50) * 0.1;
//					motor->Result_prev = motor->Result;
				} else {
					motor->Result = (i - Z) * (360 / M)
							+ map(motor->A, motor->CMap[i][0],
									motor->CMap[i + 1][0], 0, 50) * 0.1;
//					motor->Result_prev = motor->Result;
				}
				return;
			}
		}
	} else {
		for (int iA = Z; iA < M + Z; iA++) {
			if ((((motor->A >= motor->CMap[iA][0])
					&& (motor->A <= motor->CMap[iA + 1][0]))
					|| ((motor->A <= motor->CMap[iA][0])
							&& (motor->A >= motor->CMap[iA + 1][0])))
					&& (((motor->B >= motor->CMap[iA - 1][1])
							&& (motor->B <= motor->CMap[iA + 2][1]))
							|| ((motor->B <= motor->CMap[iA - 1][1])
									&& (motor->B >= motor->CMap[iA + 2][1])))) {
				for (int iB = (iA - 1); iB < iA + 2; iB++) {
					if (((motor->B >= motor->CMap[iB][1])
							&& (motor->B <= motor->CMap[iB + 1][1]))
							|| ((motor->B <= motor->CMap[iB][1])
									&& (motor->B >= motor->CMap[iB + 1][1]))) {
						if (abs(motor->CMap[iA][0] - motor->CMap[iA + 1][0])
								> abs(
										motor->CMap[iA][0]
												- motor->CMap[iA + 1][0])) {
							motor->Result = (iA - Z) * (360 / M)
									+ map(motor->A, motor->CMap[iA][0],
											motor->CMap[iA + 1][0], 0, 50)
											* 0.1;
//							motor->Result_prev = motor->Result;
							return;
						} else {
							motor->Result = (iB - Z) * (360 / M)
									+ map(motor->B, motor->CMap[iB][1],
											motor->CMap[iB + 1][1], 0, 50)
											* 0.1;
//							motor->Result_prev = motor->Result;
							return;
						}
					}
				}
			}
		}
	}
	motor->Result = 999;
//	motor->Result = motor->Result_prev;
}

int map(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Calibration(struct Motor *motor) {
	printf("Calibration.\n");
	printf("Calibration points: %d, degrees between points: %d.\n", M, 360 / M);
	for (int i = Z; i < M + Z; i++) {
		printf("Set angle: %d\260\n", (i - Z) * (360 / M));
		Read(motor);
		motor->CMap[i][0] = motor->A;
		motor->CMap[i][1] = motor->B;
		HAL_Delay(250);
		printf("Data recorded. A: %" PRId32 ",     B: %" PRId32 "\n", motor->A,
				motor->B);
		HAL_Delay(250);
	};
	printf("Calibration completed successfully.\n");
	for (int i = 0; i < Z; i++) {
		motor->CMap[i][0] = motor->CMap[i + M][0];
		motor->CMap[i][1] = motor->CMap[i + M][1];
	};
	for (int i = M + Z; i < M + Z * 2; i++) {
		motor->CMap[i][0] = motor->CMap[i - M][0];
		motor->CMap[i][1] = motor->CMap[i - M][1];
	};
	printf("Additional sections of the calibration table are completed.\n");
}

void CMapPrint(struct Motor *motor) {
	printf("Calibration map:\n{");
	for (int i = 0; i < M + Z * 2; i++) {
		printf("{%"PRId32",%"PRId32"}", motor->CMap[i][0], motor->CMap[i][1]);
		if (i < ((M + Z * 2) - 1)) {
			printf(", ");
			if ((i != 0) && ((i + 1) % 5 == 0)) {
				printf("\n");
			}
		}
		HAL_Delay(1);
	}
	printf("};\n");
}
void calibration_maps() {
	if (number_bridge == 1) {
		int values[92][2] = { { 2408, 3601 }, { 2517, 3601 }, { 2653, 3608 }, {
				2749, 3597 }, { 2896, 3602 }, { 3048, 3592 }, { 3209, 3568 }, {
				3274, 3493 }, { 3312, 3440 }, { 3328, 3366 }, { 3376, 3282 }, {
				3408, 3242 }, { 3464, 3168 }, { 3524, 3094 }, { 3520, 2966 }, {
				3568, 2787 }, { 3568, 2612 }, { 3600, 2391 }, { 3636, 2266 }, {
				3647, 2096 }, { 3632, 1947 }, { 3635, 1792 }, { 3618, 1578 }, {
				3640, 1334 }, { 3611, 1187 }, { 3558, 1034 }, { 3496, 987 }, {
				3441, 861 }, { 3377, 812 }, { 3337, 729 }, { 3276, 593 }, {
				3221, 425 }, { 3123, 317 }, { 2984, 294 }, { 2812, 274 }, {
				2644, 260 }, { 2469, 252 }, { 2357, 251 }, { 2180, 255 }, {
				2064, 263 }, { 1892, 282 }, { 1693, 324 }, { 1509, 437 }, {
				1349, 651 }, { 1262, 752 }, { 1194, 893 }, { 1092, 1066 }, {
				1040, 1232 }, { 920, 1427 }, { 786, 1577 }, { 560, 1701 }, {
				390, 1816 }, { 316, 1976 }, { 285, 2180 }, { 268, 2332 }, { 262,
				2449 }, { 261, 2578 }, { 272, 2670 }, { 303, 2865 },
				{ 424, 3036 }, { 638, 3174 }, { 853, 3227 }, { 977, 3245 }, {
						1072, 3317 }, { 1224, 3341 }, { 1380, 3384 }, { 1524,
						3474 }, { 1688, 3491 }, { 1808, 3526 }, { 1925, 3533 },
				{ 2052, 3551 }, { 2251, 3580 }, { 2408, 3601 }, { 2517, 3601 },
				{ 2653, 3608 }, { 2749, 3597 }, { 2896, 3602 }, { 3048, 3592 },
				{ 3209, 3568 }, { 3274, 3493 }, { 3312, 3440 }, { 3328, 3366 },
				{ 3376, 3282 }, { 3408, 3242 }, { 3464, 3168 }, { 3524, 3094 },
				{ 3520, 2966 }, { 3568, 2787 }, { 3568, 2612 }, { 3600, 2391 } };

		for (int i = 0; i < 92; i++) {
			motor_1.CMap[i][0] = values[i][0];
			motor_1.CMap[i][1] = values[i][1];
		}
	}
}

#include "main.h"
#include "string.h"

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	flag_of_tx = 1;
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
	flag_of_tx = 1;
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
	flag_of_tx = 1;
}

void send_info_CAN() {
	if (flag_of_rx == 1) {
		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
			pack_CAN_data();
			flag_of_rx = 0;
			if (queue == 0) {
				TxDataAngles[0] = (uint8_t) ((int32_t) (motor_1.angle_reduction
						* 100.0f) >> 24);
				TxDataAngles[1] = (uint8_t) ((int32_t) (motor_1.angle_reduction
						* 100.0f) >> 16);
				TxDataAngles[2] = (uint8_t) ((int32_t) (motor_1.angle_reduction
						* 100.0f) >> 8);
				TxDataAngles[3] = (uint8_t) ((int32_t) (motor_1.angle_reduction
						* 100.0f));

				TxDataAngles[4] = (uint8_t) ((int32_t) (motor_2.angle_reduction
						* 100.0f) >> 24);
				TxDataAngles[5] = (uint8_t) ((int32_t) (motor_2.angle_reduction
						* 100.0f) >> 16);
				TxDataAngles[6] = (uint8_t) ((int32_t) (motor_2.angle_reduction
						* 100.0f) >> 8);
				TxDataAngles[7] = (uint8_t) ((int32_t) (motor_2.angle_reduction
						* 100.0f));

				HAL_CAN_AddTxMessage(&hcan, &TxAngles, TxDataAngles, &mailbox);
				queue = 1;
			} else if (queue == 1) {
				TxDataSpeeds[0] = (uint8_t) ((int32_t) (motor_1.speed * 100.0f)
						>> 24);
				TxDataSpeeds[1] = (uint8_t) ((int32_t) (motor_1.speed * 100.0f)
						>> 16);
				TxDataSpeeds[2] = (uint8_t) ((int32_t) (motor_1.speed * 100.0f)
						>> 8);
				TxDataSpeeds[3] =
						(uint8_t) ((int32_t) (motor_1.speed * 100.0f));

				TxDataSpeeds[4] = (uint8_t) ((int32_t) (motor_2.speed * 100.0f)
						>> 24);
				TxDataSpeeds[5] = (uint8_t) ((int32_t) (motor_2.speed * 100.0f)
						>> 16);
				TxDataSpeeds[6] = (uint8_t) ((int32_t) (motor_2.speed * 100.0f)
						>> 8);
				TxDataSpeeds[7] =
						(uint8_t) ((int32_t) (motor_2.speed * 100.0f));

				HAL_CAN_AddTxMessage(&hcan, &TxSpeeds, TxDataSpeeds, &mailbox);
				queue = 2;
			} else if (queue == 2) {
				TxDataCurrents[0] = (uint8_t) ((int32_t) (motor_1.current
						* 10000.0f) >> 24);
				TxDataCurrents[1] = (uint8_t) ((int32_t) (motor_1.current
						* 10000.0f) >> 16);
				TxDataCurrents[2] = (uint8_t) ((int32_t) (motor_1.current
						* 10000.0f) >> 8);
				TxDataCurrents[3] = (uint8_t) ((int32_t) (motor_1.current
						* 10000.0f));

				TxDataCurrents[4] = (uint8_t) ((int32_t) (motor_2.current
						* 10000.0f) >> 24);
				TxDataCurrents[5] = (uint8_t) ((int32_t) (motor_2.current
						* 10000.0f) >> 16);
				TxDataCurrents[6] = (uint8_t) ((int32_t) (motor_2.current
						* 10000.0f) >> 8);
				TxDataCurrents[7] = (uint8_t) ((int32_t) (motor_2.current
						* 10000.0f));

				HAL_CAN_AddTxMessage(&hcan, &TxCurrents, TxDataCurrents,
						&mailbox);
				queue = 0;
			}
		}

	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	error_can++;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
		if (number_bridge == 1) {
			if (RxHeader.StdId == 0x0200) {
				memcpy(RxAngles, RxData, sizeof(RxAngles));
				rc_angle1 = ConvertBytesToInt32_LittleEndian(RxAngles);
				rc_angle2 = ConvertBytesToInt32_LittleEndian(&RxAngles[4]);
				flag_of_rx = 1;
			}
			if (RxHeader.StdId == 0x0202) {
				memcpy(RxCoeffs1, RxData, sizeof(RxCoeffs1));
				rc_kp_angle1 = ConvertBytesToInt32_LittleEndian(RxCoeffs1);
				rc_kp_angle2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs1[4]);
				flag_of_rx = 1;
			}
			if (RxHeader.StdId == 0x0204) {
				memcpy(RxCoeffs3, RxData, sizeof(RxCoeffs3));
				rc_kd_angle1 = ConvertBytesToInt32_LittleEndian(RxCoeffs3);
				rc_kd_angle2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs3[4]);
				flag_of_rx = 1;
			}
			if (RxHeader.StdId == 0x0206) {
				memcpy(RxCoeffs5, RxData, sizeof(RxCoeffs5));
				rc_ki_speed1 = ConvertBytesToInt32_LittleEndian(RxCoeffs5);
				rc_ki_speed2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs5[4]);
				flag_of_rx = 1;
			}
			if (RxHeader.StdId == 0x0208) {
				memcpy(RxCoeffs7, RxData, sizeof(RxCoeffs7));
				rc_ki_curr1 = ConvertBytesToInt32_LittleEndian(RxCoeffs7);
				rc_ki_curr2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs7[4]);
				flag_of_rx = 1;
			}
		}
		if (number_bridge == 2) {
			if (RxHeader.StdId == 0x0300) {
				memcpy(RxAngles, RxData, sizeof(RxAngles));
				rc_angle1 = ConvertBytesToInt32_LittleEndian(RxAngles);
				rc_angle2 = ConvertBytesToInt32_LittleEndian(&RxAngles[4]);
				flag_of_rx = 1;
			}
			if (RxHeader.StdId == 0x0302) {
				memcpy(RxCoeffs1, RxData, sizeof(RxCoeffs1));
				rc_kp_angle1 = ConvertBytesToInt32_LittleEndian(RxCoeffs1);
				rc_kp_angle2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs1[4]);
				flag_of_rx = 1;
			}
			if (RxHeader.StdId == 0x0304) {
				memcpy(RxCoeffs3, RxData, sizeof(RxCoeffs3));
				rc_kd_angle1 = ConvertBytesToInt32_LittleEndian(RxCoeffs3);
				rc_kd_angle2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs3[4]);
				flag_of_rx = 1;
			}
			if (RxHeader.StdId == 0x0306) {
				memcpy(RxCoeffs5, RxData, sizeof(RxCoeffs5));
				rc_ki_speed1 = ConvertBytesToInt32_LittleEndian(RxCoeffs5);
				rc_ki_speed2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs5[4]);
				flag_of_rx = 1;
			}
			if (RxHeader.StdId == 0x0308) {
				memcpy(RxCoeffs7, RxData, sizeof(RxCoeffs7));
				rc_ki_curr1 = ConvertBytesToInt32_LittleEndian(RxCoeffs7);
				rc_ki_curr2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs7[4]);
				flag_of_rx = 1;
			}
		}
		if (number_bridge == 3) {
			if (RxHeader.StdId == 0x0400) {
				memcpy(RxAngles, RxData, sizeof(RxAngles));
				rc_angle1 = ConvertBytesToInt32_LittleEndian(RxAngles);
				rc_angle2 = ConvertBytesToInt32_LittleEndian(&RxAngles[4]);
				flag_of_rx = 1;
			}
			if (RxHeader.StdId == 0x0402) {
				memcpy(RxCoeffs1, RxData, sizeof(RxCoeffs1));
				rc_kp_angle1 = ConvertBytesToInt32_LittleEndian(RxCoeffs1);
				rc_kp_angle2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs1[4]);
				flag_of_rx = 1;
			}
			if (RxHeader.StdId == 0x0404) {
				memcpy(RxCoeffs3, RxData, sizeof(RxCoeffs3));
				rc_kd_angle1 = ConvertBytesToInt32_LittleEndian(RxCoeffs3);
				rc_kd_angle2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs3[4]);
				flag_of_rx = 1;
			}
			if (RxHeader.StdId == 0x0406) {
				memcpy(RxCoeffs5, RxData, sizeof(RxCoeffs5));
				rc_ki_speed1 = ConvertBytesToInt32_LittleEndian(RxCoeffs5);
				rc_ki_speed2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs5[4]);
				flag_of_rx = 1;
			}
			if (RxHeader.StdId == 0x0408) {
				memcpy(RxCoeffs7, RxData, sizeof(RxCoeffs7));
				rc_ki_curr1 = ConvertBytesToInt32_LittleEndian(RxCoeffs7);
				rc_ki_curr2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs7[4]);
				flag_of_rx = 1;
			}
		}
		DecodePart1();
		received_call++;
	}
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader1, RxData1)
			== HAL_OK) {
		if (number_bridge == 1) {
			if (RxHeader1.StdId == 0x0201) {
				memcpy(RxOther, RxData1, sizeof(RxOther));
				rc_other1_obstacle = RxOther[0];
				rc_other2_obstacle = RxOther[1];
				rc_other1_flag_phase = RxOther[2] & 0x01;  // бит 0
				rc_other2_flag_phase = (RxOther[2] >> 1) & 0x01;   // бит 1
				rc_other_set_to_zero = (RxOther[2] >> 2) & 0x01;   // бит 2
				rc_other1_system_type = (RxOther[2] >> 3) & 0x07;  //биты 3 -5
				rc_other2_system_type = ((RxOther[3] & 0x01) << 2)
						| ((RxOther[2] & 0xC0) >> 6); // бит 6-7 и бит 0 rxother3
				rc_speed_limit1 = RxOther[4]; // скоростной коэффициент  регулятора скорости 1
				rc_speed_limit2 = RxOther[5]; // скоростной коэффициент  регулятора скорости 2
				flag_of_rx = 1;
			}
			if (RxHeader1.StdId == 0x0203) {
				memcpy(RxCoeffs2, RxData1, sizeof(RxCoeffs2));
				rc_ki_angle1 = ConvertBytesToInt32_LittleEndian(RxCoeffs2);
				rc_ki_angle2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs2[4]);
				flag_of_rx = 1;
			}
			if (RxHeader1.StdId == 0x0205) {
				memcpy(RxCoeffs4, RxData1, sizeof(RxCoeffs4));
				rc_kp_speed1 = ConvertBytesToInt32_LittleEndian(RxCoeffs4);
				rc_kp_speed2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs4[4]);
				flag_of_rx = 1;
			}
			if (RxHeader1.StdId == 0x0207) {
				memcpy(RxCoeffs6, RxData1, sizeof(RxCoeffs6));
				rc_kp_curr1 = ConvertBytesToInt32_LittleEndian(RxCoeffs6);
				rc_kp_curr2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs6[4]);
				flag_of_rx = 1;
			}
		}
		if (number_bridge == 2) {
			if (RxHeader1.StdId == 0x0301) {
				memcpy(RxOther, RxData1, sizeof(RxOther));
				rc_other1_obstacle = RxOther[0];
				rc_other2_obstacle = RxOther[1];
				rc_other1_flag_phase = RxOther[2] & 0x01;  // бит 0
				rc_other2_flag_phase = (RxOther[2] >> 1) & 0x01;   // бит 1
				rc_other_set_to_zero = (RxOther[2] >> 2) & 0x01;   // бит 2
				rc_other1_system_type = (RxOther[2] >> 3) & 0x07;  //биты 3 -5
				rc_other2_system_type = ((RxOther[3] & 0x01) << 2)
						| ((RxOther[2] & 0xC0) >> 6); // бит 6-7 и бит 0 rxother3
				rc_speed_limit1 = RxOther[4]; // скоростной коэффициент  регулятора скорости 1
				rc_speed_limit2 = RxOther[5]; // скоростной коэффициент  регулятора скорости 2
				flag_of_rx = 1;
			}
			if (RxHeader1.StdId == 0x0303) {
				memcpy(RxCoeffs2, RxData1, sizeof(RxCoeffs2));
				rc_ki_angle1 = ConvertBytesToInt32_LittleEndian(RxCoeffs2);
				rc_ki_angle2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs2[4]);
				flag_of_rx = 1;
			}
			if (RxHeader1.StdId == 0x0305) {
				memcpy(RxCoeffs4, RxData1, sizeof(RxCoeffs4));
				rc_kp_speed1 = ConvertBytesToInt32_LittleEndian(RxCoeffs4);
				rc_kp_speed2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs4[4]);
				flag_of_rx = 1;
			}
			if (RxHeader1.StdId == 0x0307) {
				memcpy(RxCoeffs6, RxData1, sizeof(RxCoeffs6));
				rc_kp_curr1 = ConvertBytesToInt32_LittleEndian(RxCoeffs6);
				rc_kp_curr2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs6[4]);
				flag_of_rx = 1;
			}
		}
		if (number_bridge == 3) {
			if (RxHeader1.StdId == 0x0401) {
				memcpy(RxOther, RxData1, sizeof(RxOther));
				rc_other1_obstacle = RxOther[0];
				rc_other2_obstacle = RxOther[1];
				rc_other1_flag_phase = RxOther[2] & 0x01;  // бит 0
				rc_other2_flag_phase = (RxOther[2] >> 1) & 0x01;   // бит 1
				rc_other_set_to_zero = (RxOther[2] >> 2) & 0x01;   // бит 2
				rc_other1_system_type = (RxOther[2] >> 3) & 0x07;  //биты 3 -5
				rc_other2_system_type = ((RxOther[3] & 0x01) << 2)
						| ((RxOther[2] & 0xC0) >> 6); // бит 6-7 и бит 0 rxother3
				rc_speed_limit1 = RxOther[4]; // скоростной коэффициент  регулятора скорости 1
				rc_speed_limit2 = RxOther[5]; // скоростной коэффициент  регулятора скорости 2
				flag_of_rx = 1;
			}
			if (RxHeader1.StdId == 0x0403) {
				memcpy(RxCoeffs2, RxData1, sizeof(RxCoeffs2));
				rc_ki_angle1 = ConvertBytesToInt32_LittleEndian(RxCoeffs2);
				rc_ki_angle2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs2[4]);
				flag_of_rx = 1;
			}
			if (RxHeader1.StdId == 0x0405) {
				memcpy(RxCoeffs4, RxData1, sizeof(RxCoeffs4));
				rc_kp_speed1 = ConvertBytesToInt32_LittleEndian(RxCoeffs4);
				rc_kp_speed2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs4[4]);
				flag_of_rx = 1;
			}
			if (RxHeader1.StdId == 0x0407) {
				memcpy(RxCoeffs6, RxData1, sizeof(RxCoeffs6));
				rc_kp_curr1 = ConvertBytesToInt32_LittleEndian(RxCoeffs6);
				rc_kp_curr2 = ConvertBytesToInt32_LittleEndian(&RxCoeffs6[4]);
				flag_of_rx = 1;
			}
		}
		DecodePart2();
		received_call++;
	}
}

void init_CAN_TX() {
	if (number_bridge == 1) {
		TxCurrents.StdId = 0x0101;
		TxCurrents.ExtId = 0;
		TxCurrents.RTR = CAN_RTR_DATA;
		TxCurrents.IDE = CAN_ID_STD;
		TxCurrents.DLC = 8;
		TxCurrents.TransmitGlobalTime = 0;

		TxSpeeds.StdId = 0x0103;
		TxSpeeds.ExtId = 0;
		TxSpeeds.RTR = CAN_RTR_DATA;
		TxSpeeds.IDE = CAN_ID_STD;
		TxSpeeds.DLC = 8;
		TxSpeeds.TransmitGlobalTime = 0;

		TxAngles.StdId = 0x0105;
		TxAngles.ExtId = 0;
		TxAngles.RTR = CAN_RTR_DATA;
		TxAngles.IDE = CAN_ID_STD;
		TxAngles.DLC = 8;
		TxAngles.TransmitGlobalTime = 0;
	}
	if (number_bridge == 2) {
		TxCurrents.StdId = 0x0100;
		TxCurrents.ExtId = 0;
		TxCurrents.RTR = CAN_RTR_DATA;
		TxCurrents.IDE = CAN_ID_STD;
		TxCurrents.DLC = 8;
		TxCurrents.TransmitGlobalTime = 0;

		TxSpeeds.StdId = 0x0102;
		TxSpeeds.ExtId = 0;
		TxSpeeds.RTR = CAN_RTR_DATA;
		TxSpeeds.IDE = CAN_ID_STD;
		TxSpeeds.DLC = 8;
		TxSpeeds.TransmitGlobalTime = 0;

		TxAngles.StdId = 0x0104;
		TxAngles.ExtId = 0;
		TxAngles.RTR = CAN_RTR_DATA;
		TxAngles.IDE = CAN_ID_STD;
		TxAngles.DLC = 8;
		TxAngles.TransmitGlobalTime = 0;
	}
	if (number_bridge == 3) {
		TxCurrents.StdId = 0x0501;
		TxCurrents.ExtId = 0;
		TxCurrents.RTR = CAN_RTR_DATA;
		TxCurrents.IDE = CAN_ID_STD;
		TxCurrents.DLC = 8;
		TxCurrents.TransmitGlobalTime = 0;

		TxSpeeds.StdId = 0x0503;
		TxSpeeds.ExtId = 0;
		TxSpeeds.RTR = CAN_RTR_DATA;
		TxSpeeds.IDE = CAN_ID_STD;
		TxSpeeds.DLC = 8;
		TxSpeeds.TransmitGlobalTime = 0;

		TxAngles.StdId = 0x0505;
		TxAngles.ExtId = 0;
		TxAngles.RTR = CAN_RTR_DATA;
		TxAngles.IDE = CAN_ID_STD;
		TxAngles.DLC = 8;
		TxAngles.TransmitGlobalTime = 0;
	}
}

void pack_CAN_data(void) {
	/* --- углы -------------------------------------------------- */
	TxDataAngles[0] = (uint8_t) ((int32_t) (motor_1.angle_reduction * 100.0f)
			>> 24);
	TxDataAngles[1] = (uint8_t) ((int32_t) (motor_1.angle_reduction * 100.0f)
			>> 16);
	TxDataAngles[2] = (uint8_t) ((int32_t) (motor_1.angle_reduction * 100.0f)
			>> 8);
	TxDataAngles[3] = (uint8_t) ((int32_t) (motor_1.angle_reduction * 100.0f));

	TxDataAngles[4] = (uint8_t) ((int32_t) (motor_2.angle_reduction * 100.0f)
			>> 24);
	TxDataAngles[5] = (uint8_t) ((int32_t) (motor_2.angle_reduction * 100.0f)
			>> 16);
	TxDataAngles[6] = (uint8_t) ((int32_t) (motor_2.angle_reduction * 100.0f)
			>> 8);
	TxDataAngles[7] = (uint8_t) ((int32_t) (motor_2.angle_reduction * 100.0f));

	/* --- скорости ---------------------------------------------- */
	TxDataSpeeds[0] = (uint8_t) ((int32_t) (motor_1.speed * 100.0f) >> 24);
	TxDataSpeeds[1] = (uint8_t) ((int32_t) (motor_1.speed * 100.0f) >> 16);
	TxDataSpeeds[2] = (uint8_t) ((int32_t) (motor_1.speed * 100.0f) >> 8);
	TxDataSpeeds[3] = (uint8_t) ((int32_t) (motor_1.speed * 100.0f));

	TxDataSpeeds[4] = (uint8_t) ((int32_t) (motor_2.speed * 100.0f) >> 24);
	TxDataSpeeds[5] = (uint8_t) ((int32_t) (motor_2.speed * 100.0f) >> 16);
	TxDataSpeeds[6] = (uint8_t) ((int32_t) (motor_2.speed * 100.0f) >> 8);
	TxDataSpeeds[7] = (uint8_t) ((int32_t) (motor_2.speed * 100.0f));

	/* --- токи --------------------------------------------------- */
	TxDataCurrents[0] =
			(uint8_t) ((int32_t) (motor_1.current * 10000.0f) >> 24);
	TxDataCurrents[1] =
			(uint8_t) ((int32_t) (motor_1.current * 10000.0f) >> 16);
	TxDataCurrents[2] = (uint8_t) ((int32_t) (motor_1.current * 10000.0f) >> 8);
	TxDataCurrents[3] = (uint8_t) ((int32_t) (motor_1.current * 10000.0f));

	TxDataCurrents[4] =
			(uint8_t) ((int32_t) (motor_2.current * 10000.0f) >> 24);
	TxDataCurrents[5] =
			(uint8_t) ((int32_t) (motor_2.current * 10000.0f) >> 16);
	TxDataCurrents[6] = (uint8_t) ((int32_t) (motor_2.current * 10000.0f) >> 8);
	TxDataCurrents[7] = (uint8_t) ((int32_t) (motor_2.current * 10000.0f));
}


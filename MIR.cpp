#include "MIR.h"

bool MIR_CAN_Packet(CANRxFrame rxmsg)
{
	uint8_t id = rxmsg.EID & 0xFF;
	CAN_PACKET_ID cmd = rxmsg.EID >> 8;

	switch (cmd) {
	case CAN_PACKET_STATUS:
		for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++) {
			stat_tmp = &stat_msgs[i];
			if (stat_tmp->id == id || stat_tmp->id == -1) {
				ind = 0;
				stat_tmp->id = id;
				stat_tmp->rx_time = chVTGetSystemTime();
				stat_tmp->rpm = (float)buffer_get_int32(rxmsg.data8, &ind);
				stat_tmp->current = (float)buffer_get_int16(rxmsg.data8, &ind) / 10.0;
				stat_tmp->duty = (float)buffer_get_int16(rxmsg.data8, &ind) / 1000.0;
				break;
			}
		}
		break;

	case MIR_PING:
		timeout_reset();
		break;

	case MIR_SET_DUTY:
	case MIR_SET_DUTY_GET_TELEMETRY:
		if ((app_get_configuration()->controller_id >= id) && (app_get_configuration()->controller_id < (id + (rxmsg.DLC / 2))))
		{
			uint16_t dutyI = 0;

			memcpy(&dutyI, &(rxmsg.data8[2 * (app_get_configuration()->controller_id - id)]), 2);

			float dutyD = fabs((float)dutyI / 60000.0);

			mc_interface_set_duty(dutyD);
		}

		timeout_reset();

		if (cmd == MIR_SET_DUTY_GET_TELEMETRY) goto getTelemetry;
		break;

	case MIR_GET_TELEMETRY:
	{
	getTelemetry:
		VESC_MIR_TELEMETRY0 telemetry;
		telemetry.rpm = mc_interface_get_rpm();
		telemetry.current = mc_interface_get_tot_current() * 100.0;
		telemetry.duty = mc_interface_get_duty_cycle_now() * 30000.0;
		telemetry.tempMotor = (uint8_t)mc_interface_temp_motor_filtered() *2.0;
		telemetry.tempEsc = (uint8_t)mc_interface_temp_fet_filtered() *2.0;
		comm_can_transmit_eid(app_get_configuration()->controller_id | ((uint32_t)MIR_TELEMETRY0 << 8), (uint8_t*)&telemetry, sizeof(telemetry));
	}
	timeout_reset();
	break;

	default:
		return false;
		break;
	}
	return true;
}
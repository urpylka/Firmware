#include "charging_station.h"
#include "navigator.h"

#include <systemlib/mavlink_log.h>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/uORB.h>

#define ACTION_OPEN 0
#define ACTION_CLOSE 1

#define CUSTOM_MODE_UNKNOWN 0
#define CUSTOM_MODE_OPEN 1
#define CUSTOM_MODE_OPENING 2
#define CUSTOM_MODE_CLOSED 3
#define CUSTOM_MODE_CLOSING 4

#define OPEN_TIMEOUT 20
#define NO_HEARTBEAT_TIMEOUT 6

ChargingStation::ChargingStation(Navigator *navigator) :
	MissionBlock(navigator)
{
}

void
ChargingStation::on_activation()
{
	if (_charging_station_state_sub < 0) {
		_charging_station_state_sub = orb_subscribe(ORB_ID(charging_station_state));
	}

	if (_cmd_ack_sub < 0) {
		_cmd_ack_sub = orb_subscribe(ORB_ID(vehicle_command_ack));
	}

	_sent = false;
	_complete = false;
	_heartbeat_received = false;
}

void
ChargingStation::on_active()
{
	if (!_sent) {
		// Send command to the charging station
		bool to_send = false;

		if (_action == ACTION_OPEN) {
			if (_navigator->get_vstatus()->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				// open charging station when the vehicle is armed
				to_send = true;
				PX4_INFO("Open charging station with id %d", _id);
			}
		} else if (_action == ACTION_CLOSE) {
			to_send = true;
			PX4_INFO("Close charging station with id %d", _id);
		} else {
			PX4_WARN("Unknown charging station action: %d", _action);
			return;
		}

		if (to_send) {
			vehicle_command_s cmd = {};

			cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
			cmd.param1 = charging_station_state_s::BASE_MODE_FLAG_CUSTOM_MODE_ENABLED; // mode
			cmd.param2 = _action == ACTION_OPEN ? CUSTOM_MODE_OPEN : CUSTOM_MODE_CLOSED; // custom mode
			cmd.param3 = 0; // sub mode
			cmd.target_system = _id;
			cmd.target_component = 1; // MAV_COMP_ID_AUTOPILOT1
			cmd.from_external = false;

			_navigator->publish_vehicle_cmd_to_external(&cmd);

			_start_time = hrt_absolute_time();
			_sent = true;
		}

	} else if (hrt_elapsed_time(&_start_time) / 1e6f > OPEN_TIMEOUT) {
		_navigator->set_mission_failure("Charging station opening timeout");

	} else if ((hrt_elapsed_time(&_start_time) / 1e6f > NO_HEARTBEAT_TIMEOUT) &&
			    !_heartbeat_received && !in_flight()) {
		_navigator->set_mission_failure("No heartbeats from the charging station");

	} else if (_action == ACTION_OPEN) {
		bool updated;

		// check if we get non-success ack from the charging station
		orb_check(_cmd_ack_sub, &updated);
		if (updated) {
			struct vehicle_command_ack_s ack;
			orb_copy(ORB_ID(vehicle_command_ack), _cmd_ack_sub, &ack);
			if (ack.command == vehicle_command_s::VEHICLE_CMD_DO_SET_MODE &&
				ack.source_system == _id &&
				ack.result != vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED &&
				ack.result != vehicle_command_ack_s::VEHICLE_RESULT_IN_PROGRESS) {
					_navigator->set_mission_failure("Charging station responded with failure");
					return;
				}
		}

		// check if the charging station is in needed mode
		orb_check(_charging_station_state_sub, &updated);
		if (updated) {
			struct charging_station_state_s state;
			orb_copy(ORB_ID(charging_station_state), _charging_station_state_sub, &state);

			if (state.id == _id && hrt_elapsed_time(&state.timestamp) < 2000000) {
				// valid state
				_heartbeat_received = true;
				if (state.system_status == charging_station_state_s::SYSTEM_STATUS_CRITICAL ||
					state.system_status == charging_station_state_s::SYSTEM_STATUS_EMERGENCY) {
					_navigator->set_mission_failure("Charging station failure");

				} else if (state.custom_mode == CUSTOM_MODE_OPEN) {
					PX4_INFO("Charging station is open");
					_complete = true;
				}
			}
		}
	}
}

bool
ChargingStation::is_mission_item_reached()
{
	if (_action == ACTION_CLOSE) {
		return true;
	} else {
		return _complete;
	}
}

void
ChargingStation::set_params(int id, int action)
{
	PX4_INFO("Set charging station action params: id=%d, action=%d", id, action);
	_id = id;
	_action = action;
}

bool
ChargingStation::in_flight()
{
	return !_navigator->get_land_detected()->landed;
}

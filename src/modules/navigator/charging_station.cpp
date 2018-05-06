#include "charging_station.h"
#include "navigator.h"

#include <uORB/topics/vehicle_command.h>
#include <uORB/uORB.h>


#define ACTION_OPEN 0
#define ACTION_CLOSE 1

ChargingStation::ChargingStation(Navigator *navigator) :
	MissionBlock(navigator)
{
}

void
ChargingStation::on_activation()
{
	// if (_charging_station_state_sub < 0) {
	// 	_charging_station_state_sub = orb_subscribe(ORB_ID(charging_station_state));
	// }

	_sent = false;
	_complete = false;
	_failed = false;
}

void
ChargingStation::on_active()
{
	if (!_sent) {
		// Send command to the charging station

		if (_action == ACTION_OPEN) {
			PX4_INFO("Open charging station with id %d", _id);
		} else if (_action == ACTION_CLOSE) {
			PX4_INFO("Close charging station with id %d", _id);
		}

		vehicle_command_s cmd = {};

		cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
		cmd.param1 = 0; // mode
		cmd.param2 = 666; // custom mode
		cmd.param3 = 777; // sub mode
		cmd.target_system = _id;
		cmd.target_component = 0;
		// cmd.from_external = true;

		PX4_INFO("target system: %d", cmd.target_system);

		_navigator->publish_vehicle_cmd_to_external(&cmd);
		_start_time = hrt_absolute_time();
		_sent = true;
	}

	if (hrt_elapsed_time(&_start_time) / 1e6f > 5) {
		PX4_INFO("Charging station is open");
		_complete = true;
	}
}

bool
ChargingStation::is_mission_item_reached()
{
	if (_action == ACTION_CLOSE) {
		return true;
	} else {
		return _complete && !_failed;
	}
}

void
ChargingStation::set_params(int id, int action)
{
	PX4_INFO("Set charging station action params: id=%d, action=%d", id, action);
	_id = id;
	_action = action;
}

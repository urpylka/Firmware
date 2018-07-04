/***
 * Various COEX related debugging tools.
 *
 * @author Oleg Kalachev <okalachev@gmail.com>
 */

#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/mavlink_log.h>
#include <stdio.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <string.h>

// uORB Publications
#include <uORB/Publication.hpp>
#include <uORB/topics/charging_station_state.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/external_vehicle_position.h>

#define CHARGING_STATION_ID 17

static orb_advert_t mavlink_log_pub;
static bool should_exit;

extern "C" __EXPORT int coex_tools_main(int argc, char *argv[]);
int charging_station_distance_monitor_thread(int argc, char *argv[]);
const char *cs_state_str(uint32_t custom_mode);
int charging_station_state_monitor_thread(int argc, char *argv[]);

int charging_station_distance_monitor_thread(int argc, char *argv[]) {
	PX4_INFO("Starting charging station distance monitor");

	int cs_pos_sub = orb_subscribe(ORB_ID(external_vehicle_position));
	int v_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	external_vehicle_position_s cs_pos;
	vehicle_global_position_s v_pos;

	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = cs_pos_sub;
	fds[0].events = POLLIN;

	while (!should_exit) {
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 10000);
		if (pret <= 0) {
			continue;
		}
		orb_copy(ORB_ID(external_vehicle_position), cs_pos_sub, &cs_pos);

		if (cs_pos.id != CHARGING_STATION_ID) {
			continue;
		}

		orb_copy(ORB_ID(vehicle_global_position), v_pos_sub, &v_pos);

		float dist = get_distance_to_next_waypoint(v_pos.lat, v_pos.lon, cs_pos.lat, cs_pos.lon);

		char buf[50];
		sprintf(buf, "CS %d distance: %g m", cs_pos.id, (double)dist);
		mavlink_log_info(&mavlink_log_pub, buf);
	}

	return 0;
}

const char *cs_state_str(uint32_t custom_mode) {
	// Get string representation of CS state
	switch (custom_mode) {
		case charging_station_state_s::CUSTOM_MODE_UNKNOWN: return "UNKNOWN";
		case charging_station_state_s::CUSTOM_MODE_OPEN: return "OPEN";
		case charging_station_state_s::CUSTOM_MODE_OPENING: return "OPENING";
		case charging_station_state_s::CUSTOM_MODE_CLOSED: return "CLOSED";
		case charging_station_state_s::CUSTOM_MODE_CLOSING: return "CLOSING";
	}
	return "";
}

int charging_station_state_monitor_thread(int argc, char *argv[]) {
	PX4_INFO("Starting charging station state monitor");

	int state_sub = orb_subscribe(ORB_ID(charging_station_state));
	charging_station_state_s state;
	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = state_sub;
	fds[0].events = POLLIN;

	while (!should_exit) {
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 10000);
		if (pret <= 0) {
			continue;
		}
		orb_copy(ORB_ID(charging_station_state), state_sub, &state);

		char buf[50];
		sprintf(buf, "CS %d state: %s (%d)", state.id, cs_state_str(state.custom_mode), state.custom_mode);
		mavlink_log_info(&mavlink_log_pub, buf);
	}

	return 0;
}

int coex_tools_main(int argc, char *argv[]) {
	if (argc < 2) {
		PX4_INFO("usage: coex_tools {cs_dist_mon|cs_hb_mon|stop}");
		return 1;

	} else if (strcmp(argv[1], "cs_dist_mon") == 0) {
		should_exit = false;
		__attribute__((unused)) px4_task_t deamon_task = px4_task_spawn_cmd("cs_dist_mon",
						SCHED_DEFAULT,
						1,
						1000,
						(px4_main_t)&charging_station_distance_monitor_thread,
						nullptr);

	} else if (strcmp(argv[1], "cs_hb_mon") == 0) {
		should_exit = false;
		__attribute__((unused)) px4_task_t deamon_task = px4_task_spawn_cmd("cs_hb_mon",
						SCHED_DEFAULT,
						1,
						1000,
						(px4_main_t)&charging_station_state_monitor_thread,
						nullptr);

	} else if (strcmp(argv[1], "stop") == 0) {
		should_exit = true;
	}

	return OK;
}

px4_add_module(
	MODULE modules__fake_charging_station
	MAIN fake_charging_station
	COMPILE_FLAGS
	SRCS
		fake_charging_station_main.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :

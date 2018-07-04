px4_add_module(
	MODULE modules__coex_tools
	MAIN coex_tools
	COMPILE_FLAGS
	SRCS
		coex_tools_main.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :

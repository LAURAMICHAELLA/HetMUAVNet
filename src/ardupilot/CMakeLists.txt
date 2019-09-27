include(ExternalProject)

set(USE_ARDUPILOT CACHE PATH "Path to an existing ArduPilot source repository")

if(USE_ARDUPILOT)
	# Convert USE_ARDUPILOT (which could be either a relative or an absolute
	# path) to an absolute path
	get_filename_component(
		ARDUPILOT_ABSPATH
		"${USE_ARDUPILOT}" ABSOLUTE
	)

	message(STATUS "Using ArduPilot source tree: ${ARDUPILOT_ABSPATH}")

	ExternalProject_Add(ardupilot
		# Configure paths so that output files are placed in the same
		# path they would have been if the other branch had been taken
		SOURCE_DIR "${ARDUPILOT_ABSPATH}"
		BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/ardupilot/build"
		BUILD_ALWAYS 1

		# Commands
		CONFIGURE_COMMAND "${ARDUPILOT_ABSPATH}/waf" configure --top=${ARDUPILOT_ABSPATH} --out=. --board sitl
		BUILD_COMMAND "${ARDUPILOT_ABSPATH}/waf" build --top=${ARDUPILOT_ABSPATH} --out=. --target bin/arducopter
		INSTALL_COMMAND ""
	)
else(USE_ARDUPILOT)
	set(ARDUPILOT_ABSPATH "${CMAKE_CURRENT_BINARY_DIR}/ardupilot")

	ExternalProject_Add(ardupilot
		# ArduPilot source repository
		GIT_REPOSITORY https://github.com/ArduPilot/ardupilot.git
		GIT_TAG Copter-3.6.0
		GIT_SUBMODULES modules/gtest modules/mavlink modules/waf
		UPDATE_COMMAND ""

		# Build inside downloaded source directory
		SOURCE_DIR ardupilot
		BUILD_ALWAYS 1
		BUILD_IN_SOURCE 1

		# Patch ArduPilot source files with our own version
		PATCH_COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/apply-patches.sh" safe12345678safe

		# Commands
		CONFIGURE_COMMAND ./waf configure --board sitl
		BUILD_COMMAND ./waf build --target bin/arducopter
		INSTALL_COMMAND ""
	)
endif(USE_ARDUPILOT)

install(PROGRAMS
	"${CMAKE_CURRENT_BINARY_DIR}/ardupilot/build/sitl/bin/arducopter"
	DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/gzuav/ardupilot)

install(FILES
	"${ARDUPILOT_ABSPATH}/Tools/autotest/default_params/copter.parm"
	"${ARDUPILOT_ABSPATH}/Tools/autotest/default_params/gazebo-iris.parm"
	DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/gzuav/ardupilot)

set(MAVLINK20_PATH "${CMAKE_CURRENT_BINARY_DIR}/ardupilot/build/sitl/libraries/GCS_MAVLink/include" PARENT_SCOPE)

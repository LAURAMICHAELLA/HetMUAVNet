add_library(libs
	IO/Poll.cpp
	MAVLink/MAVLink.cpp
)

# We steal MAVLink v2.0 from ArduPilot
add_dependencies(libs ardupilot)
message(STATUS "Using MAVLink v2.0 headers from: ${MAVLINK20_PATH}")

target_include_directories(libs PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
	${MAVLINK20_PATH}
)

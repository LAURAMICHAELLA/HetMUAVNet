set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Do not strip rpath in installed libraries
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Compile "GzUav_INTERNAL.so"
add_library(GzUav_INTERNAL SHARED
	GzUavPhaseGenerator.cc
)
set_target_properties(GzUav_INTERNAL PROPERTIES PREFIX "") # no "lib" prefix

# Make subsequent plugins link and load with GzUav_INTERNAL
link_libraries(GzUav_INTERNAL)
SET(CMAKE_INSTALL_RPATH "\$ORIGIN")

# Compile "libGzUavCameraPlugin.so"
add_library(GzUavCameraPlugin SHARED
	GzUavCameraPlugin/FrameServer.cc
	GzUavCameraPlugin/GzUavCameraPlugin.cc
)
target_link_libraries(GzUavCameraPlugin PUBLIC CameraPlugin)

# Compile "libGzUavVehiclePlugin.so"
add_library(GzUavVehiclePlugin SHARED
	GzUavVehiclePlugin/common.cc
	GzUavVehiclePlugin/Gimbal.cc
	GzUavVehiclePlugin/GzUavVehiclePlugin.cc
	GzUavVehiclePlugin/PoseSampler.cc
	GzUavVehiclePlugin/Rotor.cc
)

install(TARGETS
    GzUav_INTERNAL
    GzUavCameraPlugin
    GzUavVehiclePlugin
    DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/gzuav/gazebo/plugins
)

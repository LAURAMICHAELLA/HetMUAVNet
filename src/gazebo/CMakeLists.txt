find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
link_libraries(${GAZEBO_LIBRARIES})

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
	if(ITR MATCHES ".*gazebo-[0-9.]+$")
		set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
	endif()
endforeach()

install(DIRECTORY models DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/gzuav/gazebo)

add_subdirectory(plugins)

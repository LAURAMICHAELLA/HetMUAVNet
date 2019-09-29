# Ns-3 version we reuired to be compiled against. It must correspond to the one
# that is packaged in Ubuntu.
set(NS3VER 3.29)

# Locate ns-3's libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(NS3 REQUIRED
	libns${NS3VER}-applications
	libns${NS3VER}-core
	libns${NS3VER}-csma
	libns${NS3VER}-internet
	libns${NS3VER}-lr-wpan
	libns${NS3VER}-mobility
	libns${NS3VER}-network
	libns${NS3VER}-point-to-point
	libns${NS3VER}-stats
	libns${NS3VER}-wifi
)
add_definitions(${NS3_CFLAGS})
include_directories(${NS3_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/include)
link_libraries(${NS3_LIBRARIES})

# List of C++ header files comprising the external-sync module
set(HDRS
	external-sync/model/external-sync-manager.h
	external-sync/model/external-sync-simulator-impl.h
)

# List of C++ source files comprising the external-sync module
set(SRCS
	external-sync/model/external-sync-manager.cc
	external-sync/model/external-sync-simulator-impl.cc
)

# Put symlinks to our .h files in a subdiractory called "ns3", so that they can
# be included as if they were installed <ns3/filename.h>
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/ns3)
foreach (FNAME ${HDRS})
	get_filename_component(FNAME_BASENAME ${FNAME} NAME)
	execute_process(COMMAND cmake -E create_symlink
		${CMAKE_CURRENT_SOURCE_DIR}/${FNAME}
		${CMAKE_CURRENT_BINARY_DIR}/include/ns3/${FNAME_BASENAME}
	)
endforeach (FNAME)

# Build ns-3 module
add_library(ns${NS3VER}-external-sync SHARED ${SRCS})
link_libraries(ns${NS3VER}-external-sync)

# Install ns-3 module
install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ns${NS3VER}/ns3)
install(TARGETS ns${NS3VER}-external-sync DESTINATION ${CMAKE_INSTALL_LIBDIR})

# Build and install our examples too
add_executable(ns3-dev-external-sync-lan external-sync/examples/lan.cc)
add_executable(ns3-dev-external-sync-lr-wpan external-sync/examples/lr-wpan.cc)
add_executable(ns3-dev-external-sync-p2p external-sync/examples/p2p.cc)
add_executable(ns3-dev-external-sync-wifi-adhoc external-sync/examples/wifi-adhoc.cc)
add_executable(ns3-dev-external-sync-wifi external-sync/examples/wifi.cc)

install(TARGETS
	ns3-dev-external-sync-lan
	ns3-dev-external-sync-lr-wpan
	ns3-dev-external-sync-p2p
	ns3-dev-external-sync-wifi-adhoc
	ns3-dev-external-sync-wifi
	DESTINATION ${CMAKE_INSTALL_BINDIR}
)

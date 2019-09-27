add_executable(gzuavchannel
	main.cpp
	CommandLineParser.cpp
	ExternalSyncServer.cpp
	TCPTransport.cpp
	Transport.cpp
	UDSTransport.cpp
)

target_link_libraries(gzuavchannel libs)
install(TARGETS gzuavchannel DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/gzuav)

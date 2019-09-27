add_executable(mavmix
	main.cpp
)

target_link_libraries(mavmix libs)
install(TARGETS mavmix DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/gzuav)

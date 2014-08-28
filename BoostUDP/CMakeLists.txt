project(BoostUDP CXX)

if(MSVC)
	add_definitions( "/W3 /wd4503 /wd4996 " )
endif()

set(UDP_COMMON_PROJ BoostUDP_Common)

file(GLOB_RECURSE COMMON_SOURCE 
	"Common/*.h"
	"Common/*.hpp"
	"Common/*.cpp")

add_library(${UDP_COMMON_PROJ} ${COMMON_SOURCE})
set_target_properties(${UDP_COMMON_PROJ} PROPERTIES OUTPUT_NAME_DEBUG ${UDP_COMMON_PROJ}D )
set_target_properties(${UDP_COMMON_PROJ} PROPERTIES OUTPUT_NAME_RELEASE ${UDP_COMMON_PROJ})


target_link_libraries(${UDP_COMMON_PROJ} 
	${Boost_LIBRARIES} 
)


set(UDP_SERVER_PROJ BoostUDP_Server)

add_executable(${UDP_SERVER_PROJ} ServerMain.cpp)
set_target_properties(${UDP_SERVER_PROJ} PROPERTIES OUTPUT_NAME_DEBUG ${UDP_SERVER_PROJ}D )
set_target_properties(${UDP_SERVER_PROJ} PROPERTIES OUTPUT_NAME_RELEASE ${UDP_SERVER_PROJ})

add_dependencies(${UDP_SERVER_PROJ} BoostUDP_Common)

target_link_libraries(${UDP_SERVER_PROJ} 
	${Boost_LIBRARIES} 
	${UDP_COMMON_PROJ}
)


set(UDP_CLIENT_PROJ BoostUDP_Client)

add_executable(${UDP_CLIENT_PROJ} ServerMain.cpp)
set_target_properties(${UDP_CLIENT_PROJ} PROPERTIES OUTPUT_NAME_DEBUG ${UDP_CLIENT_PROJ}D )
set_target_properties(${UDP_CLIENT_PROJ} PROPERTIES OUTPUT_NAME_RELEASE ${UDP_CLIENT_PROJ})

add_dependencies(${UDP_CLIENT_PROJ} BoostUDP_Common)

target_link_libraries(${UDP_CLIENT_PROJ} 
	${Boost_LIBRARIES} 
	${UDP_COMMON_PROJ}
)
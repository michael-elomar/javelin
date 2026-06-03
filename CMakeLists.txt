cmake_minimum_required(VERSION 3.5)
project(javelin LANGUAGES C CXX)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

set(SRC_FILES
	src/main.cpp
	src/logger.cpp
	src/model.cpp
	src/world.cpp
)

set(INCLUDE
	include
	src
)

add_subdirectory(proto)
get_property(PROTO_CC_HEADERS GLOBAL PROPERTY PROTO_CC_HEADERS)
list(APPEND INCLUDE ${PROTO_CC_HEADERS})

add_executable(${PROJECT_NAME} ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${INCLUDE}>)
target_link_libraries(${PROJECT_NAME} PUBLIC
	Eigen3::Eigen
	protobuf::libprotobuf
	nexus
	neutron
)

#======== Install Support ========#

set(INSTALL_TARGETS
	${PROJECT_NAME}
)

install(TARGETS ${INSTALL_TARGETS}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin)

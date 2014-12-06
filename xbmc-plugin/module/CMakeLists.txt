cmake_minimum_required (VERSION 2.8.11)

set(PROJECT openbm)
project (${PROJECT} CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")


set(SRC
   main.cpp
   BC.cpp
   MID.cpp
   TCPIPClient.cpp
)

set(HEADERS
   BC.h
   IPCClient.h
   MID.h
)

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

add_library (${PROJECT} SHARED ${SRC} ${HEADERS})
set_target_properties(${PROJECT} PROPERTIES PREFIX "")

cmake_minimum_required (VERSION 2.8.11)

set(PROJECT openbm-client)
project (${PROJECT} CXX)



set(SRC
   main.cpp
   TCPIPClient.cpp
)

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

add_executable (${PROJECT} ${SRC})

# Boost
set(Boost_USE_STATIC_LIBS        ON) # only find static libs
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package(Boost COMPONENTS date_time system thread program_options)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT} ${Boost_LIBRARIES})

# other stuff
#target_link_libraries(${PROJECT} dl)
target_link_libraries(${PROJECT} pthread)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

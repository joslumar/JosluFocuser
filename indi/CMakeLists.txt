cmake_minimum_required(VERSION 2.4.7)
PROJECT(joslufocus CXX C)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake_modules/")
include(GNUInstallDirs)

set (DUINO_VERSION_MAJOR 0)
set (DUINO_VERSION_MINOR 2)
 
find_package(INDI REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/joslufocus.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/joslufocus.xml )

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${INDI_INCLUDE_DIR})

##################### indi joslumar #####################
set(joslufocus_SRCS
   ${CMAKE_CURRENT_SOURCE_DIR}/joslufocus.cpp
   )

add_executable(indi_joslufocus ${joslufocus_SRCS})
target_link_libraries(indi_joslufocus ${INDI_LIBRARIES})

install(TARGETS indi_joslufocus RUNTIME DESTINATION bin)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/joslufocus.xml DESTINATION ${INDI_DATA_DIR})


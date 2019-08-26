cmake_minimum_required ( VERSION 2.8 )
project ( path_planning )

set ( CMAKE_BUILD_TYPE Release )
set ( CMAKE_CXX_STANDARD 11 )
set ( CMAKE_CXX_STANDARD_REQUIRED ON )

find_package( Yaml-cpp REQUIRED )

find_package( OpenCV   REQUIRED )

include_directories ( 
	inc 
	${YAMLCPP_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

set( PATH_PLANNING_HDRS 
	${CMAKE_CURRENT_SOURCE_DIR}/inc/planner.h
	${CMAKE_CURRENT_SOURCE_DIR}/inc/path_finder.h	
	${CMAKE_CURRENT_SOURCE_DIR}/inc/map.h
)

set( PATH_PLANNING_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/path_finder.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/map.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/heuristic.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/search.cpp
)

add_library( ${PROJECT_NAME} SHARED ${PATH_PLANNING_SRCS} ${PATH_PLANNING_HDRS} )

add_executable ( planner test/test_${PROJECT_NAME}.cpp )

target_link_libraries( planner
  ${PROJECT_NAME} 
	${OpenCV_LIBS}
	${YAMLCPP_LIBRARIES}
)


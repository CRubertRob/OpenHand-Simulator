cmake_minimum_required (VERSION 2.6.0)
project(umano)
find_package(OpenRAVE REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include)
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_library(umano SHARED src/umano/umano.cpp)
set_target_properties(umano PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(umano ${OpenRAVE_LIBRARIES})
install(TARGETS umano DESTINATION .)

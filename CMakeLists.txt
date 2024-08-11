cmake_minimum_required(VERSION 3.14)
project(groundgrid/cpp
	VERSION 1.0.0
	DESCRIPTION "ground_grid_segmentation"
	LANGUAGES CXX
)


# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(cpp/cmake/CompilerOptions.cmake)
include(cpp/cmake/dependencies.cmake)
add_library(groundgrid_lib STATIC)

target_sources(groundgrid_lib PRIVATE cpp/groundgrid/main.cpp
                cpp/groundgrid/test.cpp
            #    cpp/groundgrid/GroundGrid.cpp 
            #    cpp/groundgrid/GroundGridNode.cpp
            #    cpp/groundgrid/GroundGridNodelet.cpp
            #    cpp/groundgrid/GroundSegmentation.cpp
            #    cpp/groundgrid/GridMap.cpp
               )

target_link_libraries(groundgrid_lib PUBLIC Eigen3::Eigen)
set_global_target_properties(groundgrid_lib)
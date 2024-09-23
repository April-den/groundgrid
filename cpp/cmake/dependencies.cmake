set(EIGEN_BUILD_DOC OFF CACHE BOOL "Don't build Eigen docs")
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "Don't build Eigen tests")
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "Don't build Eigen pkg-config")
set(EIGEN_BUILD_BLAS OFF CACHE BOOL "Don't build blas module")
set(EIGEN_BUILD_LAPACK OFF CACHE BOOL "Don't build lapack module")

include(FetchContent)
FetchContent_Declare(eigen URL https://github.com/nachovizzo/eigen/archive/refs/tags/3.4.90.tar.gz)
if(NOT eigen_POPULATED)
  FetchContent_Populate(eigen)
  if(${CMAKE_VERSION} GREATER_EQUAL 3.25)
    add_subdirectory(${eigen_SOURCE_DIR} ${eigen_BINARY_DIR} SYSTEM EXCLUDE_FROM_ALL)
  else()
    # Emulate the SYSTEM flag introduced in CMake 3.25. Withouth this flag the compiler will
    # consider this 3rdparty headers as source code and fail due the -Werror flag.
    add_subdirectory(${eigen_SOURCE_DIR} ${eigen_BINARY_DIR} EXCLUDE_FROM_ALL)
    get_target_property(eigen_include_dirs eigen INTERFACE_INCLUDE_DIRECTORIES)
    set_target_properties(eigen PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${eigen_include_dirs}")
  endif()
endif()

include_directories(cpp/groundgrid/test.h)
link_directories(cpp/groundgrid/test.cpp)
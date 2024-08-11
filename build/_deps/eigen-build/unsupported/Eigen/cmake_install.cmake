# Install script for directory: /home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/AdolcForward"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/AlignedVector3"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/ArpackSupport"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/AutoDiff"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/BVH"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/EulerAngles"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/FFT"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/IterativeSolvers"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/KroneckerProduct"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/LevenbergMarquardt"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/MatrixFunctions"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/MoreVectorization"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/MPRealSupport"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/NNLS"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/NonLinearOptimization"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/NumericalDiff"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/OpenGLSupport"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/Polynomials"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/Skyline"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/SparseExtra"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/SpecialFunctions"
    "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/Splines"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "/home/aiyang/groundgrid/build/_deps/eigen-src/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/aiyang/groundgrid/build/_deps/eigen-build/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()


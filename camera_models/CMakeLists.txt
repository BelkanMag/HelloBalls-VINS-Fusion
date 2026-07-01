cmake_minimum_required(VERSION 3.8)
project(camera_models)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

set(CAMERA_MODEL_SOURCES
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/PinholeFullCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc
)

add_library(camera_models ${CAMERA_MODEL_SOURCES})
target_include_directories(camera_models PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Boost_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)
target_compile_options(camera_models PRIVATE
    -include ${CMAKE_CURRENT_SOURCE_DIR}/../compat/opencv4_compat.h
)
target_link_libraries(camera_models
    PUBLIC
    Eigen3::Eigen
    PRIVATE
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
)

add_executable(Calibrations
    src/intrinsic_calib.cc
    ${CAMERA_MODEL_SOURCES}
)
target_include_directories(Calibrations PRIVATE
    include
    ${Boost_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)
target_compile_options(Calibrations PRIVATE
    -include ${CMAKE_CURRENT_SOURCE_DIR}/../compat/opencv4_compat.h
)
target_link_libraries(Calibrations
    PRIVATE
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    Eigen3::Eigen
)

install(TARGETS camera_models
    EXPORT export_camera_models
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(TARGETS Calibrations
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
    DESTINATION include
)

ament_export_targets(export_camera_models HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_package()

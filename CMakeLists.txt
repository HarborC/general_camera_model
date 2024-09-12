cmake_minimum_required(VERSION 3.10)
project(general_camera_model)

# build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_definitions(-w)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
if (OpenCV_FOUND)
  message("OpenCV has found.")
  message("OpenCV_VERSION: ${OpenCV_VERSION}")
else()
  message("ERROR: OpenCV could not be found.")
endif()

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/thirdparty
)

add_library(${PROJECT_NAME} SHARED src/general_camera_model.cpp src/function.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC 
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/thirdparty>
                           $<INSTALL_INTERFACE:include>
                           $<INSTALL_INTERFACE:thirdparty>)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

# Unit test
find_package(GTest REQUIRED)
include_directories(${GTest_INCLUDE_DIRS})

# # install
# set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY thirdparty/
  DESTINATION thirdparty/
)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}-targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(EXPORT ${PROJECT_NAME}-targets
        FILE ${PROJECT_NAME}-config.cmake
        DESTINATION share/cmake/${PROJECT_NAME}/
     )


# enable_testing()

# add_subdirectory(unit_test)

# add_subdirectory(apps)

add_subdirectory(pybind)
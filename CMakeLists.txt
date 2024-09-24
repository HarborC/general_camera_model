cmake_minimum_required(VERSION 3.10)
project(general_camera_model)

################################################################################
# CMAKE configuration
################################################################################

add_definitions(-w)
add_definitions(-DCOMPILEDWITHC17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE "Release")

set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}" "-std=c++17" )
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_COMPILER "nvcc")
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
set(CMAKE_CUDA_FLAGS_DEBUG "-g")
set(CMAKE_CUDA_FLAGS_RELEASE "-O3")
set(CMAKE_CUDA_ARCHITECTURES 80)
set(CUDA_NVCC_FLAGS "-rdc=true")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread -Wall -fPIC ${AVX_FLAGS}") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall -fPIC ${AVX_FLAGS}")

add_definitions(-DBOOST_EXCEPTION_DISABLE
                -DBOOST_GEOMETRY_OVERLAY_NO_THROW
                -DEIGEN_NO_DEBUG
                )
                
set(ENV{LD_LIBRARY_PATH} "/usr/local/lib/:$ENV{LD_LIBRARY_PATH}")

include(${PROJECT_SOURCE_DIR}/cmake/find_deps.cmake)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/thirdparty
)

add_library(${PROJECT_NAME} SHARED src/general_camera_model.cpp src/function.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC 
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/thirdparty>
                           $<INSTALL_INTERFACE:include>)
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
  DESTINATION include/
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
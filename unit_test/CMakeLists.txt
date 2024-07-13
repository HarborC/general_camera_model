# general_camera_model
add_executable(general_camera_model_test general_camera_model_test.cpp)
target_link_libraries(general_camera_model_test general_camera_model ${GTEST_BOTH_LIBRARIES})
target_compile_definitions(general_camera_model_test PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_data/")
add_test(test_general_camera_model ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/general_camera_model_test)

# simple_pinhole_camera_model
add_executable(simple_pinhole_camera_model_test simple_pinhole_camera_model_test.cpp)
target_link_libraries(simple_pinhole_camera_model_test general_camera_model ${GTEST_BOTH_LIBRARIES})
target_compile_definitions(general_camera_model_test PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_data/")
add_test(test_simple_pinhole_camera_model ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/simple_pinhole_camera_model_test)

# equirectangular_camera_model
add_executable(equirectangular_camera_model_test equirectangular_camera_model_test.cpp)
target_link_libraries(equirectangular_camera_model_test general_camera_model ${GTEST_BOTH_LIBRARIES})
target_compile_definitions(equirectangular_camera_model_test PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_data/")
add_test(test_equirectangular_camera_model ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/equirectangular_camera_model_test)

# opencv_pinhole_camera_model
add_executable(opencv_pinhole_camera_model_test opencv_pinhole_camera_model_test.cpp)
target_link_libraries(opencv_pinhole_camera_model_test general_camera_model ${GTEST_BOTH_LIBRARIES})
target_compile_definitions(opencv_pinhole_camera_model_test PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_data/")
add_test(test_opencv_pinhole_camera_model ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/opencv_pinhole_camera_model_test)

# opencv_fisheye_camera_model
add_executable(opencv_fisheye_camera_model_test opencv_fisheye_camera_model_test.cpp)
target_link_libraries(opencv_fisheye_camera_model_test general_camera_model ${GTEST_BOTH_LIBRARIES})
target_compile_definitions(opencv_fisheye_camera_model_test PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_data/")
add_test(test_opencv_fisheye_camera_model ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/opencv_fisheye_camera_model_test)

# cubemap_camera_model
add_executable(cubemap_camera_model_test cubemap_camera_model_test.cpp)
target_link_libraries(cubemap_camera_model_test general_camera_model ${GTEST_BOTH_LIBRARIES})
target_compile_definitions(cubemap_camera_model_test PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_data/")
add_test(test_cubemap_camera_model ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/cubemap_camera_model_test)

# double_sphere_camera_model
add_executable(double_sphere_camera_model_test double_sphere_camera_model_test.cpp)
target_link_libraries(double_sphere_camera_model_test general_camera_model ${GTEST_BOTH_LIBRARIES})
target_compile_definitions(double_sphere_camera_model_test PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_data/")
add_test(test_double_sphere_camera_model ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/double_sphere_camera_model_test)

# extended_unified_camera_model
add_executable(extended_unified_camera_model_test extended_unified_camera_model_test.cpp)
target_link_libraries(extended_unified_camera_model_test general_camera_model ${GTEST_BOTH_LIBRARIES})
target_compile_definitions(extended_unified_camera_model_test PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_data/")
add_test(test_extended_unified_camera_model ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/extended_unified_camera_model_test)

# unified_camera_model
add_executable(unified_camera_model_test unified_camera_model_test.cpp)
target_link_libraries(unified_camera_model_test general_camera_model ${GTEST_BOTH_LIBRARIES})
target_compile_definitions(unified_camera_model_test PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_data/")
add_test(test_unified_camera_model ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/unified_camera_model_test)

# fov_camera_model
add_executable(fov_camera_model_test fov_camera_model_test.cpp)
target_link_libraries(fov_camera_model_test general_camera_model ${GTEST_BOTH_LIBRARIES})
target_compile_definitions(fov_camera_model_test PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_data/")
add_test(test_fov_camera_model ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/fov_camera_model_test)

# scaramuzza_camera_model
add_executable(scaramuzza_camera_model_test scaramuzza_camera_model_test.cpp)
target_link_libraries(scaramuzza_camera_model_test general_camera_model ${GTEST_BOTH_LIBRARIES})
target_compile_definitions(scaramuzza_camera_model_test PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_data/")
add_test(test_scaramuzza_camera_model ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scaramuzza_camera_model_test)
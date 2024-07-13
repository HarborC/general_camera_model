# create camera_model
add_executable(create_camera_model create_camera_model.cpp)
target_link_libraries(create_camera_model general_camera_model)

# remap
add_executable(remap remap.cpp)
target_link_libraries(remap general_camera_model)
target_compile_definitions(remap PRIVATE PROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../")

# scaramuzza_camera_unditort
add_executable(scaramuzza_camera_unditort scaramuzza_camera_unditort.cpp)
target_link_libraries(scaramuzza_camera_unditort general_camera_model)
target_compile_definitions(scaramuzza_camera_unditort PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_data/")

# camera_model_convert
add_executable(camera_model_convert camera_model_convert.cpp)
target_link_libraries(camera_model_convert general_camera_model)
target_compile_definitions(camera_model_convert PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_data/")

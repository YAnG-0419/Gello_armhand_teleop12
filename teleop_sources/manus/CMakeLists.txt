cmake_minimum_required(VERSION 3.16)
project(manus_right_hand_teleop LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(REPOSITORY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(MANUS_SDK_ROOT "${REPOSITORY_ROOT}/third_party/manus_sdk")

add_executable(manus_right_hand_teleop src/manus_right_hand_teleop.cpp)
target_include_directories(
  manus_right_hand_teleop
  PRIVATE include "${MANUS_SDK_ROOT}/include"
)
target_link_libraries(
  manus_right_hand_teleop
  PRIVATE "${MANUS_SDK_ROOT}/lib/libManusSDK_Integrated.so" pthread
)
target_compile_definitions(
  manus_right_hand_teleop
  PRIVATE
    MANUS_DEFAULT_CALIBRATION_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/Calibration_right.mcal"
)
set_target_properties(
  manus_right_hand_teleop
  PROPERTIES
    BUILD_RPATH "${MANUS_SDK_ROOT}/lib"
    INSTALL_RPATH "$ORIGIN/../../../third_party/manus_sdk/lib"
)
target_compile_options(
  manus_right_hand_teleop PRIVATE -Wall -Wextra -Wpedantic -Werror
)

add_library(manus_skeleton_bridge SHARED native/manus_skeleton_bridge.cpp)
target_include_directories(
  manus_skeleton_bridge
  PRIVATE native "${MANUS_SDK_ROOT}/include"
)
target_link_libraries(
  manus_skeleton_bridge
  PRIVATE "${MANUS_SDK_ROOT}/lib/libManusSDK_Integrated.so" pthread
)
set_target_properties(
  manus_skeleton_bridge
  PROPERTIES
    BUILD_RPATH "${MANUS_SDK_ROOT}/lib"
    OUTPUT_NAME manus_skeleton_bridge
)
target_compile_options(
  manus_skeleton_bridge PRIVATE -Wall -Wextra -Wpedantic -Werror
)

enable_testing()
add_executable(test_manus_retarget tests/test_retarget.cpp)
target_include_directories(test_manus_retarget PRIVATE include)
target_compile_options(test_manus_retarget PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME manus_retarget COMMAND test_manus_retarget)

cmake_minimum_required(VERSION 3.27)
project(camera_calibration)

add_compile_options(-std=c++17)

# set(CMAKE_PREFIX_PATH "/usr/local/include" "/opt/homebrew/opt" "/opt/homebrew/lib")
# set(CMAKE_BUILD_TYPE "Debug")

set(MODULE_TEST         ON)
set(MODULE_BENCHMARK    OFF)

message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
message(STATUS "Is module test: ${MODULE_TEST}")
message(STATUS "Is module benchmark: ${MODULE_BENCHMARK}")

# Dependencies
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

set(DEPENDENCIES 
    Eigen3::Eigen
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES})

include_directories("${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include")

set(EXAMPLE_DIR	"${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src")

# Build exec file
# add_executable(
#     ${PROJECT_NAME}
#     ${EXAMPLE_DIR}/main.cpp)
# target_link_libraries(
#     ${PROJECT_NAME}
#     ${DEPENDENCIES}
# )


if (${MODULE_TEST})
    enable_testing()
    find_package(GTest REQUIRED)
    set(TEST_DEPENDENCIES 
        GTest::GTest 
        GTest::Main)
    set(TEST_DIR "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/test")
    
    file(GLOB TEST_SOURCES "${TEST_DIR}/*.cpp")
    foreach(TEST_SOURCE ${TEST_SOURCES})
        get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
        add_executable(${TEST_NAME} ${TEST_SOURCE})
        target_link_libraries(${TEST_NAME} ${TEST_DEPENDENCIES} ${DEPENDENCIES})
        add_test(${TEST_NAME} ${TEST_NAME})
    endforeach()
endif()

if(${MODULE_BENCHMARK})
    find_package(benchmark REQUIRED)
    set(BENCHMARK_DEPENDENCIES 
        benchmark::benchmark 
        benchmark::benchmark_main)


endif()







# include(FetchContent)

# FetchContent_Declare(
#   jsoncpp
#   GIT_REPOSITORY https://github.com/open-source-parsers/jsoncpp.git
#   GIT_TAG        master
# )
# FetchContent_MakeAvailable(jsoncpp)

# include_directories(
#   include
#   ${OpenCV_INCLUDE_DIRECTORIES}
#   ${CERES_INCLUDE_DIRS}
# )

# add_executable(${PROJECT_NAME}
#     src/main.cpp
# )

# target_link_libraries(${PROJECT_NAME}
#   ${OpenCV_LIBRARIES}
#   ${CERES_LIBRARIES}
#   # jsoncpp_lib_static
#   jsoncpp_lib
# )
# target_include_directories(${PROJECT_NAME} PRIVATE ${jsoncpp_SOURCE_DIR}/include)

# add_executable(DSCM_calib
#     src/main_calib.cpp
# )

# target_link_libraries(DSCM_calib
#   ${OpenCV_LIBRARIES}
#   ${CERES_LIBRARIES}
#   # jsoncpp_lib_static
#   jsoncpp_lib
# )
# target_include_directories(DSCM_calib PRIVATE ${jsoncpp_SOURCE_DIR}/include)



# enable_testing()

# add_executable(
#   test_camera_model
#   src/test_camera_model.cpp
# )
# target_link_libraries(
#   test_camera_model
#   GTest::gtest_main
#   ${OpenCV_LIBRARIES}
#   ${CERES_LIBRARIES}
# )


# include(GoogleTest)
# gtest_discover_tests(test_camera_model)
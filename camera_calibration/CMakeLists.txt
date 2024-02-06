cmake_minimum_required(VERSION 3.22)
project(camera_calibration)

add_compile_options(-std=c++17)

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

##### jsoncpp #####
include(FetchContent)
FetchContent_Declare(
  jsoncpp
  GIT_REPOSITORY https://github.com/open-source-parsers/jsoncpp.git
  GIT_TAG        master
)
# jsoncpp 속성 가져오기
FetchContent_GetProperties(jsoncpp)
if(NOT jsoncpp_POPULATED)
  FetchContent_Populate(jsoncpp)
  # jsoncpp 테스트 비활성화
  set(JSONCPP_WITH_TESTS OFF CACHE BOOL "" FORCE)
  set(JSONCPP_WITH_POST_BUILD_UNITTEST OFF CACHE BOOL "" FORCE)
  # jsoncpp 추가
  add_subdirectory(${jsoncpp_SOURCE_DIR} ${jsoncpp_BINARY_DIR})
endif()
##### jsoncpp #####


set(DEPENDENCIES 
    Eigen3::Eigen
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
    jsoncpp_lib)


include_directories("${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include")
set(EXAMPLE_DIR	"${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src")



add_executable(mono_calibration ${EXAMPLE_DIR}/mono_calibration.cpp)
target_link_libraries(mono_calibration ${DEPENDENCIES})
target_include_directories(mono_calibration PRIVATE ${jsoncpp_SOURCE_DIR}/include)


add_executable(undistort ${EXAMPLE_DIR}/undistort.cpp)
target_link_libraries(undistort ${DEPENDENCIES}
)
target_include_directories(undistort PRIVATE ${jsoncpp_SOURCE_DIR}/include)


add_executable(stereo_calibration ${EXAMPLE_DIR}/stereo_calibration.cpp)
target_link_libraries(stereo_calibration ${DEPENDENCIES}
)
target_include_directories(stereo_calibration PRIVATE ${jsoncpp_SOURCE_DIR}/include)



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
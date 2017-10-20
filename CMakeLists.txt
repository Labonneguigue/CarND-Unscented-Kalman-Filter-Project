project(UnscentedKF)

# Options. Turn on with 'cmake -Dmyvarname=ON'.
option(test "Build all tests." OFF) # Makes boolean 'test' available.

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

#set(sources src/ukf.cpp src/main.cpp src/tools.cpp src/ukf.h src/tools.h)
include_directories(
    ${PROJECT_SOURCE_DIR}/src
    )

add_library(
    ukf
    src/ukf.h
    src/ukf.cpp
    )

add_library(
    tools
    src/tools.h
    src/tools.cpp
    )

set(GOOGLETEST_ROOT googletest/googletest CACHE STRING "Google Test source root")

include_directories(
    ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}
    ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}/include
    )

set(GOOGLETEST_SOURCES
    ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}/src/gtest-all.cc
    ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}/src/gtest_main.cc
    )

foreach(_source ${GOOGLETEST_SOURCES})
    set_source_files_properties(${_source} PROPERTIES GENERATED 1)
endforeach()

add_library(googletest ${GOOGLETEST_SOURCES})

add_executable(
    unit_tests
    src/test/main_tests.cpp
    src/test/ukf_tests.cpp
    src/test/tools_tests.cpp
    )

add_dependencies(unit_tests googletest)

target_link_libraries(
    unit_tests
    googletest
    ukf
    tools
    )

#include(CTest)

enable_testing()

add_test(unit ${PROJECT_BINARY_DIR}/unit_tests)


###

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

    include_directories(/src/)
    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

#add_library(UnscentedKF_lib ${sources})
add_executable(UnscentedKF src/main.cpp)

target_link_libraries(UnscentedKF tools ukf z ssl uv uWS)


################################
# Testing
################################
if (0)#(test)
    if (APPLE)
        add_definitions(-DGTEST_USE_OWN_TR1_TUPLE)
        add_definitions(-D__GLIBCXX__)
    endif (APPLE)

    # This adds another subdirectory, which has 'project(gtest)'.
    add_subdirectory(googletest/googletest)

    enable_testing()

    # Include the gtest library. gtest_SOURCE_DIR is available due to
    # 'project(gtest)' above.
    include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

    ##############
    # Unit Tests
    ##############
    add_executable(runUnitTests src/test/ukf_tests.cpp)

    # Standard linking to gtest stuff.
    target_link_libraries(runUnitTests gtest gtest_main)

    # Extra linking for the project.
    target_link_libraries(runUnitTests UnscentedKF_lib)

    # This is so you can do 'make test' to see all your tests run, instead of
    # manually running the executable runUnitTests to see those specific tests.
    #add_test(NAME that-test-I-made COMMAND runUnitTests)

    # You can also omit NAME and COMMAND. The second argument could be some other
    # test executable.
    #add_test(that-other-test-I-made runUnitTests)
endif()

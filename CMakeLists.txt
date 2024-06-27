cmake_minimum_required(VERSION 3.28)
project(GRS_Simulator)

set(CMAKE_CXX_STANDARD 17)

add_executable(GRS_Simulator source/main.cpp
        source/Definitions/yamlCustomStructure.h
        source/Mathematics/vector2.h
        source/Mathematics/vector3.h
        source/RigidBodyAndLink/node.cpp
        source/RigidBodyAndLink/node.h
        source/RigidBodyAndLink/rigidBody.cpp
        source/RigidBodyAndLink/rigidBody.h
        source/world.cpp
        source/world.h
        source/RigidBodyAndLink/fixedWing.cpp
        source/RigidBodyAndLink/fixedWing.h
        source/Control/lowLevelController.cpp
        source/Control/lowLevelController.h
        source/Definitions/worldPhysics.h
        source/Mathematics/grsMath.h
        source/Mathematics/matrix3.h
        source/Control/lowPassFilter.cpp
        source/Control/lowPassFilter.h
        source/Control/filter.h
        source/Control/slewLimiter.cpp
        source/Control/slewLimiter.h
        source/Control/pid.cpp
        source/Control/pid.h
        source/Control/pitchController.cpp
        source/Control/pitchController.h
        source/system.cpp
        source/system.h
        source/Definitions/tetherDefinitions.h
        source/Definitions/fixedWingDefinitions.h
        source/Definitions/configurationDefinitions.h
        source/Control/llcBaseVersion.cpp
        source/Control/llcBaseVersion.h
        source/Control/rollController.cpp
        source/Control/rollController.h
        source/Control/altitudeController.cpp
        source/Control/altitudeController.h
        source/RigidBodyAndLink/link.cpp
        source/RigidBodyAndLink/link.h
        source/RigidBodyAndLink/tether.cpp
        source/RigidBodyAndLink/tether.h
        source/configurationParser.cpp
        source/configurationParser.h
        source/Definitions/systemDefinitions.h
        source/Control/highLevelController.cpp
        source/Control/highLevelController.h
        source/Control/groundspeedController.cpp
        source/Control/groundspeedController.h
        source/Control/hlcSingleAircraft.cpp
        source/Control/hlcSingleAircraft.h
        source/Log/logger.cpp
        source/Log/logger.h
        source/Definitions/logDefinitions.h
)

target_include_directories (GRS_Simulator PUBLIC source)

include(FetchContent)

# YAML
FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG yaml-cpp-0.7.0 # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_GetProperties(yaml-cpp)

if(NOT yaml-cpp_POPULATED)
    message(STATUS "Fetching yaml-cpp...")
    FetchContent_Populate(yaml-cpp)
    add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
endif()
target_link_libraries(GRS_Simulator PUBLIC yaml-cpp::yaml-cpp)


# GOOGLE TESTS
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(GRS_Simulator_tests
        tests/tests-vector2.cpp
        tests/tests-vector3.cpp
        tests/tests-mathematics.cpp
)

target_include_directories (GRS_Simulator_tests PUBLIC source)
target_include_directories (GRS_Simulator_tests PUBLIC tests)

target_link_libraries(
        GRS_Simulator_tests
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(GRS_Simulator_tests)
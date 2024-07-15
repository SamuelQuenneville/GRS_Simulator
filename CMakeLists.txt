cmake_minimum_required(VERSION 3.28)
project(GRS_Simulator)

set(CMAKE_CXX_STANDARD 17)

enable_testing()
include(FetchContent)

# Project files
set(GRS_CONFIGURATION
        source/Configuration/configurationParser.cpp
        source/Configuration/configurationParser.h
)

set(GRS_CONTROL
        source/Control/lowLevelController.cpp
        source/Control/lowLevelController.h
        source/Control/lowPassFilter.cpp
        source/Control/lowPassFilter.h
        source/Control/filter.h
        source/Control/slewLimiter.cpp
        source/Control/slewLimiter.h
        source/Control/pid.cpp
        source/Control/pid.h
        source/Control/pitchController.cpp
        source/Control/pitchController.h
        source/Control/llcBaseVersion.cpp
        source/Control/llcBaseVersion.h
        source/Control/rollController.cpp
        source/Control/rollController.h
        source/Control/altitudeController.cpp
        source/Control/altitudeController.h
        source/Control/highLevelController.cpp
        source/Control/highLevelController.h
        source/Control/groundspeedController.cpp
        source/Control/groundspeedController.h
        source/Control/hlcSingleAircraft.cpp
        source/Control/hlcSingleAircraft.h
)

set(GRS_DEFINITIONS
        source/Definitions/worldPhysics.h
        source/Definitions/tetherDefinitions.h
        source/Definitions/fixedWingDefinitions.h
        source/Definitions/configurationDefinitions.h
        source/Definitions/systemDefinitions.h
        source/Definitions/logDefinitions.h
        source/Definitions/yamlCustomStructure.h
)

set(GRS_LOG
        source/Log/logger.cpp
        source/Log/logger.h
)

set(GRS_MATHEMATICS
        source/Mathematics/grsMath.h
        source/Mathematics/matrix3.h
        source/Mathematics/vector2.h
        source/Mathematics/vector3.h
        source/Mathematics/vector4.h
        source/Mathematics/matrix4.h
        source/Mathematics/color.h
        source/Mathematics/quaternion.h
        source/Mathematics/quaternion.cpp
        source/Mathematics/transform.cpp
        source/Mathematics/transform.h
)

set(GRS_RIGID_BODIES_AND_LINK
        source/RigidBodyAndLink/node.cpp
        source/RigidBodyAndLink/node.h
        source/RigidBodyAndLink/rigidBody.cpp
        source/RigidBodyAndLink/rigidBody.h
        source/RigidBodyAndLink/fixedWing.cpp
        source/RigidBodyAndLink/fixedWing.h
        source/RigidBodyAndLink/link.cpp
        source/RigidBodyAndLink/link.h
        source/RigidBodyAndLink/tether.cpp
        source/RigidBodyAndLink/tether.h
)

set(GRS_SOURCE
        source/main.cpp
        source/world.cpp
        source/world.h
        source/system.cpp
        source/system.h
)

# Copy the Shaders used for the demo into the build directory
file(COPY "source/Shaders/" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/source/Shaders/")

# Copy the Meshes used for the demo into the build directory
file(COPY "source/Meshes/" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/source/Meshes/")

add_executable(GRS_Simulator
        ${GRS_CONFIGURATION}
        ${GRS_CONTROL}
        ${GRS_DEFINITIONS}
        ${GRS_LOG}
        ${GRS_MATHEMATICS}
        ${GRS_RIGID_BODIES_AND_LINK}
        ${GRS_SOURCE}
)

# Include libraries
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

# OpenGL
set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
set(NANOGUI_INSTALL       OFF CACHE BOOL " " FORCE)
set(NANOGUI_BACKEND       OpenGL CACHE BOOL " " FORCE)

SET(NANOGUI_BUILD_SHARED OFF CACHE BOOL "Build Nanogui with static libraries" FORCE)


add_subdirectory(source/Extern/nanogui)

set_property(TARGET glfw glfw_objects nanogui PROPERTY FOLDER "dependencies")

add_definitions(${NANOGUI_EXTRA_DEFS})
include_directories(${NANOGUI_EXTRA_INCS})

# Project
target_include_directories(GRS_Simulator PUBLIC source)

foreach(p ${NANOGUI_EXTRA_INCS})
    target_include_directories(GRS_Simulator PRIVATE $<1:${p}>)
endforeach()


# Compile definitions
target_compile_definitions(GRS_Simulator PRIVATE ${NANOGUI_EXTRA_DEFS})

# C++17 compiler features
target_compile_features(GRS_Simulator PUBLIC cxx_std_17)
set_target_properties(GRS_Simulator PROPERTIES CXX_EXTENSIONS OFF)

target_link_libraries(GRS_Simulator yaml-cpp::yaml-cpp nanogui ${NANOGUI_EXTRA_LIBS})

# GOOGLE TESTS
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)


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
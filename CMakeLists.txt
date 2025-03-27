cmake_minimum_required(VERSION 3.10)

# Project properties
set(PROJECT_NAME inekf)
set(PROJECT_DESCRIPTION "Invariant EKF")
set(PROJECT_URL https://github.com/edantec/invariant-ekf)
set(PROJECT_CUSTOM_HEADER_DIR "inekf")
set(PROJECT_CUSTOM_HEADER_EXTENSION "hpp")

# Project configuration
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_EXTENSIONS OFF)
set(PROJECT_USE_CMAKE_EXPORT True)

include(FetchContent)
include(CMakeDependentOption)

find_package(Eigen3 REQUIRED)

option(BUILD_PYTHON_INTERFACE "Build the python binding" ON)
cmake_dependent_option(
    GENERATE_PYTHON_STUBS
    "Generate Python stubs"
    OFF
    BUILD_PYTHON_INTERFACE
    ON
)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(EXISTS "${JRL_CMAKE_MODULES}/base.cmake")
    message(STATUS "JRL cmakemodules found in 'cmake/' git submodule")
else()
    find_package(jrl-cmakemodules QUIET CONFIG)
    if(jrl-cmakemodules_FOUND)
        get_property(
            JRL_CMAKE_MODULES
            TARGET jrl-cmakemodules::jrl-cmakemodules
            PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        )
        message(
            STATUS
            "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}"
        )
    elseif(${CMAKE_VERSION} VERSION_LESS "3.14.0")
        message(
            FATAL_ERROR
            "\nCan't find jrl-cmakemodules. Please either:\n"
            "  - use git submodule: 'git submodule update --init'\n"
            "  - or install https://github.com/jrl-umi3218/jrl-cmakemodules\n"
            "  - or upgrade your CMake to >= 3.14 to allow automatic fetching\n"
        )
    else()
        message(STATUS "JRL cmakemodules not found. Let's fetch it.")
        FetchContent_Declare(
            "jrl-cmakemodules"
            GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git"
        )
        FetchContent_MakeAvailable("jrl-cmakemodules")
        FetchContent_GetProperties(
            "jrl-cmakemodules"
            SOURCE_DIR JRL_CMAKE_MODULES
        )
    endif()
endif()

# The following was copied over from
# https://github.com/Simple-Robotics/aligator: Use BoostConfig module
# distributed by boost library instead of using FindBoost module distributed by
# CMake
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
    # find_package(SDFormat14) reset this policy (with cmake_minimum_required) and
    # redefine the find_dependency macro. Then, futur call of this macro don't use
    # the right version of this policy. To avoid that, we define the CMP0167
    # default value.
    set(CMAKE_POLICY_DEFAULT_CMP0167 NEW)
endif()
# JRL-cmakemodule setup
include("${JRL_CMAKE_MODULES}/base.cmake")
include("${JRL_CMAKE_MODULES}/boost.cmake")

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

add_compile_options(-Wno-error)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(
        CACHE CMAKE_BUILD_TYPE
        PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo"
    )
endif()

# Main Library
file(GLOB iekf_SOURCE CONFIGURE_DEPENDS src/*.cpp)
file(GLOB iekf_HEADER CONFIGURE_DEPENDS include/${PROJECT_NAME}/*.hpp)

add_library(${PROJECT_NAME} SHARED ${iekf_HEADER} ${iekf_SOURCE})
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${iekf_HEADER}")
set_target_properties(
    ${PROJECT_NAME}
    PROPERTIES LINKER_LANGUAGE CXX INSTALL_RPATH "\$ORIGIN"
)

# Bindings
if(BUILD_PYTHON_INTERFACE)
    add_subdirectory(bindings)
endif()

# Installation
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR}
)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

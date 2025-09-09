# Copyright(C) 2024 eSOL Co., Ltd.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(rmw_zenoh_demos_rclc)

find_package(ament_cmake REQUIRED)

include(ExternalProject)
include(CMakeParseArguments)

if(ANDROID)
    set(CROSS_CMAKE_ARGS
        -DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}
        -DCMAKE_SYSTEM_VERSION:STRING=${CMAKE_SYSTEM_VERSION}
        -DCMAKE_ANDROID_ARCH_ABI:STRING=${CMAKE_ANDROID_ARCH_ABI}
        -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE:STRING=BOTH
        )
endif()

function(export_executable)
    set(subtree "")
    foreach(arg IN LISTS ARGN)
        set(subtree ${subtree}/${arg})
    endforeach()
    get_filename_component(leaf ${subtree} NAME)
    ExternalProject_Add(${leaf}
        PREFIX
            ${PROJECT_BINARY_DIR}${subtree}
        SOURCE_DIR
            ${PROJECT_SOURCE_DIR}${subtree}
        INSTALL_DIR
            ${PROJECT_BINARY_DIR}/temp_install
        BUILD_ALWAYS 1
        CMAKE_CACHE_ARGS
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
            -DCMAKE_PREFIX_PATH:PATH=${CMAKE_PREFIX_PATH}
            -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
            ${CROSS_CMAKE_ARGS}
    )
endfunction()

export_executable(listener)
export_executable(talker)
export_executable(client)
export_executable(server)

if(EXISTS ${PROJECT_BINARY_DIR}/temp_install/)
    install(
        DIRECTORY
            ${PROJECT_BINARY_DIR}/temp_install/
        DESTINATION
            lib/${PROJECT_NAME}
        USE_SOURCE_PERMISSIONS
        )
endif()

ament_package()

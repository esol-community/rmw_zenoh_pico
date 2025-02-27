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

project(talker LANGUAGES C)

find_package(ament_cmake REQUIRED)
find_package(microcdr REQUIRED)
find_package(rclc REQUIRED)
find_package(rmw_zenoh_pico REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(${PROJECT_NAME} main.c)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-result")

ament_target_dependencies(${PROJECT_NAME}
  rclc
  microcdr
  rmw_zenoh_pico
  std_msgs
)

target_include_directories(${PROJECT_NAME}
  PRIVATE ${CMAKE_INSTALL_PREFIX}/include/rcl_action
  PRIVATE ${CMAKE_INSTALL_PREFIX}/include/action_msgs
  PRIVATE ${CMAKE_INSTALL_PREFIX}/include/unique_identifier_msgs
  )

target_link_libraries(${PROJECT_NAME}
  atomic
)

install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION
    ${PROJECT_NAME}
  )

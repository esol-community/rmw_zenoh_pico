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

project(client LANGUAGES C)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(example_interfaces REQUIRED)

add_compile_options(-Wno-deprecated-declarations -Wno-unused-result)

add_executable(${PROJECT_NAME} main.c)

ament_target_dependencies(${PROJECT_NAME}
  rcl
  rclc
  example_interfaces
  )

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${PROJECT_NAME}
  )

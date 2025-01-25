#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0057 NEW)

set(PACKAGE_VERSION "0.0.1")
project(rmw_zenoh_pico
  VERSION
    ${PACKAGE_VERSION}
  LANGUAGES
    C
)

find_package(ament_cmake_ros REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(microcdr REQUIRED)
find_package(zenohpico REQUIRED)

# Build options
option(BUILD_DOCUMENTATION              "Use doxygen to create product documentation" OFF)

# --- RMW CONFIGURATION ---
set(RMW_ZENOH_PICO_TRANSPORT            "unicast"      CACHE STRING "Sets zenoh pico transport to use. (unicast | serial)")
set(RMW_ZENOH_PICO_TRANSPORT_MODE       "client"       CACHE STRING "zenoh transport connect mode")
set(RMW_ZENOH_PICO_CONNECT              "127.0.0.1"    CACHE STRING "Sets the scout address.")
set(RMW_ZENOH_PICO_CONNECT_PORT         "7447"         CACHE STRING "Sets the scout port.")
set(RMW_ZENOH_PICO_LISTEN               "127.0.0.1"    CACHE STRING "Sets the listen address.")
set(RMW_ZENOH_PICO_LISTEN_PORT          "-1"           CACHE STRING "Sets the listen port.")

set(RMW_ZENOH_PICO_SERIAL_DEVICE        "/dev/ttyAMA0" CACHE STRING "Sets the agent serial port.")
set(RMW_ZENOH_PICO_MAX_LINENESS_LEN     "256"          CACHE STRING "This value sets the number of max liveliness resource length")

set(RMW_ZENOH_PICO_C_STANDARD           "99"           CACHE STRING  "Version of the C language used to build the library")

# Transport handle define macros.
set(RMW_ZENOH_PICO_TRANSPORT_SERIAL  OFF)
set(RMW_ZENOH_PICO_TRANSPORT_UNICAST OFF)

if(${RMW_ZENOH_PICO_TRANSPORT} STREQUAL "serial")
  set(RMW_ZENOH_PICO_TRANSPORT_SERIAL ON)
elseif(${RMW_ZENOH_PICO_TRANSPORT} STREQUAL "unicast")
  set(RMW_ZENOH_PICO_TRANSPORT_UNICAST ON)
else()
  message(FATAL_ERROR "Transport not supported. Use \"unicast\", \"serial\"")
endif()

# Set install directories
if(WIN32)
    set(DOC_DIR "doc")
else()
    set(DOC_DIR "${DATA_INSTALL_DIR}/doc")
endif()

set(DOC_INSTALL_DIR ${DOC_DIR} CACHE PATH "Installation directory for documentation")

set(SRCS
  src/zenoh_pico/zenoh_pico_topicInfo.c
  src/zenoh_pico/zenoh_pico_entity.c
  src/zenoh_pico/zenoh_pico_liveliness.c
  src/zenoh_pico/zenoh_pico_nodeInfo.c
  src/zenoh_pico/zenoh_pico_receiveMessage.c
  src/zenoh_pico/zenoh_pico_messageType.c
  src/zenoh_pico/zenoh_pico_string.c
  src/zenoh_pico/zenoh_pico_condition.c

  src/utilities/test_qos_profile.c

  src/rmw_client.c
  src/rmw_compare_gids_equal.c
  src/rmw_count.c
  src/rmw_event.c
  src/rmw_features.c
  src/rmw_get_gid_for_publisher.c
  src/rmw_get_gid_for_client.c
  src/rmw_get_implementation_identifier.c
  src/rmw_get_topic_endpoint_info.c
  src/rmw_get_endpoint_network_flow.c
  src/rmw_qos_profile_check_compatible.c
  src/rmw_dynamic_message_type_support.c
  src/rmw_guard_condition.c
  src/rmw_init.c
  src/rmw_logging.c
  src/rmw_node.c
  src/rmw_node_info_and_types.c
  src/rmw_node_names.c
  src/rmw_publish.c
  src/rmw_publisher.c
  src/rmw_request.c
  src/rmw_response.c
  src/rmw_serialize.c
  src/rmw_service.c
  src/rmw_service_names_and_types.c
  src/rmw_service_server_is_available.c
  src/rmw_subscription.c
  src/rmw_take.c
  src/rmw_topic_names_and_types.c
  src/rmw_trigger_guard_condition.c
  src/rmw_wait.c
  src/rmw_wait_set.c
  src/rmw_event_callbacks.c
)

add_library(${PROJECT_NAME}
  ${SRCS})

target_link_libraries(${PROJECT_NAME}
  microcdr
)

# Type support lock-up mechanism
find_package(rosidl_typesupport_microxrcedds_c)
if(rosidl_typesupport_microxrcedds_c_FOUND)
  ament_target_dependencies(${PROJECT_NAME}
    "rosidl_typesupport_microxrcedds_c"
)
  target_compile_definitions(${PROJECT_NAME}
    PUBLIC
      HAVE_C_TYPESUPPORT
)
else()
  find_package(rosidl_typesupport_microxrcedds_cpp REQUIRED)
endif()

if(rosidl_typesupport_microxrcedds_cpp_FOUND)
  ament_target_dependencies(${PROJECT_NAME}
    "rosidl_typesupport_microxrcedds_cpp"
)
  target_compile_definitions(${PROJECT_NAME}
    PUBLIC
      HAVE_CPP_TYPESUPPORT
)
endif()

ament_target_dependencies(${PROJECT_NAME}
  "zenohpico"
  "rcutils"
  "rmw"
)

configure_rmw_library(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
  PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  C_STANDARD
    ${RMW_ZENOH_PICO_C_STANDARD}
  C_STANDARD_REQUIRED
    YES
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE
  ZENOH_DEBUG_ENABLE
)

target_compile_options(${PROJECT_NAME}
  PRIVATE
    $<$<C_COMPILER_ID:GNU>:-Wall>
    $<$<C_COMPILER_ID:GNU>:-Wextra>
    $<$<C_COMPILER_ID:GNU>:-pedantic>
    $<$<C_COMPILER_ID:GNU>:-Werror-implicit-function-declaration>
    $<$<C_COMPILER_ID:GNU>:-Wno-unused-parameter>
    $<$<C_COMPILER_ID:GNU>:-Wno-unused-variable>
    $<$<C_COMPILER_ID:GNU>:-Wno-unused-function>
    $<$<C_COMPILER_ID:GNU>:-Wno-pedantic>
    $<$<C_COMPILER_ID:GNU>:$<$<VERSION_GREATER:$<C_COMPILER_VERSION>,8.1>:-Wcast-align=strict>>
    $<$<C_COMPILER_ID:GNU>:-g>
    $<$<C_COMPILER_ID:GNU>:-O0>
    $<$<C_COMPILER_ID:MSVC>:/Wall>
)

file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/include)

ament_export_include_directories(${CMAKE_INSTALL_PREFIX}/include)
ament_export_include_directories(${microcdr_INCLUDE_DIR})
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  zenohpico
  rcutils
  rmw
)

# ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(rosidl_typesupport_microxrcedds_c_FOUND)
  ament_export_dependencies(rosidl_typesupport_microxrcedds_c)
  set(implementations "c:rosidl_typesupport_c:rosidl_typesupport_microxrcedds_c")
endif()
if(rosidl_typesupport_microxrcedds_cpp_FOUND)
  ament_export_dependencies(rosidl_typesupport_microxrcedds_cpp)
  set(implementations
    ${implementations}
    "cpp:rosidl_typesupport_cpp:rosidl_typesupport_microxrcedds_cpp"
)
endif()

register_rmw_implementation(${implementations})

if(BUILD_DEMO)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(demo/host)
endif()

# Create source files with the define
configure_file(${PROJECT_SOURCE_DIR}/src/config.h.in
  ${PROJECT_BINARY_DIR}/include/rmw_zenoh_pico/config.h)

# Documentation
if(BUILD_DOCUMENTATION)
    find_package(Doxygen)
    if(NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "doxygen is needed to build the documentation. Please install it correctly")
    endif()
    if(UNIX)
        find_program(DOXYFILE_MAKE make)
        if(DOXYFILE_MAKE)
            message(STATUS "Found Make: ${DOXYFILE_MAKE}")
        else()
            message(FATAL_ERROR "make is needed to build the documentation. Please install it correctly")
        endif()
    elseif(WIN32)
        set(DOXYFILE_MAKE make.bat)
    endif()

    # Target to create documentation directories
    file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/doc)
    add_custom_target(docdirs
        COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/doc/api_reference
        COMMENT "Creating documentation directory" VERBATIM)

    ### Doxygen ########################
    # Configure the template doxyfile for or specific project
    configure_file(doxyfile.in ${PROJECT_BINARY_DIR}/doxyfile @ONLY IMMEDIATE)
    # Add custom target to run doxygen when ever the project is build
    add_custom_target(doxygen
        COMMAND "${DOXYGEN_EXECUTABLE}" "${PROJECT_BINARY_DIR}/doxyfile"
        SOURCES "${PROJECT_BINARY_DIR}/doxyfile"
        COMMENT "Generating API documentation with doxygen" VERBATIM)

    add_dependencies(doxygen docdirs)

    add_custom_target(doc ALL
    COMMENT "Generated project documentation" VERBATIM)

    add_dependencies(doc doxygen)
endif()

ament_package()

# Install includes.
install(
  DIRECTORY
    ${PROJECT_SOURCE_DIR}/include/
  DESTINATION
    include
)

# Install config.h file.
install(
  FILES
    ${PROJECT_BINARY_DIR}/include/rmw_zenoh_pico/config.h
  DESTINATION
    include/rmw_zenoh_pico
)

# Install library.
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION
    lib
  LIBRARY DESTINATION
    lib
  RUNTIME DESTINATION
    bin
)

if(BUILD_DOCUMENTATION)
  # Instalation of doxygen files
  install(DIRECTORY ${PROJECT_BINARY_DIR}/doc/api_reference
      DESTINATION ${DOC_INSTALL_DIR}
      COMPONENT documentation)
endif()

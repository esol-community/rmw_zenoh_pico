// Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Copyright(C) 2024 eSOL Co., Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

const char * const zenoh_pico_identifier = "rmw_zenoh_pico";
const char * const zenoh_pico_serialization_format = "cdr";
const char * const rosidl_typesupport_rmw_zenoh_identifier = "rosidl_typesupport_microxrcedds_c";

const char *
rmw_get_implementation_identifier(void)
{
  return zenoh_pico_identifier;
}

const char *
rmw_get_serialization_format(void)
{
  return zenoh_pico_serialization_format;
}

const char *
rmw_zenoh_pico_typesupport_c(void)
{
  return rosidl_typesupport_rmw_zenoh_identifier;
}

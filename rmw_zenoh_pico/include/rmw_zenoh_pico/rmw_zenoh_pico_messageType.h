#ifndef ZENOH_PICO_MESSAGETYPE_H
#define ZENOH_PICO_MESSAGETYPE_H

#include <rmw/rmw.h>
#include <rmw/types.h>

#include <ucdr/microcdr.h>

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_typesupport_microxrcedds_c/message_type_support.h>

extern const rosidl_message_type_support_t * find_message_type_support(
  const rosidl_message_type_support_t * type_supports);

#endif

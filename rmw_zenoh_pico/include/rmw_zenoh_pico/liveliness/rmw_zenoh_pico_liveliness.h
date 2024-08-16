#ifndef RMW_ZENOH_PICO_LIVELINESS_H
#define RMW_ZENOH_PICO_LIVELINESS_H

#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_entity.h"

#include <rmw/rmw.h>
#include <rmw/types.h>
#include <rmw/allocators.h>
#include <rmw/error_handling.h>

extern size_t generate_liveliness(ZenohPicoEntity *entity, char *buf, size_t size);

extern z_string_t conv_domain(size_t domain);
extern z_string_t convert_hash(const rosidl_type_hash_t * type_hash);
extern z_string_t convert_message_type(const message_type_support_callbacks_t *callbacks);
extern z_string_t qos_to_keyexpr(rmw_qos_profile_t *qos);

#endif

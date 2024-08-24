
#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

const rosidl_message_type_support_t * find_message_type_support(
  const rosidl_message_type_support_t * type_supports)
{
  const rosidl_message_type_support_t * type_support = get_message_typesupport_handle(
    type_supports, RMW_ZENOH_PICO_TYPESUPPORT_C);
  if (!type_support) {
    rcutils_error_string_t error_string = rcutils_get_error_string();
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Type support not from this implementation. Got:\n"
      "    %s\n"
      "while fetching it",
      error_string.str);
    return NULL;
  }

  return type_support;
}

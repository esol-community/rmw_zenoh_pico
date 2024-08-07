#include <stdio.h>
#include <string.h>

#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_param.h"

ZenohPicoTransportParams *zenoh_pico_generate_param(ZenohPicoTransportParams *param)
{
  ZenohPicoGenerateData(param, ZenohPicoTransportParams);

  return param;
}

bool zenoh_pico_destroy_param(ZenohPicoTransportParams *param)
{
  ZenohPicoDestroyData(param);

  return true;
}

bool zenoh_pico_clone_param(ZenohPicoTransportParams *dst, ZenohPicoTransportParams *src)
{
  if ((dst == NULL) || (src == NULL))
    return false;

  memcpy(dst, src, sizeof(ZenohPicoTransportParams));

  return true;
}

#include <rmw_zenoh_pico/config.h>

#include <rmw/allocators.h>
#include <rmw/rmw.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

ZenohPicoWaitData * zenoh_pico_generate_wait_data(rmw_context_t * context)
{

  ZenohPicoWaitData *wait_data = NULL;
  ZenohPicoGenerateData(wait_data, ZenohPicoWaitData);
  if(wait_data == NULL)
    return NULL;

  z_mutex_init(&wait_data->condition_mutex);
  z_condvar_init(&wait_data->condition_variable);
  wait_data->triggered = false;

  wait_data->context = context;

  return wait_data;
}

bool zenoh_pico_destroy_wait_data(ZenohPicoWaitData *wait_data)
{
  z_mutex_free(&wait_data->condition_mutex);
  z_condvar_free(&wait_data->condition_variable);

  ZenohPicoDestroyData(wait_data);

  return true;
}

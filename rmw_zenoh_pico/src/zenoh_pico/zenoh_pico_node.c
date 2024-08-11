
#include <stdio.h>
#include <string.h>

#include <rmw/allocators.h>
#include <rmw/rmw.h>

#include "rmw_zenoh_pico/rmw_zenoh_pico_logging.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_node.h"
#include "zenoh-pico/api/macros.h"
#include "zenoh-pico/api/primitives.h"

ZenohPicoNodeData * zenoh_pico_generate_node_data(ZenohPicoNodeData *node_data,
						  ZenohPicoSession *session,
						  ZenohPicoEntity *entity)
{
  if((session == NULL) || (entity == NULL))
    return NULL;

  ZenohPicoGenerateData(node_data, ZenohPicoNodeData);
  if(node_data == NULL)
    return NULL;

  node_data->session_ = session;
  node_data->entity_ = entity;

  // generate key from entity data
  {
    char _buf[128];
    int ret = generate_liveliness(entity, _buf, sizeof(_buf));
    if(ret >= (int)(sizeof(_buf) -1))
      return NULL;

    node_data->key_ =  z_string_make(_buf);
  }

  return node_data;
}

bool zenoh_pico_destroy_node_data(ZenohPicoNodeData *node_data)
{
  _Z_DEBUG("%s : start (%p)", __func__, node_data);

  Z_STRING_FREE(node_data->key_);

  if (z_check(node_data->keyexpr_)) {
      ZenohPicoSession *session = node_data->session_;
      z_undeclare_keyexpr(z_loan(session->z_session_), &node_data->keyexpr_);
  }

  // delete entitiy
  if(node_data->entity_ != NULL){
    zenoh_pico_destroy_entitiy(node_data->entity_);
    node_data->entity_ = NULL;
  }

  ZenohPicoDestroyData(node_data);

  return true;
}

void zenoh_pico_debug_node_data(ZenohPicoNodeData *node_data)
{
  printf("--------- node data ----------\n");
  printf("is_alloc = %d\n", node_data->is_alloc_);

  Z_STRING_PRINTF(node_data->key_, keyexpr);

  z_keyexpr_t *value = node_data->keyexpr_._value;
  if(value != NULL){
    printf("z_keyexpr id = %d\n", value->_id);
    printf("z_keyexpr mapping = %x\n", value->_mapping._val);
    printf("z_keyexpr suffix = %s\n", value->_suffix);
  } else {
    printf("value is zero\n");
  }

  // debug entity member
  zenoh_pico_debug_entitiy(node_data->entity_);
}

rmw_node_t * rmw_node_generate(rmw_context_t *context, ZenohPicoNodeData *node_data)
{

  if(node_data->entity_->node_info_ == NULL)
    return NULL;

  ZenohPicoNodeInfo_t *node_info = node_data->entity_->node_info_;

  rmw_node_t * node = z_malloc(sizeof(rmw_node_t));
  if(node == NULL)
    return NULL;

  memset(node, 0, sizeof(rmw_node_t));

  node->name				= node_info->name_.val;
  node->namespace_			= node_info->ns_.val;
  node->data				= (void *)node_data;
  node->implementation_identifier	= rmw_get_implementation_identifier();
  node->context				= context;

  return node;
}

rmw_ret_t rmw_node_destroy(rmw_node_t * node)
{
  _Z_DEBUG("%s : start(%p)", __func__, node);

  if(node != NULL){
    ZenohPicoNodeData *node_data = (ZenohPicoNodeData *)node->data;
    if(node_data != NULL){
      zenoh_pico_destroy_node_data(node_data);
      node_data = NULL;
    }

    z_free(node);
  }

  return RMW_RET_OK;
}

//-----------------------------

bool declaration_node_data(ZenohPicoNodeData *node_data)
{
  ZenohPicoSession *session = node_data->session_;
  const char *keyexpr = Z_STRING_VAL(node_data->key_);

  _Z_DEBUG("Declaring key expression '%s'...\n", keyexpr);
  node_data->keyexpr_ = z_declare_keyexpr(z_loan(session->z_session_), z_keyexpr(keyexpr));
  if (!z_check(node_data->keyexpr_)) {
    return false;
  }

  zenoh_pico_debug_node_data(node_data);

  return true;
}

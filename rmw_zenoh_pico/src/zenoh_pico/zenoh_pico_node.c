
#include <stdio.h>
#include <string.h>

#include <rmw/allocators.h>
#include <rmw/rmw.h>

#include "rmw_zenoh_pico/rmw_zenoh_pico_logging.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_node.h"
#include "zenoh-pico/api/macros.h"
#include "zenoh-pico/api/primitives.h"

ZenohPicoNodeData * zenoh_pico_generate_node_data(size_t node_id,
						  ZenohPicoSession *session,
						  ZenohPicoEntity *entity)
{
  _Z_DEBUG("%s : start", __func__);

  if((session == NULL) || (entity == NULL))
    return NULL;

  ZenohPicoNodeData *node_data = NULL;
  ZenohPicoGenerateData(node_data, ZenohPicoNodeData);
  if(node_data == NULL)
    return NULL;

  node_data->session_	= session;
  node_data->entity_	= entity;
  node_data->id_	= node_id;

  // generate key from entity data
  node_data->token_key_ = generate_liveliness(entity);

  return node_data;
}

ZenohPicoNodeData *zenoh_pico_loan_node_data(ZenohPicoNodeData *node_data)
{
  // ATTENTION :
  // I have not implemented smart pointer, yet.

  return node_data;
}

bool zenoh_pico_destroy_node_data(ZenohPicoNodeData *node_data)
{
  _Z_DEBUG("%s : start", __func__);

  (void)undeclaration_node_data(node_data);

  Z_STRING_FREE(node_data->token_key_);

  if (z_check(node_data->token_)) {
      ZenohPicoSession *session = node_data->session_;
      z_undeclare_keyexpr(z_loan(session->session_), &node_data->token_);
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
  printf("ref = %d\n", node_data->ref_);

  Z_STRING_PRINTF(node_data->token_key_, token_key);

  z_keyexpr_t *value = node_data->token_._value;
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

//-----------------------------

bool declaration_node_data(ZenohPicoNodeData *node_data)
{
  ZenohPicoSession *session = node_data->session_;
  const char *keyexpr = Z_STRING_VAL(node_data->token_key_);

  _Z_DEBUG("Declaring node key expression '%s'...", keyexpr);
  node_data->token_ = z_declare_keyexpr(z_loan(session->session_), z_keyexpr(keyexpr));
  if (!z_check(node_data->token_)) {
    return false;
  }

  zenoh_pico_debug_node_data(node_data);

  return true;
}

bool undeclaration_node_data(ZenohPicoNodeData *node_data)
{
  ZenohPicoSession *session = node_data->session_;

  if (z_check(node_data->token_)) {
    z_undeclare_keyexpr(z_loan(session->session_), &node_data->token_);
  }

  return true;
}

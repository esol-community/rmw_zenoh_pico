
#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"

#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_nodeInfo.h"
#include "zenoh-pico/collections/string.h"

const char *node_domain(ZenohPicoNodeInfo_t *node)      { return Z_STRING_VAL(node->domain_); }
const char *node_namespace(ZenohPicoNodeInfo_t *node)	{ return Z_STRING_VAL(node->ns_); }
const char *node_name(ZenohPicoNodeInfo_t *node)	{ return Z_STRING_VAL(node->name_); }
const char *node_enclave(ZenohPicoNodeInfo_t *node)	{ return Z_STRING_VAL(node->enclave_); }

static void _zenoh_pico_clear_node_info_member(ZenohPicoNodeInfo_t *node)
{
  Z_STRING_FREE(node->domain_);
  Z_STRING_FREE(node->ns_);
  Z_STRING_FREE(node->name_);
  Z_STRING_FREE(node->enclave_);
}

ZenohPicoNodeInfo_t * zenoh_pico_generate_node_info(ZenohPicoNodeInfo_t *node,
						    z_string_t *domain,
						    z_string_t *ns,
						    z_string_t *name,
						    z_string_t *enclave)
{
  ZenohPicoGenerateData(node, ZenohPicoNodeInfo_t);

  if(node == NULL)
    return NULL;

  _zenoh_pico_clear_node_info_member(node);

  _z_string_move(&node->domain_, domain);
  _z_string_move(&node->ns_, ns);
  _z_string_move(&node->name_, name);
  _z_string_move(&node->enclave_, enclave);

  return node;
}

bool zenoh_pico_destroy_node_info(ZenohPicoNodeInfo_t *node)
{
  _zenoh_pico_clear_node_info_member(node);

  ZenohPicoDestroyData(node);

  return true;
}

void zenoh_pico_clone_node_info(ZenohPicoNodeInfo_t *dst, ZenohPicoNodeInfo_t *src)
{
  _zenoh_pico_clear_node_info_member(dst);

  _z_string_copy(&dst->domain_, &src->domain_);
  _z_string_copy(&dst->ns_, &src->ns_);
  _z_string_copy(&dst->name_, &src->name_);
  _z_string_copy(&dst->enclave_, &src->enclave_);
}

void zenoh_pico_debug_node_info(ZenohPicoNodeInfo_t *node)
{
  printf("node info ...\n");
  printf("\tdomain        = %s\n", node->domain_.val);
  printf("\tnamespace     = %s\n", node->ns_.val);
  printf("\tname          = %s\n", node->name_.val);
  printf("\tenclave_      = %s\n", node->enclave_.val);
}

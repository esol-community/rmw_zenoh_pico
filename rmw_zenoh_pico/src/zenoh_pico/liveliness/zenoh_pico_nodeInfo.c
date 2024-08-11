
#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"

#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_nodeInfo.h"

size_t node_domain_id(ZenohPicoNodeInfo_t *node)      { return node->domain_id_; }
const char *node_enclave(ZenohPicoNodeInfo_t *node)   { return Z_STRING_VAL(node->enclave_); }
const char *node_namespace(ZenohPicoNodeInfo_t *node) { return Z_STRING_VAL(node->ns_); }
const char *node_name(ZenohPicoNodeInfo_t *node)      { return Z_STRING_VAL(node->name_); }

ZenohPicoNodeInfo_t * zenoh_pico_generate_node_info(ZenohPicoNodeInfo_t *node,
						    size_t domain_id,
						    const char *ns,
						    const char *name,
						    const char *enclave)
{
  ZenohPicoGenerateData(node, ZenohPicoNodeInfo_t);

  if(node == NULL)
    return NULL;

  node->domain_id_	= domain_id;
  node->ns_		= (ns == NULL)        ? _z_string_make("") : _z_string_make(ns);
  node->name_		= (name == NULL)      ? _z_string_make("") : _z_string_make(name);
  node->enclave_	= (enclave == NULL)   ? _z_string_make("") : _z_string_make(enclave);

  return node;
}

static void _zenoh_pico_clear_node_info_member(ZenohPicoNodeInfo_t *node)
{
  Z_STRING_FREE(node->ns_);
  Z_STRING_FREE(node->name_);
  Z_STRING_FREE(node->enclave_);
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

  dst->domain_id_ = src->domain_id_;
  _z_string_copy(&dst->ns_, &src->ns_);
  _z_string_copy(&dst->name_, &src->name_);
  _z_string_copy(&dst->enclave_, &src->enclave_);
}

void zenoh_pico_debug_node_info(ZenohPicoNodeInfo_t *node)
{
  printf("node info ...\n");
  printf("\tdomain_id     = %ld\n", node->domain_id_);
  printf("\tnamespace     = %s\n", node->ns_.val);
  printf("\tname          = %s\n", node->name_.val);
  printf("\tenclave_      = %s\n", node->enclave_.val);
}

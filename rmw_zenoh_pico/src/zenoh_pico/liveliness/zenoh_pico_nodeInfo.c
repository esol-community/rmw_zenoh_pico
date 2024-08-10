
#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"

#include "rmw_zenoh_pico/liveliness/rmw_zenoh_pico_nodeInfo.h"

const _z_string_t *node_domain(ZenohPicoNodeInfo_t *node)	{ return &node->domain_id_; }
const _z_string_t *node_namespace(ZenohPicoNodeInfo_t *node)	{ return &node->ns_; }
const _z_string_t *node_name(ZenohPicoNodeInfo_t *node)		{ return &node->name_; }
const _z_string_t *node_enclave(ZenohPicoNodeInfo_t *node)	{ return &node->enclave_; }

ZenohPicoNodeInfo_t * zenoh_pico_generate_node_info(ZenohPicoNodeInfo_t *node,
						    const char *domain_id,
						    const char *ns,
						    const char *name,
						    const char *enclave)
{
  ZenohPicoGenerateData(node, ZenohPicoNodeInfo_t);

  if(node == NULL)
    return NULL;

  node->domain_id_	= (domain_id == NULL) ? _z_string_make("") : _z_string_make(domain_id);
  node->ns_		= (ns == NULL)        ? _z_string_make("") : _z_string_make(ns);
  node->name_		= (name == NULL)      ? _z_string_make("") : _z_string_make(name);
  node->enclave_	= (enclave == NULL)   ? _z_string_make("") : _z_string_make(enclave);

  return node;
}

static void _zenoh_pico_clear_node_info_member(ZenohPicoNodeInfo_t *node)
{
  Z_STRING_FREE(node->domain_id_);
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

  _z_string_copy(&dst->domain_id_, &src->domain_id_);
  _z_string_copy(&dst->ns_, &src->ns_);
  _z_string_copy(&dst->name_, &src->name_);
  _z_string_copy(&dst->enclave_, &src->enclave_);
}

void zenoh_pico_debug_node_info(ZenohPicoNodeInfo_t *node)
{
  printf("node info ...\n");
  printf("\tdomain_id     = %s\n", node->domain_id_.val);
  printf("\tnamespace     = %s\n", node->ns_.val);
  printf("\tname          = %s\n", node->name_.val);
  printf("\tenclave_      = %s\n", node->enclave_.val);
}

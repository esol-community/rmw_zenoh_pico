
#include <stdio.h>
#include <string.h>

#include "rmw_zenoh_pico/rmw_zenoh_pico_macros.h"
#include "rmw_zenoh_pico/rmw_zenoh_pico_node.h"

#include "zenoh-pico/collections/string.h"

const _z_string_t *node_domain(ZenohPicoNodeInfo_t *node)	{ return &node->domain_id_; }
const _z_string_t *node_namespace(ZenohPicoNodeInfo_t *node)	{ return &node->ns_; }
const _z_string_t *node_name(ZenohPicoNodeInfo_t *node)		{ return &node->name_; }
const _z_string_t *node_enclave(ZenohPicoNodeInfo_t *node)	{ return &node->enclave_; }

ZenohPicoNodeInfo_t * zenoh_pico_generate_node(ZenohPicoNodeInfo_t *node,
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

static void _zenoh_pico_clear_node_member(ZenohPicoNodeInfo_t *node)
{
  if (node->domain_id_.len != 0) _z_string_clear(&node->domain_id_);
  if (node->ns_.len != 0)	 _z_string_clear(&node->ns_);
  if (node->name_.len != 0)	 _z_string_clear(&node->name_);
  if (node->enclave_.len != 0)	 _z_string_clear(&node->enclave_);
}

bool zenoh_pico_destroy_node(ZenohPicoNodeInfo_t *node)
{
  _zenoh_pico_clear_node_member(node);

  ZenohPicoDestroyData(node);

  return true;
}

void zenoh_pico_clone_node(ZenohPicoNodeInfo_t *dst, ZenohPicoNodeInfo_t *src)
{
  _zenoh_pico_clear_node_member(dst);

  _z_string_copy(&dst->domain_id_, &src->domain_id_);
  _z_string_copy(&dst->ns_, &src->ns_);
  _z_string_copy(&dst->name_, &src->name_);
  _z_string_copy(&dst->enclave_, &src->enclave_);
}

void zenoh_pico_debug_node(ZenohPicoNodeInfo_t *node)
{
  printf("node info ...\n");
  printf("\tdomain_id     = %s\n", node->domain_id_.val);
  printf("\tnamespace     = %s\n", node->ns_.val);
  printf("\tname          = %s\n", node->name_.val);
  printf("\tenclave_      = %s\n", node->enclave_.val);
}

//-----------------------------

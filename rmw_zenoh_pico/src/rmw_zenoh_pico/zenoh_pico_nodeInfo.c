/*
 * Copyright(C) 2024 eSOL Co., Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

const char *node_domain(ZenohPicoNodeInfo *node)      { return Z_STRING_VAL(node->domain); }
const char *node_namespace(ZenohPicoNodeInfo *node)	{ return Z_STRING_VAL(node->ns); }
const char *node_name(ZenohPicoNodeInfo *node)	{ return Z_STRING_VAL(node->name); }
const char *node_enclave(ZenohPicoNodeInfo *node)	{ return Z_STRING_VAL(node->enclave); }

z_mutex_t mutex_ZenohPicoNodeInfo;

static void _zenoh_pico_clear_node_info_member(ZenohPicoNodeInfo *node)
{
  Z_STRING_FREE(node->domain);
  Z_STRING_FREE(node->ns);
  Z_STRING_FREE(node->name);
  Z_STRING_FREE(node->enclave);
}

ZenohPicoNodeInfo * zenoh_pico_generate_node_info(z_string_t *domain,
						  z_string_t *ns,
						  z_string_t *name,
						  z_string_t *enclave)
{
  ZenohPicoNodeInfo *node = NULL;
  ZenohPicoGenerateData(node, ZenohPicoNodeInfo);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node,
    "failed to allocate struct for the ZenohPicoNodeInfo",
    return NULL);

  _zenoh_pico_clear_node_info_member(node);

  _z_string_move(&node->domain, domain);
  _z_string_move(&node->ns, ns);
  _z_string_move(&node->name, name);
  _z_string_move(&node->enclave, enclave);

  return node;
}

ZenohPicoNodeInfo *zenoh_pico_clone_node_info(ZenohPicoNodeInfo *src)
{
  ZenohPicoNodeInfo *node = NULL;
  ZenohPicoGenerateData(node, ZenohPicoNodeInfo);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node,
    "failed to allocate struct for the ZenohPicoNodeInfo",
    return NULL);

  _z_string_copy(&node->domain, &src->domain);
  _z_string_copy(&node->ns, &src->ns);
  _z_string_copy(&node->name, &src->name);
  _z_string_copy(&node->enclave, &src->enclave);

  return node;
}

bool zenoh_pico_destroy_node_info(ZenohPicoNodeInfo *node)
{
  _zenoh_pico_clear_node_info_member(node);

  ZenohPicoDestroyData(node, ZenohPicoNodeInfo);

  return true;
}


void zenoh_pico_debug_node_info(ZenohPicoNodeInfo *node)
{
  printf("node info ...\n");
  printf("\tdomain        = %s\n", node->domain.val);
  printf("\tnamespace     = %s\n", node->ns.val);
  printf("\tname          = %s\n", node->name.val);
  printf("\tenclave_      = %s\n", node->enclave.val);
}

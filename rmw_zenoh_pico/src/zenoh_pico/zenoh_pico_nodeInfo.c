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

const z_loaned_string_t *node_domain(ZenohPicoNodeInfo *node)		{ return z_loan(node->domain); }
const z_loaned_string_t *node_namespace(ZenohPicoNodeInfo *node)	{ return z_loan(node->ns); }
const z_loaned_string_t *node_name(ZenohPicoNodeInfo *node)		{ return z_loan(node->name); }
const z_loaned_string_t *node_enclave(ZenohPicoNodeInfo *node)		{ return z_loan(node->enclave); }

z_owned_mutex_t mutex_ZenohPicoNodeInfo;

ZenohPicoNodeInfo * zenoh_pico_generate_node_info(size_t domain_id,
						  const char *ns,
						  const char *name,
						  const z_loaned_string_t *enclave)
{
  ZenohPicoNodeInfo *node = NULL;
  ZenohPicoGenerateData(node, ZenohPicoNodeInfo);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node,
    "failed to allocate struct for the ZenohPicoNodeInfo",
    return NULL);

  z_string_empty(&node->domain);
  z_string_empty(&node->ns);
  z_string_empty(&node->name);
  z_string_empty(&node->enclave);

  conv_domain(domain_id, &node->domain);
  z_string_copy_from_str(&node->ns, ns);
  z_string_copy_from_str(&node->name, name);
  z_string_clone(&node->enclave, enclave);

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

  z_string_clone(&node->domain, z_loan(src->domain));
  z_string_clone(&node->ns, z_loan(src->ns));
  z_string_clone(&node->name, z_loan(src->name));
  z_string_clone(&node->enclave, z_loan(src->enclave));

  return node;
}

bool zenoh_pico_destroy_node_info(ZenohPicoNodeInfo *node)
{
  RMW_ZENOH_FUNC_ENTRY(node);

  RMW_CHECK_ARGUMENT_FOR_NULL(node, false);

  z_drop(z_move(node->domain));
  z_drop(z_move(node->ns));
  z_drop(z_move(node->name));
  z_drop(z_move(node->enclave));

  ZenohPicoDestroyData(node, ZenohPicoNodeInfo);

  return true;
}

void zenoh_pico_debug_node_info(ZenohPicoNodeInfo *node)
{
  printf("node info ...\n");

  const z_loaned_string_t *domain	= z_loan(node->domain);
  const z_loaned_string_t *ns		= z_loan(node->ns);
  const z_loaned_string_t *name		= z_loan(node->name);
  const z_loaned_string_t *enclave	= z_loan(node->enclave);

  printf("\tdomain        = %.*s\n", (int)z_string_len(domain),  z_string_data(domain));
  printf("\tnamespace     = %.*s\n", (int)z_string_len(ns),      z_string_data(ns));
  printf("\tname          = %.*s\n", (int)z_string_len(name),    z_string_data(name));
  printf("\tenclave_      = %.*s\n", (int)z_string_len(enclave), z_string_data(enclave));
}

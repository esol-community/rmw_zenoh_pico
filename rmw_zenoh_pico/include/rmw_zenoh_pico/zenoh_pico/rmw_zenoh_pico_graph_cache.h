/*
 * Copyright(C) 2026 the ros2-emscripten-zenoh-demo project.
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

#ifndef RMW_ZENOH_PICO_GRAPH_CACHE_H
#define RMW_ZENOH_PICO_GRAPH_CACHE_H

#include <rmw/rmw.h>
#include <zenoh-pico.h>

#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_entity.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_session.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_event.h>

#if defined(__cplusplus)
extern "C"
{
#endif  // if defined(__cplusplus)

  // One local publisher, subscription, or client, registered with its
  // session's graph cache so the session-wide liveliness discovery
  // subscriber can find it when a same-topic remote peer is discovered (or
  // lost). For a Publisher/Subscription this updates its MATCHED /
  // QOS_INCOMPATIBLE event counters; for a Client (matched against a
  // remote Service) it updates the owning ZenohPicoServiceData's
  // available_services count instead (backing
  // rmw_service_server_is_available()) -- qos/event_mgr are unused in that
  // case, just kept non-NULL for a uniform registration call. Owns none of
  // the pointed-to data -- topic_name/topic_type/qos/event_mgr all live
  // inside the ZenohPicoPubData/ZenohPicoSubData/ZenohPicoServiceData this
  // entry was registered for, and must outlive it (guaranteed:
  // unregistered in the same rmw_destroy_publisher()/
  // rmw_destroy_subscription()/rmw_destroy_client() call that frees it).
  typedef struct _ZenohPicoGraphLocalEntity
  {
    struct _ZenohPicoGraphLocalEntity *next;

    ZenohPicoEntityType type;  // Publisher, Subscription, or Client
    const z_loaned_string_t *topic_name;
    const z_loaned_string_t *topic_type;
    rmw_qos_profile_t *qos;
    DataEventManager *event_mgr;

    // Opaque identity used only to find this entry again on unregister --
    // always the owning ZenohPicoPubData*/ZenohPicoSubData*/
    // ZenohPicoServiceData*, never dereferenced here.
    void *owner;
  } ZenohPicoGraphLocalEntity;

  // Registers a local publisher, subscription, or client so remote
  // liveliness discovery can match against it. Safe to call multiple times
  // per session (once per local entity) -- the session-wide liveliness
  // subscriber itself is only actually declared once, on the first
  // registration.
  extern bool graph_cache_register_local(
    ZenohPicoSession *session,
    ZenohPicoEntityType type,
    const z_loaned_string_t *topic_name,
    const z_loaned_string_t *topic_type,
    rmw_qos_profile_t *qos,
    DataEventManager *event_mgr,
    void *owner);

  // Unregisters a previously-registered local entity (matched by the same
  // `owner` pointer passed to graph_cache_register_local()). No-op if not
  // found (e.g. registration itself failed).
  extern void graph_cache_unregister_local(ZenohPicoSession *session, void *owner);

  // Tears down the session-wide liveliness discovery subscriber and frees
  // every remaining registered local entity. Called from
  // zenoh_pico_destroy_session().
  extern void graph_cache_session_destroy(ZenohPicoSession *session);

#if defined(__cplusplus)
}
#endif  // if defined(__cplusplus)

#endif

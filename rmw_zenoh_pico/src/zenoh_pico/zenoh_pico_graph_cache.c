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
 *
 * QoS event/matching support. rmw_zenoh_pico already declares a real
 * liveliness token for every publisher/subscription it creates, with the
 * full topic name/type/QoS encoded into the key (see
 * zenoh_pico_liveliness.c's generate_liveliness()/qos_to_keyexpr()) -- but
 * nothing ever subscribed to *other* peers' liveliness tokens to discover
 * them, which is the "gid_cache is not implemented" this file's own header
 * comment used to describe. This adds that other half: one session-wide
 * liveliness subscriber under the same "@ros2_lv" admin space every peer
 * already announces into, decoding each discovered publisher/subscription's
 * topic and QoS and matching it against this session's own local
 * publishers/subscriptions on the same topic.
 */

#include <stdlib.h>
#include <string.h>

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_graph_cache.h>

// A plain macro (not the z_view_keyexpr_from_substr()-facing char array
// below) so it can still be concatenated with an adjacent string literal
// at compile time (e.g. ADMIN_SPACE_PREFIX_LIT "/**") -- a `const char[]`
// identifier can't be: string-literal concatenation is a preprocessor/
// lexer-level operation on adjacent literal tokens, not something that
// works on a variable's runtime value.
#define ADMIN_SPACE_PREFIX_LIT "@ros2_lv"
static const char ADMIN_SPACE_PREFIX[] = ADMIN_SPACE_PREFIX_LIT;
// Segment index (0-based, after splitting the whole key on '/') of the
// two-letter entity-type code -- see conv_entity_type() in
// zenoh_pico_liveliness.c for the exact values ("MP" publisher, "MS"
// subscription, "NN" node, "SS" service, "SC" client) and
// generate_liveliness() for why it's always this position.
#define ENTITY_TYPE_SEGMENT_INDEX 5
#define TOPIC_NAME_SEGMENT_INDEX  9
#define TOPIC_TYPE_SEGMENT_INDEX  10
#define TOPIC_QOS_SEGMENT_INDEX   12
#define MIN_TOPIC_ENTITY_SEGMENTS 13

// ---------------------------------------------------------------- parsing

// Splits `key` (a NUL-terminated, mutable copy of a liveliness sample's key
// expression) in place on '/', filling `out[]` with a pointer to the start
// of each segment (each also left NUL-terminated, since the '/' bytes
// themselves get overwritten). Returns the segment count found, capped at
// `max_segments`.
static size_t split_segments(char *key, char **out, size_t max_segments)
{
  size_t count = 0;
  char *p = key;

  out[count++] = p;
  while (*p != '\0' && count < max_segments) {
    if (*p == '/') {
      *p = '\0';
      out[count++] = p + 1;
    }
    p++;
  }

  return count;
}

// SLASH_REPLACEMENT ('%') -> '/', in place -- mirrors
// zenoh_pico_liveliness.c's own (file-local, not exported) demangle_name(),
// which generate_liveliness() uses to escape a real '/' inside a name
// (e.g. a namespaced topic) before splicing it into the liveliness key as
// one segment. Duplicated here rather than exported from that file for one
// caller.
static void demangle_segment(char *s)
{
  for (; *s != '\0'; s++) {
    if (*s == '%') {
      *s = '/';
    }
  }
}

// Reverse of qos_to_keyexpr() (zenoh_pico_liveliness.c): that function
// always emits these 11 QoS fields in this fixed order, joined by a plain
// field separator (either ':' or ',' -- deliberately treated the same way
// here, since the *order* of values is what carries meaning, not which
// separator character happened to be used at each position) with a
// zero-valued field written as an empty token instead of "0".
static void qos_from_segment(const char *qos_segment, rmw_qos_profile_t *out)
{
  memset(out, 0, sizeof(*out));

  char buf[64];
  memset(buf, 0, sizeof(buf));
  strncpy(buf, qos_segment, sizeof(buf) - 1);

  long values[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  size_t idx = 0;
  char *tok_start = buf;
  char *p = buf;

  while (idx < 11) {
    if (*p == ':' || *p == ',' || *p == '\0') {
      char saved = *p;
      *p = '\0';
      values[idx++] = (tok_start[0] != '\0') ? strtol(tok_start, NULL, 10) : 0;
      if (saved == '\0') {
	break;
      }
      tok_start = p + 1;
    }
    p++;
  }

  out->reliability                    = (rmw_qos_reliability_policy_t)values[0];
  out->durability                     = (rmw_qos_durability_policy_t)values[1];
  out->history                        = (rmw_qos_history_policy_t)values[2];
  out->depth                          = (size_t)values[3];
  out->deadline.sec                   = (uint64_t)values[4];
  out->deadline.nsec                  = (uint64_t)values[5];
  out->lifespan.sec                   = (uint64_t)values[6];
  out->lifespan.nsec                  = (uint64_t)values[7];
  out->liveliness                     = (rmw_qos_liveliness_policy_t)values[8];
  out->liveliness_lease_duration.sec  = (uint64_t)values[9];
  out->liveliness_lease_duration.nsec = (uint64_t)values[10];
}

// ------------------------------------------------------------ compatibility

// Standard ROS 2 / DDS offered-vs-requested QoS compatibility rules for the
// two policies real applications actually rely on for QoS matching in
// practice. SYSTEM_DEFAULT on either side is treated as "no constraint"
// here -- a documented simplification; a fully rigorous check would first
// resolve SYSTEM_DEFAULT to whatever the middleware's actual default is.
// deadline/lifespan/liveliness compatibility is intentionally not checked:
// narrower in scope than a complete DDS-compatible implementation, but
// covers the two policies (reliability, durability) that dominate
// real-world QoS mismatches.
static bool qos_reliability_compatible(rmw_qos_reliability_policy_t offered,
					rmw_qos_reliability_policy_t requested)
{
  return !(requested == RMW_QOS_POLICY_RELIABILITY_RELIABLE &&
	   offered == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
}

static bool qos_durability_compatible(rmw_qos_durability_policy_t offered,
				       rmw_qos_durability_policy_t requested)
{
  return !(requested == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL &&
	   offered == RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

// pub_qos is always the offered (publisher-side) profile and sub_qos is
// always the requested (subscription-side) profile, regardless of which
// side is the local entity being matched.
static bool qos_compatible(rmw_qos_profile_t *pub_qos, rmw_qos_profile_t *sub_qos)
{
  return qos_reliability_compatible(pub_qos->reliability, sub_qos->reliability) &&
	 qos_durability_compatible(pub_qos->durability, sub_qos->durability);
}

// ------------------------------------------------------------- matching

static void update_match_events(ZenohPicoGraphLocalEntity *local,
				  rmw_qos_profile_t *remote_qos,
				  bool is_put)
{
  rmw_qos_profile_t *pub_qos = (local->type == Publisher) ? local->qos : remote_qos;
  rmw_qos_profile_t *sub_qos = (local->type == Publisher) ? remote_qos : local->qos;
  bool reliability_ok = qos_reliability_compatible(pub_qos->reliability, sub_qos->reliability);
  bool durability_ok = qos_durability_compatible(pub_qos->durability, sub_qos->durability);
  bool compatible = reliability_ok && durability_ok;

  rmw_event_type_t matched_event = (local->type == Publisher)
    ? RMW_EVENT_PUBLICATION_MATCHED : RMW_EVENT_SUBSCRIPTION_MATCHED;
  rmw_event_type_t incompatible_event = (local->type == Publisher)
    ? RMW_EVENT_OFFERED_QOS_INCOMPATIBLE : RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE;

  if (compatible) {
    if (is_put) {
      // A newly-discovered compatible peer is both a new lifetime match
      // (total_count) and one more currently-live match (current_count).
      add_rmw_zenoh_pico_event_total(local->event_mgr, matched_event, true);
      add_rmw_zenoh_pico_event_current(local->event_mgr, matched_event, true);
    } else {
      // A previously-compatible peer going away is one fewer live match --
      // total_count (the lifetime count) never decreases, matching
      // rmw_matched_status_t's own documented semantics.
      sub_rmw_zenoh_pico_event_current(local->event_mgr, matched_event, true);
    }
  } else if (is_put) {
    // rmw_qos_incompatible_event_status_t has no current_count at all --
    // it's a pure lifetime incident counter (see rmw/events_statuses/
    // incompatible_qos.h), so nothing to do on a DELETE here either way.
    // last_policy_kind must be set before the event's total_count/changed
    // flag goes live (add_rmw_zenoh_pico_event_total() below) -- a reader
    // that wakes up on `changed` should never see a stale/unset value.
    // Reliability is checked first above, so if it's the one that failed,
    // report it; otherwise durability must be the culprit (compatible is
    // only false here because at least one of the two failed).
    set_rmw_zenoh_pico_event_last_policy_kind(
      local->event_mgr, incompatible_event,
      !reliability_ok ? RMW_QOS_POLICY_RELIABILITY : RMW_QOS_POLICY_DURABILITY);
    add_rmw_zenoh_pico_event_total(local->event_mgr, incompatible_event, true);
  }
}

// rmw_service_server_is_available() only cares about existence, not QoS
// compatibility -- unlike update_match_events() (Publisher<->Subscription),
// there's no MATCHED/INCOMPATIBLE event pair here, just a live count of
// currently-discovered matching services. Clamped at 0: a DELETE for a
// service this client never actually saw a matching PUT for (e.g. it
// existed before this client's own liveliness subscriber was declared, and
// this session never got the "history" PUT replay for some reason) must
// never take the count negative.
static void update_client_availability(ZenohPicoServiceData *client_data, bool is_put)
{
  client_data->available_services += is_put ? 1 : -1;
  if (client_data->available_services < 0) {
    client_data->available_services = 0;
  }
}

static void graph_cache_process_sample(ZenohPicoSession *session,
					const char *key_str, size_t key_len,
					bool is_put)
{
  char buf[RMW_ZENOH_PICO_MAX_LINENESS_LEN];
  if (key_len >= sizeof(buf)) {
    key_len = sizeof(buf) - 1;
  }
  memcpy(buf, key_str, key_len);
  buf[key_len] = '\0';

  char *segments[MIN_TOPIC_ENTITY_SEGMENTS];
  size_t segment_count = split_segments(buf, segments, MIN_TOPIC_ENTITY_SEGMENTS);
  if (segment_count < MIN_TOPIC_ENTITY_SEGMENTS) {
    // Not a topic-bearing entity -- a plain node's liveliness key has no
    // topic_info segments at all (generate_liveliness() only appends them
    // `if (entity->topic_info != NULL)`, which a plain Node never has).
    // Service and Client entities DO carry topic_info (the service name/
    // type, same as a topic) and so reach the entity_type dispatch below
    // just like Publisher/Subscription -- confirmed via
    // zenoh_pico_generate_service_entity()'s own call to
    // zenoh_pico_generate_topic_info().
    return;
  }
  if (strcmp(segments[0], ADMIN_SPACE_PREFIX) != 0) {
    return;
  }

  const char *entity_type = segments[ENTITY_TYPE_SEGMENT_INDEX];
  ZenohPicoEntityType remote_type;
  if (strcmp(entity_type, "MP") == 0) {
    remote_type = Publisher;
  } else if (strcmp(entity_type, "MS") == 0) {
    remote_type = Subscription;
  } else if (strcmp(entity_type, "SS") == 0) {
    remote_type = Service;
  } else {
    // A client/node liveliness token -- also not matched here. Nothing
    // currently needs a service or client to discover remote *clients*
    // (only rmw_service_server_is_available(), client-side only, needs
    // this at all), so "SC" is deliberately left unhandled, same as "NN".
    return;
  }

  demangle_segment(segments[TOPIC_NAME_SEGMENT_INDEX]);
  demangle_segment(segments[TOPIC_TYPE_SEGMENT_INDEX]);

  rmw_qos_profile_t remote_qos;
  qos_from_segment(segments[TOPIC_QOS_SEGMENT_INDEX], &remote_qos);

  z_mutex_lock(z_loan_mut(session->graph_lock));

  for (ZenohPicoGraphLocalEntity *local = session->graph_local_entities;
       local != NULL; local = local->next) {
    // Publisher<->Subscription and Client<->Service are the only pairs
    // that can ever match -- a local publisher only cares about remote
    // subscriptions (and vice versa), and a local client only cares about
    // remote services (nothing currently needs the reverse).
    bool pubsub_pair =
      (local->type == Publisher && remote_type == Subscription) ||
      (local->type == Subscription && remote_type == Publisher);
    bool client_service_pair =
      local->type == Client && remote_type == Service;
    if (!pubsub_pair && !client_service_pair) {
      continue;
    }

    if (z_string_len(local->topic_name) != strlen(segments[TOPIC_NAME_SEGMENT_INDEX]) ||
	strncmp(z_string_data(local->topic_name), segments[TOPIC_NAME_SEGMENT_INDEX],
		z_string_len(local->topic_name)) != 0) {
      continue;
    }
    if (z_string_len(local->topic_type) != strlen(segments[TOPIC_TYPE_SEGMENT_INDEX]) ||
	strncmp(z_string_data(local->topic_type), segments[TOPIC_TYPE_SEGMENT_INDEX],
		z_string_len(local->topic_type)) != 0) {
      continue;
    }

    if (client_service_pair) {
      update_client_availability((ZenohPicoServiceData *)local->owner, is_put);
    } else {
      update_match_events(local, &remote_qos, is_put);
    }
  }

  z_mutex_unlock(z_loan_mut(session->graph_lock));
}

static void graph_liveliness_sample_handler(z_loaned_sample_t *sample, void *ctx)
{
  ZenohPicoSession *session = (ZenohPicoSession *)ctx;
  if (session == NULL) {
    return;
  }

  z_view_string_t keystr;
  z_keyexpr_as_view_string(z_sample_keyexpr(sample), &keystr);
  const z_loaned_string_t *loaned = z_loan(keystr);

  bool is_put = (z_sample_kind(sample) == Z_SAMPLE_KIND_PUT);
  graph_cache_process_sample(session, z_string_data(loaned), z_string_len(loaned), is_put);
}

// ----------------------------------------------------------- registration

bool graph_cache_register_local(
  ZenohPicoSession *session,
  ZenohPicoEntityType type,
  const z_loaned_string_t *topic_name,
  const z_loaned_string_t *topic_type,
  rmw_qos_profile_t *qos,
  DataEventManager *event_mgr,
  void *owner)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(session, false);
  RMW_CHECK_ARGUMENT_FOR_NULL(owner, false);

  ZenohPicoGraphLocalEntity *entry = (ZenohPicoGraphLocalEntity *)
    malloc(sizeof(ZenohPicoGraphLocalEntity));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    entry, "failed to allocate ZenohPicoGraphLocalEntity", return false);

  entry->type = type;
  entry->topic_name = topic_name;
  entry->topic_type = topic_type;
  entry->qos = qos;
  entry->event_mgr = event_mgr;
  entry->owner = owner;

  if (!session->graph_liveliness_sub_active) {
    // Lazily declared on the first local publisher/subscription, once the
    // session is definitely already open (both rmw_create_publisher() and
    // rmw_create_subscription() only reach here after their own liveliness
    // token declared successfully, which itself requires an open session).
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_substr(&ke, ADMIN_SPACE_PREFIX_LIT "/**", strlen(ADMIN_SPACE_PREFIX_LIT "/**"));

    z_liveliness_subscriber_options_t options;
    z_liveliness_subscriber_options_default(&options);
    options.history = true;  // see already-declared peers, not just future changes

    z_owned_closure_sample_t callback;
    z_closure(&callback, graph_liveliness_sample_handler, 0, (void *)session);

    if (_Z_IS_ERR(z_liveliness_declare_subscriber(
	  z_loan(session->session), &session->graph_liveliness_sub,
	  z_loan(ke), z_move(callback), &options))) {
      RMW_ZENOH_LOG_INFO("Unable to declare graph liveliness subscriber -- "
			  "QoS matching/events will not see remote peers.");
      // Not fatal: the local entity itself still gets created and works
      // for pub/sub as normal, it just never sees any matches.
    } else {
      session->graph_liveliness_sub_active = true;
    }
  }

  z_mutex_lock(z_loan_mut(session->graph_lock));
  entry->next = session->graph_local_entities;
  session->graph_local_entities = entry;
  z_mutex_unlock(z_loan_mut(session->graph_lock));

  return true;
}

void graph_cache_unregister_local(ZenohPicoSession *session, void *owner)
{
  if (session == NULL || owner == NULL) {
    return;
  }

  z_mutex_lock(z_loan_mut(session->graph_lock));

  ZenohPicoGraphLocalEntity **prev = &session->graph_local_entities;
  ZenohPicoGraphLocalEntity *cur = session->graph_local_entities;
  while (cur != NULL) {
    if (cur->owner == owner) {
      *prev = cur->next;
      free(cur);
      break;
    }
    prev = &cur->next;
    cur = cur->next;
  }

  z_mutex_unlock(z_loan_mut(session->graph_lock));
}

void graph_cache_session_destroy(ZenohPicoSession *session)
{
  if (session == NULL) {
    return;
  }

  if (session->graph_liveliness_sub_active) {
    z_undeclare_subscriber(z_move(session->graph_liveliness_sub));
    session->graph_liveliness_sub_active = false;
  }

  ZenohPicoGraphLocalEntity *cur = session->graph_local_entities;
  while (cur != NULL) {
    ZenohPicoGraphLocalEntity *next = cur->next;
    free(cur);
    cur = next;
  }
  session->graph_local_entities = NULL;
}

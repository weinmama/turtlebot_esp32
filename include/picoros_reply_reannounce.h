#pragma once

#include "picoros.h"
#include "zenoh-pico/net/primitives.h"

// Pinned Zenoh-Pico adapter: refresh wire interest without destroying the local
// callback, its key resource or ROS liveliness token. Full drop/redeclare also
// churns discovery and can disrupt a busy split serial-router session.
// Only the transport service task may call this; never hold the motor mutex.
inline bool picoros_reply_reannounce(picoros_subscriber_t& subscriber) {
  const auto* sub = z_subscriber_loan(&subscriber.zsub);
  auto session = _z_session_weak_upgrade_if_open(&sub->_zn);
  if (_Z_RC_IS_NULL(&session)) return false;
  auto* zn = _Z_RC_IN_VAL(&session);
  const auto* key = z_subscriber_keyexpr(sub);
  if (!key) { _z_session_rc_drop(&session); return false; }

  _z_network_message_t message;
  auto declaration = _z_make_undecl_subscriber(sub->_entity_id, nullptr);
  _z_n_msg_make_declare(&message, declaration, _z_optional_id_make_none());
  const auto removed = _z_send_undeclare(zn, &message);
  _z_n_msg_clear(&message);

  // Send the full expression, not a resource ID which might itself be missing
  // on the other end. The existing local callback remains alive throughout.
  auto wire = _z_keyexpr_alias_to_wire(&key->_inner);
  declaration = _z_make_decl_subscriber(&wire, sub->_entity_id);
  _z_n_msg_make_declare(&message, declaration, _z_optional_id_make_none());
  const auto declared = _z_send_declare(zn, &message);
  _z_n_msg_clear(&message);
  _z_session_rc_drop(&session);
  return removed == _Z_RES_OK && declared == _Z_RES_OK;
}

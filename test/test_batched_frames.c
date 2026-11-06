// Regression: adjacent best-effort/reliable Frames in one Zenoh batch must be
// returned to the transport decoder, not decoded as a network message (MID 5).
#include <assert.h>
#include "zenoh-pico/protocol/codec/transport.h"
#include "zenoh-pico/transport/unicast/rx.h"
#include "zenoh-pico/transport/unicast/transport.h"

int main(void) {
    uint8_t bytes[] = {_Z_MID_T_FRAME, 1, _Z_MID_T_FRAME | _Z_FLAG_T_FRAME_R, 1};
    _z_slice_t slice = _z_slice_alias_buf(bytes, sizeof(bytes));
    _z_zbuf_t buffer = _z_slice_as_zbuf(slice);
    _z_transport_unicast_t transport = {0};
    _z_transport_peer_unicast_t peer = {0};
    transport._common._sn_res = 0xffffffff;
    _z_transport_message_t message;
    assert(_z_transport_message_decode(&message, &buffer) == _Z_RES_OK);
    assert(_z_unicast_handle_transport_message(&transport, &message, &peer) == _Z_RES_OK);
    assert(_z_zbuf_len(&buffer) == 2);
    assert(_z_transport_message_decode(&message, &buffer) == _Z_RES_OK);
    assert(_z_unicast_handle_transport_message(&transport, &message, &peer) == _Z_RES_OK);
    assert(_z_zbuf_len(&buffer) == 0);
    assert(peer._sn_rx_best_effort == 1 && peer._sn_rx_reliable == 1);
    return 0;
}

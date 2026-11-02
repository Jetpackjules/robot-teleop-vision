# Native point preview over unordered WebRTC

Opt in with `?native_point_webrtc=1` on controller.html and select 20k or 40k
points per camera. Default behavior is unchanged pending cross-country relay
validation. Signaling uses the existing authenticated /webrtc-offer endpoint.
The native_points source requires an integer pointBudget of 20000 or 40000.

The channel is unordered with maxRetransmits=0. Native camera-buffer packets
retain the existing exact lossless point-delta representation. Each packet is
split into messages no larger than 16384 bytes, with an NPC1 header containing
packet sequence, total size, part index/count and camera slot. Reassembly is
bounded to 8 partial packets of 512 KiB, expires partial packets after 250 ms, and
discards obsolete completed frames per camera. A missing chunk never blocks a
later camera capture. Delivered frames preserve all selected point bytes;
packet loss can drop an incomplete preview frame. Recording is independent.

The sender keeps the latest pending packet per camera and never adds another
capture behind a buffered data-channel packet. Full-detail legacy WebRTC is
unchanged. The renderer automatically falls back to WebSocket if connectivity
fails or complete camera packets stop arriving. Changing Full/sparse mode in
the opt-in UI renegotiates the appropriate peer; 20k/40k changes stay dynamic.

Validation: 10 browser transport/reassembly assertions, including actual local
unordered RTC transfer of 360000 exact bytes; 41 existing rendering checks;
2 native RTC tests including live camera-buffer packets for all 3 cameras at
both budgets across a real local aiortc peer pair; 7 existing native stream tests;
6 legacy stream/socket tests. One Node-dependent host test was skipped.

The actual laptop-to-lab UI attempted a native point channel but ICE failed;
WebSocket fallback displayed all 3 cameras without page errors. This network
still needs a working direct route or relay. Existing ICE configuration is
STUN-only; both browser and gateway need TURN configuration for a relay.
No cross-country WebRTC FPS or freeze improvement is claimed. Nine saved
calibration/configuration hashes stayed unchanged; capture/follower were not
restarted. Evidence: .teleop/native-point-rtc-20261027/.


## Relay configuration

The gateway reads a private JSON file named by ROBOT_TELEOP_ICE_CONFIG.
A Cloudflare TURN key can be configured as:

```json
{"cloudflareTurn":{"keyId":"TURN_KEY_ID","apiToken":"PRIVATE_TURN_KEY_TOKEN","ttl":86400}}
```

The API token stays on the host. The gateway mints expiring ICE credentials,
caches them, and returns only those credentials to authenticated browsers via
/api/v1/ice-servers (no-store). Credential generation runs outside the realtime
async event loop. A coturn REST shared secret or static ICE entries can also
be configured using iceServers; sharedSecret is never returned to browsers.
Use rtc_relay=1 with native_point_webrtc=1 for a forced-relay diagnostic test.
Normal behavior remains opt-in until the actual cross-country relay is proven.

On the current host the private configuration lives at
.teleop/turn-relay-20261027/ice.json, initially containing only the existing STUN
entry. No TURN credentials have been provisioned yet. The documented public
static test relay did not allocate over UDP80/443 or TCP/TLS443, so it was not
enabled in production. Capture was not restarted. Eight protected files stayed byte-identical; a
concurrent gripper-calibration save changed registration jaw fields only. Both
versions were backed up, and the newer live registration was left intact.

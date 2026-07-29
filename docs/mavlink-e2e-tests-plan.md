# MAVLink byte-stream e2e tests (pymavlink-referenced)

**Status:** Tier 1 = **implemented** (libmavconn TX/RX + signing e2e, all green in container).
Tier 2 = postponed, scope confirmed.

**Goal:** Verify libmavconn serializes/deserializes MAVLink byte streams correctly, using
pymavlink as the reference implementation. Later, verify the MAVLink router end-to-end in
all modes.

**Design constraints (from maintainer):**

- `test_depend` on a pip package (`python3-pymavlink-pip`) is acceptable.
- Minimize owned surface area: no shared Python framework, no CI/pip automation, tests must
  skip cleanly when pymavlink is absent so there is nothing to babysit.
- Deterministic, low-flake tests preferred.

**Environment:** Container `ros2-lyrical` (podman), workspace at `/ws`, repo mounted at
`/ws/src/mavros`. pymavlink installed via `pip install --break-system-packages pymavlink`
(one-time, manual — not automated anywhere).

---

## Tier 1 — libmavconn byte-stream e2e (IMPLEMENT NOW)

### Files

1. **`libmavconn/test/e2e/mavconn_e2e_shim.cpp`** (new; built only under `BUILD_TESTING`)
   - Thin C-ABI wrapper over `MAVConnUDP` for `ctypes`. Glue only, no abstractions.
   - API:
     - `mavconn_udp_create(sysid, compid, bind_port, remote_host, remote_port) -> void*`
       (receive callback pushes decoded `(msgid, sysid, compid, seq, len, framing, payload[])`
       to a thread-safe queue)
     - `mavconn_poll_rx(handle, out*, timeout_ms) -> int`
     - `mavconn_send_heartbeat(handle, ...) -> int`
       (serialize `common::msg::HEARTBEAT`, `send_message()`)
     - `mavconn_send_raw_bytes(handle, bytes, len) -> int` (`send_bytes()`)
     - `mavconn_setup_signing(handle, key[32], sign_outgoing, link_id, init_ts)`
     - `mavconn_destroy(handle)`

2. **`libmavconn/test/e2e/test_libmavconn_e2e.py`** (new, pytest)
   - Loads shim via `ctypes` (path from `MAVCONN_E2E_SHIM` env var set by CMake).
   - pymavlink packing/parsing inlined directly — no shared helper module.
   - Every test guarded by `pytest.importorskip("pymavlink")` → skips in CI/buildfarm
     without pip (verified: whole module skips when pymavlink is absent).
   - Tests (all passing in `ros2-lyrical`):
     - `test_tx_heartbeat_matches_pymavlink` — libmavconn HEARTBEAT byte-for-byte equal to
       pymavlink reference frame.
     - `test_rx_heartbeat_from_pymavlink` — pymavlink HEARTBEAT → libmavconn decodes
       msgid/ids/seq/payload exactly.
     - `test_rx_sys_status_from_pymavlink` — extension-field message payload round-trips.
     - `test_tx_signed_heartbeat_verified_by_pymavlink` — signed TX verified by pymavlink
       (SIGNED incompat flag set, `badsig_count == 0`).
     - `test_rx_signed_heartbeat_from_pymavlink` — signed RX accepted with `Framing::ok`.
     - `test_rx_bad_signature_rejected` — wrong key → `Framing::bad_signature`.

   **Gotcha captured:** pymavlink message constructors take fields in *declaration* order,
   not wire order — always construct reference frames with keyword args (a positional
   mix-up here initially produced a false wire mismatch).

3. **`libmavconn/CMakeLists.txt`** (edit)
   - In `if(BUILD_TESTING)`: `find_package(ament_cmake_pytest REQUIRED)`; build shim as
     `SHARED` lib linked to `mavconn`; `ament_add_pytest_test(libmavconn_pymavlink_e2e …)`
     passing shim path via `ENV MAVCONN_E2E_SHIM` and shim dir via
     `APPEND_ENV LD_LIBRARY_PATH`. Also points flake8 at the new `setup.cfg`.

4. **`libmavconn/package.xml`** (edit)
   - Add `<test_depend>ament_cmake_pytest</test_depend>` and
     `<test_depend>python3-pymavlink-pip</test_depend>`.

5. **`libmavconn/setup.cfg`** (new)
   - flake8 config mirroring `mavros/setup.cfg` (double quotes allowed, google import
     order), so the new Python test matches the project's actual style. libmavconn had no
     flake8 config before, so it previously used ament's stricter default.

### Build / run (container)

    podman exec ros2-lyrical bash -lc 'source /opt/ros/lyrical/setup.bash && \
      colcon build --base-paths /ws/src/mavros --packages-select libmavconn'
    podman exec ros2-lyrical bash -lc 'source /ws/install/setup.bash && \
      cd /ws/build/libmavconn && ctest --output-on-failure -R pymavlink'

**Est. tokens:** ~70–120k (deterministic, in-process, no ROS runtime → low flake).

---

## Tier 2 — router wire e2e (POSTPONED, scope confirmed)

**Confirmed requirement:** router must be verified functioning correctly in **all modes**
(FCU↔GCS, FCU↔UAS, GCS↔UAS routing; broadcast + targeted). Postponed until after Tier 1
lands.

**Endpoint mapping (verified from code):**

- `fcu_urls` / `gcs_urls` → `MAVConnEndpoint` (real UDP/TCP wire links).
- `uas_urls` → `ROSEndpoint` (ROS topics `<url>/mavlink_source` FCU→UAS,
  `<url>/mavlink_sink` UAS→FCU; QoS best_effort + volatile).

**Planned topology:** pymavlink peer (FCU over UDP) → `MAVConnEndpoint` → Router →
`ROSEndpoint` ↔ test's rclpy node on `/uas1/mavlink_{source,sink}`. Optionally add a
`gcs_urls` UDP link for wire→wire broadcast coverage.

**Test cases (all asserted against pymavlink reference):**

- FCU→UAS (wire→ROS topic): pymavlink packs HEARTBEAT + targeted msg into FCU link; rclpy
  asserts `mavros_msgs::msg::Mavlink` fields (msgid/sysid/compid/payload64/len/framing_status)
  match.
- UAS→FCU (ROS topic→wire): rclpy publishes `Mavlink` (from pymavlink-packed msg) to sink;
  pymavlink peer asserts emitted wire bytes parse and match (msgid, payload, ids, CRC).
- Targeted routing: `target_system`/`target_component` reaches only matching endpoint.
- All-modes matrix: verify each FCU/GCS/UAS pairing per routing rules in
  `Router::route_message`.
- Signing through router: **blocked** — `MAVConnEndpoint::open()` has
  `// TODO(vooon): message signing?` and does not wire signing. Deferred until router
  signing is implemented.

**Files (planned):**

- `mavros/test/router_e2e/test_router_wire_e2e.py` (rclpy + pymavlink; `importorskip`
  guard; inline pymavlink helpers, no shared module).
- `mavros/CMakeLists.txt` — `ament_add_pytest_test(mavros_router_wire_e2e …)`.
- `mavros/package.xml` — `<test_depend>python3-pymavlink-pip</test_depend>`.

**Risk note:** Tier 2 introduces rclpy + DDS + running component + timing → highest
flake/maintenance potential. Time-box when implemented; if router launch/timing flakes in
the container, cut scope rather than nurse it. Existing `test_router.cpp` already
unit-tests the routing table with mocks — Tier 2 targets the **wire path +
`mavros_msgs::mavlink::convert` round-trip**, which is the real gap.

**Est. tokens:** ~70–130k (2–4 debug iterations likely).

---

## Model / execution notes

- Implementation: mid-tier agentic coding model is sufficient; escalate only router-launch
  debugging to a flagship if it drags >2 iterations.
- Recommendation (cost control): land Tier 1 as its own PR; Tier 2 as a follow-up PR
  rebased on it.

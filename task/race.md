# xbot_monitoring — MQTT `connected()` reentrance deadlock

Upstream repo: [`open_mower_ros`](https://github.com/ClemensElflein/open_mower_ros) — source at [`../open_mower_ros/src/lib/xbot_monitoring/src/xbot_monitoring.cpp`](../open_mower_ros/src/lib/xbot_monitoring/src/xbot_monitoring.cpp).

Affected images (confirmed reproducing):
- `ghcr.io/clemenselflein/open_mower_ros:1.1.1`
- `ghcr.io/clemenselflein/open_mower_ros:1.1.1-legacy`
- `ghcr.io/alexvaut/open_mower_ros:sha-07e190a` (alexvaut fork off the same source)

---

## 1. Symptom

On startup of `openmower.service`, **`mower_logic` hangs at `"registering actions"`** and never reaches `"Got all servers, we can mow"`. As a downstream effect, `/mower_logic/current_state` is never published, the GUI dashboard shows "Unknown", and MQTT `openmower/robot_state/json` is empty so any external consumer (e.g. an MCP server) has no data.

`map_service` exhibits the same class of hang on a different code path: it never advertises its `/mower_map_service/*` services (`get_mowing_area`, `add_mowing_area`, …), stuck in `ros::ServiceClient::call()` to `/xbot/rpc/register`.

The failure is **race-dependent**. Some restarts succeed; others hang. Conditions that make the race more likely (empirically, on an OpenMowerOS v1 / v1 hardware setup):
- External MQTT broker instead of localhost (network latency widens the race window).
- Simpler/smaller map (`map_service`'s `buildMap()` finishes sooner, so its RPC registration lands in the vulnerable window).

---

## 2. Root cause

`xbot_monitoring` uses `ros::AsyncSpinner(1)` — a **single** callback thread (line [722](../open_mower_ros/src/lib/xbot_monitoring/src/xbot_monitoring.cpp#L722)). That one thread processes *all* ROS service callbacks and subscriber callbacks for this node.

Two code paths both try to publish MQTT messages and end up contending for paho's internal mutex:

**Path A — paho `on_connected` callback** ([xbot_monitoring.cpp:77-98](../open_mower_ros/src/lib/xbot_monitoring/src/xbot_monitoring.cpp#L77-L98)):

```cpp
void connected(const mqtt::string &string) override {
    ROS_INFO_STREAM("MQTT Connected");
    publish_capabilities();
    publish_sensor_metadata();
    publish_map();
    publish_map_overlay();
    publish_actions();
    publish_version();
    publish_params();
    client_->subscribe(...);
    // ...
}
```

This runs on paho's `MQTTAsync_rcv` thread **while paho is still holding its internal callback mutex**. Each `publish_*()` helper calls `try_publish()` → `mqtt::async_client::publish()` → `MQTTAsync_sendMessage()` → `pthread_mutex_lock(same_mutex)`. Per [Eclipse Paho docs](https://www.eclipse.org/paho/files/mqttdoc/MQTTAsync/html/callbacks.html), **you must not call `publish()` from inside a paho callback**. In the common case this "works" because paho happens to release the mutex before `publish()` needs it, but under contention it deadlocks.

**Path B — ROS subscriber callback for `/xbot_monitoring/sensors/*/info`** ([xbot_monitoring.cpp:741-768](../open_mower_ros/src/lib/xbot_monitoring/src/xbot_monitoring.cpp#L741-L768)):

```cpp
active_subscribers[item.name] = n->subscribe<xbot_msgs::SensorInfo>(
    item.name, 1, [topic = item.name](const xbot_msgs::SensorInfo::ConstPtr &msg) {
        // ...
        publish_sensor_metadata();  // ← also calls client_->publish() via try_publish
    });
```

This runs on the `AsyncSpinner(1)` thread. Same `publish_sensor_metadata()`, same paho mutex.

When Path A and Path B execute concurrently (paho connects at roughly the same time a sensor topic becomes visible), they deadlock on paho's internal lock.

### 2.1 Cascade

Once the `AsyncSpinner(1)` thread is wedged in `publish_sensor_metadata()`, **no other callback on this node is ever processed**. In particular:

- `register_methods` ([line 668](../open_mower_ros/src/lib/xbot_monitoring/src/xbot_monitoring.cpp#L668)) — called by every node using `xbot_rpc::RpcProvider::init()` (includes `map_service`).
- `registerActions` ([line 585](../open_mower_ros/src/lib/xbot_monitoring/src/xbot_monitoring.cpp#L585)) — called by `mower_logic`.

Any `ros::ServiceClient::call()` from another node blocks forever waiting for a reply that never comes.

### 2.2 Evidence — GDB backtraces

Both threads caught live during a hang. Paho mutex is the one held on both sides.

Thread 8 (AsyncSpinner):
```
#0  pthread_mutex_lock
#2  (paho MQTTAsync internals)
#4  MQTTAsync_send
#5  MQTTAsync_sendMessage
#6  mqtt::async_client::publish(...)
#8  try_publish(...)
#9  publish_sensor_metadata()
#10 main::{lambda}::operator()  (sensor-info subscriber lambda, xbot_monitoring.cpp:742)
#12 ros::SubscriptionCallbackHelperT::call
#13 ros::SubscriptionQueue::call
#14 ros::CallbackQueue::callOneCB
```

Thread 7 (`MQTTAsync_rcv`):
```
#0  pthread_mutex_lock
#2  publish_sensor_metadata()
#3  MqttCallback::connected(...)
#4  mqtt::async_client::on_connected
#5  (paho internal)
```

Downstream: `map_service` main thread:
```
#0  pthread_cond_wait
#1  ros::ServiceServerLink::call
#2  ros::ServiceClient::call
#3  xbot_rpc::RpcProvider::publishMethods  (calls /xbot/rpc/register)
#4  xbot_rpc::RpcProvider::init
#5  main
```

### 2.3 Git history — when the footprint grew

The pattern "publish from `connected()`" is not new (first appearance `0dd1b2f` "map now is also published on reconnect", 2022-11-24). But each newer PR has added another publish inside `connected()`, widening the race window:

| Commit | Date | Change |
| --- | --- | --- |
| `0dd1b2f` | 2022-11-24 | first `publish_map()` on reconnect |
| `6182e97` | 2025-10-26 | JSON RPC 2.0 — added `rpc/request` subscribe inside `connected()` |
| `a57cbd0` | 2025-11-05 | added `publish_capabilities()` to `connected()` |
| `7596011` | **2026-03-15** | added `publish_params()` to `connected()` — shipped in `1.1.1` (built 2026-03-16) |

Six publishes + seven subscribes now run inside the paho callback mutex.

---

## 3. Reproduction

Any OpenMowerOS v1 / v1-hardware setup with:
- `ghcr.io/clemenselflein/open_mower_ros:1.1.1-legacy` (bundled mosquitto) or `:1.1.1` + external/remote MQTT broker.
- A map with low polygon density (e.g. our sample `map_backup_2026-04-12 (2).json` — 42932 B, 32 areas, max 64 points/polygon).

Run `sudo systemctl restart openmower.service`. Odds of hanging are high (~80–100% on the smallest maps, ~20% on maps with ~150 points/polygon). Confirmed deadlock by attaching GDB to `xbot_monitoring` (`sudo podman inspect openmower` to get host PID) — two threads blocked on the same paho mutex.

Detection without GDB: grep `podman logs openmower` for `"Got all servers, we can mow"`. If absent >30 s after `registering actions`, you're hung.

---

## 4. Fix

**Drop every `publish_*()` call out of `MqttCallback::connected()`.** The callback must return to paho quickly without reentering the client.

There are two reasonable shapes for the fix. Pick one:

### Option A — defer to the main loop (preferred; minimal change)

Add a flag and let the existing `while (ros::ok())` loop at [line 731](../open_mower_ros/src/lib/xbot_monitoring/src/xbot_monitoring.cpp#L731) observe it:

```cpp
std::atomic<bool> needs_initial_publish{false};

void connected(const mqtt::string &s) override {
    ROS_INFO_STREAM("MQTT Connected");
    needs_initial_publish.store(true, std::memory_order_release);

    client_->subscribe(this->mqtt_topic_prefix + "/teleop", 0);
    client_->subscribe(this->mqtt_topic_prefix + "/command", 0);
    client_->subscribe(this->mqtt_topic_prefix + "/action", 0);
    client_->subscribe(this->mqtt_topic_prefix + "teleop", 0);
    client_->subscribe(this->mqtt_topic_prefix + "command", 0);
    client_->subscribe(this->mqtt_topic_prefix + "action", 0);
    client_->subscribe(this->mqtt_topic_prefix + "rpc/request", 0);
}
```

In the main loop (around line 731):
```cpp
while (ros::ok()) {
    if (needs_initial_publish.exchange(false, std::memory_order_acq_rel)) {
        publish_capabilities();
        publish_sensor_metadata();
        publish_map();
        publish_map_overlay();
        publish_actions();
        publish_version();
        publish_params();
    }
    // ...existing sensor-topic-discovery code...
    sensor_check_rate.sleep();
}
```

Rationale: main thread doesn't hold paho's callback mutex, so `publish()` there contends only with the normal publisher path, never with `connected()`. No additional threads introduced. Shortest diff.

**Watch out for**: `client_->subscribe(...)` inside `connected()` is also documented as "safe during callback" by paho — keep those there. Only the `publish()` calls are the problem.

### Option B — dedicated worker thread

Introduce a `std::thread` (or `ros::AsyncSpinner(N)` with a dedicated callback queue) that drains a work queue populated from `connected()`. More flexible long-term (you can post other deferred work to it), more code, more review surface. Not required for this fix.

### Do **not**

- Don't fix this by increasing `AsyncSpinner(1)` → `AsyncSpinner(2+)`. It masks the bug (the service callbacks get processed on the second thread) but the paho deadlock between the two `publish_sensor_metadata()` call paths still exists — you'll see occasional `MQTT_send` hangs under load.
- Don't add a mutex around `publish_sensor_metadata()` in userspace. Path A is already inside paho's mutex; Path B would now need both mutexes in order → just different deadlock.

---

## 5. Also worth cleaning up (optional, same PR)

These aren't strictly required for the fix but are closely related:

- **Missing mutex on `registered_actions`** ([line 42](../open_mower_ros/src/lib/xbot_monitoring/src/xbot_monitoring.cpp#L42)). It's a `std::map` written by `registerActions()` (service callback, spinner thread) and read by `publish_actions()`. Currently papered over by single-threaded spinner; breaks if Option B is chosen. Add a `std::mutex registered_actions_mutex` to mirror what `registered_methods_mutex` does.
- **`/xbot_driver_gps/xb_pose` → `/ll/position/gps`, `/imu/data_raw` → `/ll/imu/data_raw`** topic renames happened inside `open_mower_ros` but the `openmower-gui` repo's subscribes still reference the old names. Not in this repo — flag to the GUI maintainer. Our local patch is in `openmower-gui/pkg/providers/ros.go` for reference.

---

## 6. Test plan

Before/after measurements need a reproduction environment that races reliably. Easiest recipe: OpenMowerOS v1, map with <60 points per polygon, external MQTT at a broker on the same LAN (not localhost).

1. **Hang-detection loop** — script 30 restarts of `openmower.service`, count how many reach `"Got all servers, we can mow"` within 60 s. Before fix on the reproducer: ~3/30. After fix: 30/30.
2. **GDB attach** — in `xbot_monitoring`, no thread should ever be stuck with both a paho frame *and* a `publish_sensor_metadata()` frame on its stack. Script: on every restart, after stack is up, run `gdb -batch -ex "thread apply all bt 12"` and grep for `publish_sensor_metadata` in a frame above `pthread_mutex_lock` — should not match.
3. **Service-call timing** — time `rosservice call /xbot/register_actions "{node_prefix: test, actions: []}"`. Should return in <500 ms. Currently hangs indefinitely when the deadlock is live.
4. **Log assertion** — within 10 s of "MQTT Connected", expect `new actions registered: mower_logic registered N actions` followed by `Got all servers, we can mow` and non-empty `openmower/robot_state/json` on MQTT.
5. **Regression** — a successful run should still log all six publish types (capabilities, sensor_metadata, map, map_overlay, actions, version, params) once each within 1 s of MQTT connect. Check MQTT topics with `mosquitto_sub -h <broker> -t 'openmower/#' -W 5` — compare topic list before/after.

---

## 7. Workarounds in place until this PR lands

Captured here so future-us doesn't forget to undo them after the real fix ships:

- `openmower.service` pinned to `ghcr.io/clemenselflein/open_mower_ros:1.1.1-legacy` (previously `alexvaut/...:sha-07e190a`). Service file backup at `/etc/systemd/system/openmower.service.bak-pre-legacy` on the mower.
- `OM_MQTT_HOSTNAME="127.0.0.1"` in `/boot/openmower/mower_config.txt` (was `"192.168.1.3"`, an external LAN broker). Backup at `/boot/openmower/mower_config.txt.before-1776020455`.
- Host `mosquitto` stopped and disabled (bundled mosquitto inside the container is authoritative on port 1883 now).
- MCP server host pointer updated: `mcp_server/openmower_mcp.py` `MQTT_HOST = "192.168.1.67"` (was `"192.168.1.3"`).
- `openmower-gui` patched binary bind-mounted over `/app/openmower-gui` inside the container. Source changes live in [`openmower-gui/pkg/providers/ros.go`](../openmower-gui/pkg/providers/ros.go) — only renamed two subscribe topics to match the `/ll/*` prefix the newer `open_mower_ros` uses. Revert by removing the `--volume /root/openmower-gui-patched:/app/openmower-gui:ro` line in `/etc/systemd/system/container-openmower-gui.service` (backup at `/etc/systemd/system/container-openmower-gui.service.bak-1776021460`).
- Map currently in use at `/root/ros_home/.ros/map.json` — keep it on the denser 251473 B backup (`map.json.before-1776009647`) rather than the simplified 42932 B one until the fix is in, since the simplified map triggers the race more consistently.

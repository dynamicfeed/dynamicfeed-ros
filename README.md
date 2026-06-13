# Dynamic Feed for ROS 2

Signed, fresh **situational awareness** for robots — weather, hazards, GPS-interference risk, air quality, space weather — published as **standard ROS 2 topics**, with the **Ed25519 signature verified on the robot**. Keyless. The robot-side mirror of [Dynamic Feed's](https://dynamicfeed.ai) keyless MCP for AI agents.

[![DF-VERIFY/1](https://dynamicfeed.ai/badge.svg)](https://dynamicfeed.ai/standard)

> **Verify, don't trust.** Every datapoint carries a `Provenance` envelope — source, freshness, and whether its signature verified *on this robot*. A safety-critical stack can refuse to act on stale or unverified world-state. That's [DF-VERIFY/1](https://dynamicfeed.ai/standard) for embodied AI.

## Humanoid robots

Bipedal robots operating near people face a specific set of exogenous risks — geomagnetic interference with heading sensors, GPS jamming in urban environments, air quality affecting both hardware and the people around the robot, and regional hazards (wildfire, quake, severe weather) that a local perception stack will never see coming.

Dynamic Feed's `humanoid` robot class weights exactly those factors.

### The embedding pattern

A humanoid robot's motion planner should ask "is it safe to proceed?" BEFORE committing to a waypoint or action primitive — not just subscribe to a periodic topic. The `~/check_awareness` **service** is that call:

```python
from dynamicfeed_msgs.srv import CheckAwareness

cli = node.create_client(CheckAwareness, "/dynamicfeed_awareness/check_awareness")
req = CheckAwareness.Request()
req.latitude = my_lat       # current position
req.longitude = my_lon
req.robot_class = "humanoid"

resp = cli.call(req)        # synchronous from the planner thread's perspective
# Gate on the verdict AND the verified signature
clear = (resp.verdict != "no-go") and resp.signature_valid
# Log the snapshot_id alongside your decision (audit trail)
logger.info("proceeding=%s snapshot=%s" % (clear, resp.snapshot_id))
```

### On-device verification — the value

The robot doesn't trust the data; it **proves** it.  Before acting on any verdict, the node re-derives the Ed25519 signature over the exact canonical byte sequence that Dynamic Feed signed.  A man-in-the-middle, a replay, or a single tampered byte fails the verify step on the robot — and the verdict is dropped (or overridden to `caution` when `require_signature: true`).

> The robot holds Dynamic Feed accountable for every decision it makes. That's the inverse of "just call an API".

### 60-second quickstart (humanoid)

```bash
# 1. Clone and build
git clone https://github.com/dynamicfeed/dynamicfeed-ros src/dynamicfeed-ros
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash

# 2. Edit config — set your site coordinates
nano src/dynamicfeed-ros/dynamicfeed_bringup/config/humanoid.yaml

# 3. Launch (awareness node + planner gate example)
ros2 launch dynamicfeed_bringup humanoid.launch.py

# 4. Watch the gate topic — True when verdict != "no-go" AND signature verified
ros2 topic echo /dynamic_feed/clear_to_proceed

# 5. Inject a test fix (or let your GPS driver publish to /fix)
ros2 topic pub /fix sensor_msgs/NavSatFix \
  "{header: {stamp: {sec: 0}}, latitude: 37.7749, longitude: -122.4194, altitude: 10.0}"
```

### Service: `~/check_awareness`

| Field | Type | Meaning |
|---|---|---|
| **request** `latitude` | `float64` | WGS-84 decimal degrees |
| **request** `longitude` | `float64` | WGS-84 decimal degrees |
| **request** `robot_class` | `string` | `"humanoid"` / `"ground"` / etc.; empty = node default |
| **response** `verdict` | `string` | `"go"` / `"caution"` / `"no-go"` |
| **response** `reason` | `string` | One-line explanation |
| **response** `signature_valid` | `bool` | Ed25519 verified ON THIS ROBOT |
| **response** `key_id` | `string` | Signing key id |
| **response** `snapshot_id` | `string` | Hash of the exact signed data — **log this with every decision** |
| **response** `fact_ids` | `string[]` | Grounded fact IDs that drove the verdict |
| **response** `fact_summaries` | `string[]` | Human-readable summary of each fact |
| **response** `degraded` | `bool` | True if Dynamic Feed returned partial/degraded data |

### Safety framing (inviolable)

> **Dynamic Feed provides verifiable situational data as a decision INPUT. It is NOT a safety certification or guarantee — your robot's own safety stack owns the final decision. Verdicts degrade to "caution" on uncertainty and never fabricate "go".**

A `"go"` verdict means: at query time, Dynamic Feed's signed feeds contained no known disqualifying condition for this location and robot class.  It does not mean the environment is physically safe.  Your robot's collision avoidance, joint torque limits, and emergency stop are not replaced by this package.

### Example node

`dynamicfeed_bringup/examples/humanoid_example_node.py` is a documented, copy-pasteable pattern showing the full integration: subscribe to `/fix`, call the service, evaluate `(verdict != "no-go") AND signature_valid`, publish `/dynamic_feed/clear_to_proceed`.  Copy it into your own package rather than depending on it as a production node.

## Why

A robot acting outdoors needs to know what's around it that its own sensors can't see: a wildfire upwind, a weather alert, GNSS jamming in the area, a geomagnetic storm degrading its fix. Dynamic Feed already serves that, signed. This package meets robots where they live — as a ROS 2 node any robot can run, publishing into the message types an existing nav/perception stack already understands.

## Quickstart (v0.1 — source build)

```bash
# in your colcon workspace
git clone https://github.com/dynamicfeed/dynamicfeed-ros src/dynamicfeed-ros
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select dynamicfeed_msgs dynamicfeed_awareness
source install/setup.bash

ros2 run dynamicfeed_awareness awareness_node --ros-args \
  -p latitude:=51.5 -p longitude:=-0.12 -p robot_class:=aerial
```

> Once bloom-released into `rosdistro`, the whole install becomes `sudo apt install ros-jazzy-dynamicfeed-awareness` — present in the index every ROS robot already pulls from, keyless.

## Topics and services

### Topics

| Topic | Type | Source feed |
|---|---|---|
| `~/weather/temperature` | `sensor_msgs/Temperature` | weather (when provided) |
| `~/weather/humidity` | `sensor_msgs/RelativeHumidity` | weather (when provided) |
| `~/weather/wind` | `geometry_msgs/Vector3Stamped` (m/s) | weather |
| `~/air_quality` | `dynamicfeed_msgs/AirQuality` | air quality |
| `~/space_weather` | `dynamicfeed_msgs/SpaceWeather` | NOAA SWPC |
| `~/gps_interference` | `dynamicfeed_msgs/GpsInterference` | ADS-B NIC/NACp |
| `~/hazards` | `dynamicfeed_msgs/HazardAlert` | quakes / wildfires / alerts / volcanoes |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | verdict + signature/freshness state |

Standard `sensor_msgs` are used wherever ROS already has a type (so off-the-shelf stacks consume Dynamic Feed with zero DF-specific code); custom `dynamicfeed_msgs` appear only where ROS has nothing.

### Service

| Service | Type | Use |
|---|---|---|
| `~/check_awareness` | `dynamicfeed_msgs/CheckAwareness` | Synchronous planner query — request a verdict for a specific (lat, lon, robot_class) right now, get back verdict + signature_valid + snapshot_id. |

Use the **topic** for continuous environmental monitoring (weather dashboard, hazard overlay).  Use the **service** for gated planner decisions ("am I clear to proceed to this waypoint?") where you need a fresh, logged verdict at the moment of decision.

## Parameters

| Param | Default | Notes |
|---|---|---|
| `latitude`, `longitude` | `51.5`, `-0.12` | the robot's location |
| `robot_class` | `ground` | `ground` / `aerial` / `marine` / `orbital` / `humanoid` |
| `base_url` | `https://dynamicfeed.ai` | |
| `poll_period_s` | `60.0` | |
| `require_signature` | `true` | if true, **drop** data whose signature didn't verify |

## Trust model

The signing key is **pinned** in `verify.py` (`df-ed25519-4cb32e72f333`); the node opportunistically refreshes from the issuer JWKS but the pinned key is what it trusts — so a network attacker can't substitute a key. Verify the pin against https://dynamicfeed.ai/.well-known/keys. The verification (strip `signature`, canonicalize json-sorted-compact, check Ed25519) is byte-identical to the reference verifiers at https://dynamicfeed.ai/standard.

## Status

v0.1 MVP — targets **ROS 2 Jazzy** (broad LTS) and **Humble** (large install base), Lyrical/Kilted forward. Source build today; rosdistro release is the distribution goal. The `dynamicfeed_msgs` definitions are the **open interface contract**: if `HazardAlert` + the `Provenance` envelope become how robots expect signed external awareness to arrive, the interface is the standard — not just one feed.

## License

MIT.

# Dynamic Feed for ROS 2

Signed, fresh **situational awareness** for robots — weather, hazards, GPS-interference risk, air quality, space weather — published as **standard ROS 2 topics**, with the **Ed25519 signature verified on the robot**. Keyless. The robot-side mirror of [Dynamic Feed's](https://dynamicfeed.ai) keyless MCP for AI agents.

[![DF-VERIFY/1](https://dynamicfeed.ai/badge.svg)](https://dynamicfeed.ai/standard)

> **Verify, don't trust.** Every datapoint carries a `Provenance` envelope — source, freshness, and whether its signature verified *on this robot*. A safety-critical stack can refuse to act on stale or unverified world-state. That's [DF-VERIFY/1](https://dynamicfeed.ai/standard) for embodied AI.

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

## Topics

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

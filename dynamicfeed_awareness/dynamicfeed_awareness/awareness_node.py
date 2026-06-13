#!/usr/bin/env python3
"""
dynamicfeed_awareness — ROS 2 node publishing Dynamic Feed signed situational awareness.

Polls POST /v1/awareness for the robot's location, VERIFIES the Ed25519 signature on the robot
(verify, don't trust), and republishes the grounded facts as standard ROS messages — so an
existing nav/perception stack consumes them with no Dynamic-Feed-specific code. Every custom
message carries a Provenance envelope (source, freshness, signature_valid). If the signature does
not verify and require_signature is true, the node raises a diagnostic ERROR and drops the data.

Parameters:
  latitude (float, 51.5), longitude (float, -0.12)   the robot's location
  robot_class (str, 'ground')                          ground|aerial|marine|orbital|humanoid
  base_url (str, https://dynamicfeed.ai)
  poll_period_s (float, 60.0)
  require_signature (bool, true)                       drop data whose signature didn't verify

Topics (standard sensor_msgs where they exist; custom dynamicfeed_msgs where ROS has nothing):
  ~/weather/temperature  sensor_msgs/Temperature        (when the feed provides it)
  ~/weather/humidity     sensor_msgs/RelativeHumidity   (when provided)
  ~/weather/wind         geometry_msgs/Vector3Stamped   m/s
  ~/air_quality          dynamicfeed_msgs/AirQuality
  ~/space_weather        dynamicfeed_msgs/SpaceWeather
  ~/gps_interference     dynamicfeed_msgs/GpsInterference
  ~/hazards              dynamicfeed_msgs/HazardAlert
  /diagnostics           diagnostic_msgs/DiagnosticArray  overall verdict + signature/freshness state

Services:
  ~/check_awareness      dynamicfeed_msgs/CheckAwareness
      Synchronous planner-facing service: "is it safe to proceed at this location right now?"
      Request carries lat/lon/robot_class; response includes verdict (go/caution/no-go),
      signature_valid, snapshot_id, and grounded fact summaries. A planner should log the
      snapshot_id alongside every action it gates on this verdict.

      SAFETY CONTRACT: verdict "go" means no known disqualifying conditions at query time.
      It is a decision INPUT, not a safety certification.  Your robot's own safety stack owns
      the final decision.  Verdicts degrade to "caution" on uncertainty; they never fabricate
      "go" from insufficient data.
"""
import datetime as _dt
import json
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from sensor_msgs.msg import Temperature, RelativeHumidity
from geometry_msgs.msg import Vector3Stamped
from geographic_msgs.msg import GeoPoint
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from dynamicfeed_msgs.msg import Provenance, AirQuality, SpaceWeather, GpsInterference, HazardAlert
from dynamicfeed_msgs.srv import CheckAwareness

from . import verify as dfverify

NAN = float("nan")
# Latched QoS for alerts so a late-joining node still receives the last hazard / interference message.
LATCHED = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, history=QoSHistoryPolicy.KEEP_LAST)


def _iso_to_time(s):
    """Parse an ISO 8601 timestamp to builtin_interfaces/Time; zero Time if unparseable."""
    t = Time()
    try:
        dt = _dt.datetime.fromisoformat(str(s).replace("Z", "+00:00"))
        epoch = dt.timestamp()
        t.sec = int(epoch)
        t.nanosec = int((epoch - int(epoch)) * 1e9)
    except Exception:
        pass
    return t


def _num(fact, default=NAN):
    try:
        return float(fact.get("value"))
    except (TypeError, ValueError, AttributeError):
        return default


class AwarenessNode(Node):
    def __init__(self):
        super().__init__("dynamicfeed_awareness")
        self.declare_parameter("latitude", 51.5)
        self.declare_parameter("longitude", -0.12)
        self.declare_parameter("robot_class", "ground")
        self.declare_parameter("base_url", dfverify.DEFAULT_BASE)
        self.declare_parameter("poll_period_s", 60.0)
        self.declare_parameter("require_signature", True)
        self.declare_parameter("nearby_radius_km", 500.0)

        self.base = str(self.get_parameter("base_url").value).rstrip("/")
        self.require_sig = bool(self.get_parameter("require_signature").value)

        # Trust the PINNED key; opportunistically refresh from the issuer JWKS (pinned still trusted).
        self.keys = dict(dfverify.PINNED_KEYS)
        try:
            self.keys.update(dfverify.fetch_keys(self.base))
        except Exception as e:  # offline / unreachable — pinned key still works
            self.get_logger().warn("could not fetch JWKS (%s); using pinned key only" % e)

        self.pub_temp = self.create_publisher(Temperature, "/dynamic_feed/weather/temperature", qos_profile_sensor_data)
        self.pub_hum = self.create_publisher(RelativeHumidity, "/dynamic_feed/weather/humidity", qos_profile_sensor_data)
        self.pub_wind = self.create_publisher(Vector3Stamped, "/dynamic_feed/weather/wind", qos_profile_sensor_data)
        self.pub_aq = self.create_publisher(AirQuality, "/dynamic_feed/air_quality", qos_profile_sensor_data)
        self.pub_sw = self.create_publisher(SpaceWeather, "/dynamic_feed/space_weather", qos_profile_sensor_data)
        self.pub_gps = self.create_publisher(GpsInterference, "/dynamic_feed/gps_interference", LATCHED)
        self.pub_haz = self.create_publisher(HazardAlert, "/dynamic_feed/hazards", LATCHED)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        period = float(self.get_parameter("poll_period_s").value)
        self.timer = self.create_timer(period, self.poll)

        # ~/check_awareness service — synchronous planner-facing query.
        # A robot's motion planner calls this BEFORE committing to an action: give me the verdict
        # for this (lat, lon, robot_class) right now, with the snapshot_id I can log alongside
        # my decision so it's auditable.
        self._srv_check = self.create_service(
            CheckAwareness, "~/check_awareness", self._handle_check_awareness)

        self.get_logger().info(
            "dynamicfeed_awareness up — polling %s/v1/awareness every %.0fs" % (self.base, period))

    def _hdr(self):
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = "dynamicfeed"
        return h

    def _fetch(self):
        body = json.dumps({
            "robot": {"class": self.get_parameter("robot_class").value},
            "location": {"lat": self.get_parameter("latitude").value,
                         "lon": self.get_parameter("longitude").value},
        }).encode("utf-8")
        req = urllib.request.Request(self.base + "/v1/awareness", data=body,
                                     headers={"Content-Type": "application/json",
                                              "User-Agent": "dynamicfeed-ros/0.1 (+https://dynamicfeed.ai)"})
        with urllib.request.urlopen(req, timeout=20) as r:
            return r.read().decode("utf-8")

    def _prov(self, fact, sig_ok, kid, snap):
        p = Provenance()
        fact = fact or {}
        p.source = str(fact.get("source", ""))
        p.source_url = str(fact.get("source_url", ""))
        try:
            p.age_seconds = float(fact.get("age_s") or 0.0)
        except (TypeError, ValueError):
            p.age_seconds = 0.0
        p.freshness_state = Provenance.FRESHNESS_STALE if fact.get("stale") else Provenance.FRESHNESS_LIVE
        p.max_age_seconds = NAN
        p.signature_valid = bool(sig_ok)
        p.key_id = kid or ""
        p.snapshot_id = snap or ""
        p.canonicalization = "json-sorted-compact"
        p.measured_at = _iso_to_time(fact.get("observed_at"))
        p.reported_at = _iso_to_time(fact.get("observed_at"))
        return p

    def _publish_nearby(self):
        """Fetch /v1/nearby, verify its signature, and publish each hazard as a HazardAlert."""
        radius = float(self.get_parameter("nearby_radius_km").value)
        url = "%s/v1/nearby?lat=%s&lon=%s&radius_km=%s" % (
            self.base, self.get_parameter("latitude").value,
            self.get_parameter("longitude").value, radius)
        req = urllib.request.Request(url, headers={"User-Agent": "dynamicfeed-ros/0.1 (+https://dynamicfeed.ai)"})
        try:
            with urllib.request.urlopen(req, timeout=20) as r:
                env = json.loads(r.read().decode("utf-8"))
        except Exception as e:
            self.get_logger().warn("nearby fetch failed: %s" % e)
            return
        sig_ok, detail = dfverify.verify(env, self.keys)
        if not sig_ok and self.require_sig:
            self.get_logger().error("nearby signature unverified (%s) — dropping hazards" % detail)
            return
        kid = (env.get("signature") or {}).get("key_id", "")
        snap = env.get("snapshot_id", "")
        for hz in (env.get("hazards") or []):
            h = HazardAlert()
            h.header = self._hdr()
            h.category = str(hz.get("category", "hazard"))
            h.severity = str(hz.get("severity", ""))
            h.headline = str(hz.get("headline", ""))
            loc = GeoPoint()
            loc.latitude = float(hz.get("latitude") or 0.0)
            loc.longitude = float(hz.get("longitude") or 0.0)
            loc.altitude = 0.0
            h.location = loc
            h.distance_km = float(hz["distance_km"]) if hz.get("distance_km") is not None else NAN
            h.magnitude = float(hz["magnitude"]) if hz.get("magnitude") is not None else NAN
            h.provenance = self._prov({"source": hz.get("source"), "observed_at": hz.get("time")}, sig_ok, kid, snap)
            self.pub_haz.publish(h)

    def poll(self):
        try:
            env = json.loads(self._fetch())
        except Exception as e:
            self._diag(DiagnosticStatus.ERROR, "fetch failed: %s" % e, {})
            return
        sig_ok, detail = dfverify.verify(env, self.keys)
        kid = (env.get("signature") or {}).get("key_id", "")
        snap = env.get("snapshot_id", "")
        if not sig_ok and self.require_sig:
            self.get_logger().error("awareness signature unverified (%s) — NOT publishing" % detail)
            self._diag(DiagnosticStatus.ERROR, "signature unverified — data dropped: %s" % detail,
                       {"signature_valid": "false"})
            return

        facts = {f.get("id"): f for f in (env.get("facts") or [])}

        if "wx.temp_c" in facts:
            m = Temperature(); m.header = self._hdr(); m.temperature = _num(facts["wx.temp_c"], 0.0); m.variance = 0.0
            self.pub_temp.publish(m)
        if "wx.humidity" in facts:
            m = RelativeHumidity(); m.header = self._hdr()
            m.relative_humidity = _num(facts["wx.humidity"], 0.0) / 100.0; m.variance = 0.0
            self.pub_hum.publish(m)
        if "wx.wind_kmh" in facts:
            v = Vector3Stamped(); v.header = self._hdr()
            v.vector.x = _num(facts["wx.wind_kmh"], 0.0) / 3.6  # km/h -> m/s (speed; bearing not provided)
            self.pub_wind.publish(v)
        if "aq.us_aqi" in facts:
            aq = facts["aq.us_aqi"]; m = AirQuality(); m.header = self._hdr()
            m.us_aqi = _num(aq, 0.0); m.category = str(aq.get("category", "")); m.pm2_5 = NAN; m.pm10 = NAN
            m.provenance = self._prov(aq, sig_ok, kid, snap); self.pub_aq.publish(m)
        if "sw.kp_index" in facts:
            m = SpaceWeather(); m.header = self._hdr(); m.kp_index = _num(facts["sw.kp_index"], 0.0)
            m.storm_g = int(_num(facts.get("sw.storm_g"), 0) or 0); m.storm_s = int(_num(facts.get("sw.storm_s"), 0) or 0)
            m.provenance = self._prov(facts["sw.kp_index"], sig_ok, kid, snap); self.pub_sw.publish(m)
        gi = facts.get("gps.interference") or facts.get("gps.nic")
        if gi is not None:
            m = GpsInterference(); m.header = self._hdr()
            m.latitude = float(self.get_parameter("latitude").value)
            m.longitude = float(self.get_parameter("longitude").value)
            m.level = int(_num({"value": gi.get("level")}, 0) or 0); m.confidence = float(gi.get("confidence") or 0.0)
            m.method = "adsb-nic-nacp"; m.provenance = self._prov(gi, sig_ok, kid, snap)
            self.pub_gps.publish(m)
        self._publish_nearby()

        verdict = (env.get("verdict") or {}).get("status", "?")
        degraded = bool(env.get("degraded"))
        level = DiagnosticStatus.OK if (sig_ok and not degraded) else DiagnosticStatus.WARN
        self._diag(level, "verdict=%s signature=%s degraded=%s"
                   % (verdict, "valid" if sig_ok else detail, degraded),
                   {"verdict": str(verdict), "signature_valid": str(bool(sig_ok)).lower(),
                    "key_id": kid, "snapshot_id": snap, "degraded": str(degraded).lower()})

    def _handle_check_awareness(self, request, response):
        """
        Service handler for ~/check_awareness (dynamicfeed_msgs/CheckAwareness).

        Makes a FRESH /v1/awareness call for the requested location and robot_class, verifies the
        Ed25519 signature on this robot, and fills the response.  The call is synchronous from the
        caller's perspective — the timer-driven background topic is independent of this path.

        The caller SHOULD:
          - log response.snapshot_id alongside any action taken (audit trail)
          - gate on (response.verdict != "no-go") AND response.signature_valid when require_signature=true
          - treat a service timeout as CAUTION (same as the API's own degraded-floor rule)

        The caller MUST NOT treat a "go" verdict as a safety certification. Dynamic Feed provides
        verifiable situational data as a decision INPUT; your robot's safety stack owns the final
        decision.
        """
        # Resolve lat/lon/robot_class — use request values if non-default, else node defaults.
        lat = request.latitude if request.latitude != 0.0 else float(self.get_parameter("latitude").value)
        lon = request.longitude if request.longitude != 0.0 else float(self.get_parameter("longitude").value)
        robot_class = (request.robot_class.strip() or
                       str(self.get_parameter("robot_class").value))

        # Build and execute the awareness request.
        body = json.dumps({
            "robot": {"class": robot_class},
            "location": {"lat": lat, "lon": lon},
        }).encode("utf-8")
        req = urllib.request.Request(
            self.base + "/v1/awareness", data=body,
            headers={"Content-Type": "application/json",
                     "User-Agent": "dynamicfeed-ros/0.1 (+https://dynamicfeed.ai)"})
        try:
            with urllib.request.urlopen(req, timeout=20) as r:
                env = json.loads(r.read().decode("utf-8"))
        except Exception as e:
            # Network or parse error — respond with caution so a caller polling with a timeout
            # gets the degraded-floor behaviour rather than a hard exception.
            self.get_logger().warn("check_awareness fetch failed: %s" % e)
            response.verdict = "caution"
            response.reason = "fetch failed: %s" % e
            response.signature_valid = False
            response.key_id = ""
            response.snapshot_id = ""
            response.fact_ids = []
            response.fact_summaries = []
            response.degraded = True
            return response

        sig_ok, detail = dfverify.verify(env, self.keys)
        kid = (env.get("signature") or {}).get("key_id", "")
        snap = env.get("snapshot_id", "")
        degraded = bool(env.get("degraded"))

        verdict_block = env.get("verdict") or {}
        verdict = str(verdict_block.get("status", "caution"))
        reason = str(verdict_block.get("reason", ""))

        # If require_signature is set and the signature didn't verify, override the verdict to
        # "caution" — never let unverified data produce a "go".
        if not sig_ok and self.require_sig:
            self.get_logger().error(
                "check_awareness: signature unverified (%s) — verdict overridden to caution" % detail)
            verdict = "caution"
            reason = "signature unverified: %s" % detail
            degraded = True

        # Summarise the grounded facts for the caller's log / planner context.
        fact_ids = []
        fact_summaries = []
        for f in (env.get("facts") or []):
            fid = str(f.get("id", ""))
            label = str(f.get("label") or f.get("id") or "")
            value = f.get("value")
            unit = str(f.get("unit") or "")
            summary = "%s=%s%s" % (label, value, (" " + unit) if unit else "")
            fact_ids.append(fid)
            fact_summaries.append(summary)

        response.verdict = verdict
        response.reason = reason
        response.signature_valid = bool(sig_ok)
        response.key_id = kid
        response.snapshot_id = snap
        response.fact_ids = fact_ids
        response.fact_summaries = fact_summaries
        response.degraded = degraded

        self.get_logger().info(
            "check_awareness: lat=%.4f lon=%.4f class=%s → verdict=%s sig=%s snap=%s"
            % (lat, lon, robot_class, verdict, "valid" if sig_ok else "INVALID", snap))
        return response

    def _diag(self, level, message, values):
        arr = DiagnosticArray(); arr.header = self._hdr()
        st = DiagnosticStatus(); st.level = level; st.name = "dynamicfeed: awareness"
        st.message = message; st.hardware_id = "dynamicfeed.ai"
        st.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        arr.status = [st]
        self.pub_diag.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = AwarenessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

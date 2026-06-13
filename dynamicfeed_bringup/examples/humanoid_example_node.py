#!/usr/bin/env python3
"""
humanoid_example_node.py — Dynamic Feed situational awareness for a bipedal/humanoid robot.

WHAT THIS EXAMPLE SHOWS
-----------------------
Pattern: a robot's planner asks "is it safe to proceed to waypoint X?" BEFORE committing to
a motion primitive.  Dynamic Feed provides the answer as a verified, signed data point.

Concretely this node does:
  1. Subscribes to /fix (sensor_msgs/NavSatFix) — the robot's live GNSS position.
     For an indoor robot without GPS, a static location from the config file is used instead.

  2. On each new /fix, calls the ~/check_awareness SERVICE (dynamicfeed_msgs/CheckAwareness)
     with the robot's current lat/lon and robot_class="humanoid".

  3. The service returns:
       - verdict      : "go" | "caution" | "no-go"
       - signature_valid : Ed25519 verified ON THIS ROBOT (not trusted from the network)
       - snapshot_id  : hash of the exact signed data used — log this with every decision
       - fact_ids / fact_summaries : the grounded facts that drove the verdict

  4. Gates a std_msgs/Bool on /dynamic_feed/clear_to_proceed:
       clear = (verdict != "no-go") AND (signature_valid OR require_signature is false)
     A downstream planner subscribes to this topic.  If it goes False, the planner should
     pause and request human confirmation before resuming.

  5. Every decision (verdict, snapshot_id, clear flag) is logged to /rosout at INFO level
     so it appears in any bag file.

SAFETY CONTRACT (INVIOLABLE)
-----------------------------
Dynamic Feed provides verifiable situational data as a decision INPUT.
It is NOT a safety certification or guarantee — your robot's own safety stack owns the
final decision.  Verdicts degrade to "caution" on uncertainty and never fabricate "go".

QUICKSTART (60 seconds)
-----------------------
  # In your colcon workspace:
  git clone https://github.com/dynamicfeed/dynamicfeed-ros src/dynamicfeed-ros
  rosdep install --from-paths src --ignore-src -r -y
  colcon build
  source install/setup.bash

  # Terminal 1: launch the awareness node (humanoid config)
  ros2 launch dynamicfeed_bringup humanoid.launch.py

  # Terminal 2: run this example node
  ros2 run dynamicfeed_bringup humanoid_example_node

  # Terminal 3: watch the gate topic
  ros2 topic echo /dynamic_feed/clear_to_proceed

  # Optionally: inject a static GPS fix for testing (no physical GPS needed)
  ros2 topic pub /fix sensor_msgs/NavSatFix \
    "{header: {stamp: {sec: 0}}, latitude: 37.7749, longitude: -122.4194, altitude: 10.0}"

ROS GRAPH
---------
  /fix  ──(NavSatFix)──▶  [humanoid_example_node]  ──(Bool)──▶  /dynamic_feed/clear_to_proceed
                                    │
                          ~/check_awareness (srv call) ──▶  [dynamicfeed_awareness node]
                                                                       │
                                                           POST /v1/awareness (HTTPS)
                                                           Ed25519 verify on-robot
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from dynamicfeed_msgs.srv import CheckAwareness


class HumanoidExampleNode(Node):
    """
    Example: gate a clear_to_proceed topic on a Dynamic Feed awareness verdict.

    This is intentionally simple — copy the _on_fix → _request_awareness → _publish_gate
    pattern into your own planner node rather than depending on this node directly.
    """

    def __init__(self):
        super().__init__("humanoid_awareness_example")

        # ── Parameters ────────────────────────────────────────────────────────────────────────
        # fallback_lat/lon: used when no NavSatFix is available (indoor robot, GPS not locked).
        # robot_class: override here if this node is embedded in a larger stack.
        self.declare_parameter("fallback_latitude", 37.7749)
        self.declare_parameter("fallback_longitude", -122.4194)
        self.declare_parameter("robot_class", "humanoid")
        # require_signature_gate: if True, a signature failure alone sets clear=False even if
        # verdict is "go".  Matches the awareness node's own require_signature logic.
        self.declare_parameter("require_signature_gate", True)

        self._require_sig = bool(self.get_parameter("require_signature_gate").value)
        self._robot_class = str(self.get_parameter("robot_class").value)

        # ── Subscriptions ─────────────────────────────────────────────────────────────────────
        # /fix — the robot's GNSS position.  Use qos_profile_sensor_data (best-effort) to match
        # typical GNSS driver publishers.
        self._sub_fix = self.create_subscription(
            NavSatFix, "/fix", self._on_fix, qos_profile_sensor_data)

        # ── Publishers ────────────────────────────────────────────────────────────────────────
        # /dynamic_feed/clear_to_proceed — True when verdict != "no-go" AND sig valid (or not required).
        # Latched (TRANSIENT_LOCAL) so a late-joining subscriber immediately gets the last value.
        from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                               QoSReliabilityPolicy, QoSHistoryPolicy)
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self._pub_clear = self.create_publisher(Bool, "/dynamic_feed/clear_to_proceed", latched)

        # ── Service client ────────────────────────────────────────────────────────────────────
        # ~/check_awareness — provided by the dynamicfeed_awareness node.
        # The service is in the awareness node's namespace; adjust if you remap it.
        self._cli = self.create_client(CheckAwareness, "/dynamicfeed_awareness/check_awareness")

        # Publish a conservative initial value (False) so the planner doesn't act before the
        # first verdict arrives.
        self._publish_gate(clear=False, reason="initialising — waiting for first verdict")

        # ── Periodic fallback poll ────────────────────────────────────────────────────────────
        # If no NavSatFix arrives (indoor / GPS-denied), poll every 60 s using the fallback
        # coordinates so the gate topic stays current.
        self._last_fix_lat = self.get_parameter("fallback_latitude").value
        self._last_fix_lon = self.get_parameter("fallback_longitude").value
        self._fix_received = False
        self._timer = self.create_timer(60.0, self._periodic_poll)

        self.get_logger().info(
            "humanoid_example_node up — waiting for /fix or polling every 60 s "
            "(fallback %.4f, %.4f)" % (self._last_fix_lat, self._last_fix_lon))

    # ── Callbacks ─────────────────────────────────────────────────────────────────────────────

    def _on_fix(self, msg: NavSatFix):
        """
        Called on each NavSatFix.  Forward the position to the awareness service.

        We skip GNSS_FIX_NONE (status -1) because a bad fix would query the wrong location.
        The planner should treat the resulting conservative "clear=False" as expected behaviour
        when GPS is lost.
        """
        if msg.status.status < 0:
            self.get_logger().debug("GPS fix not valid (status=%d) — skipping" % msg.status.status)
            return

        self._last_fix_lat = msg.latitude
        self._last_fix_lon = msg.longitude
        self._fix_received = True
        self._request_awareness(msg.latitude, msg.longitude)

    def _periodic_poll(self):
        """
        Fallback poll — fires every 60 s regardless of GPS.
        Uses the last known position (or the fallback parameter if GPS never locked).
        """
        if not self._fix_received:
            self.get_logger().debug(
                "No NavSatFix yet — polling with fallback coordinates "
                "(%.4f, %.4f)" % (self._last_fix_lat, self._last_fix_lon))
        self._request_awareness(self._last_fix_lat, self._last_fix_lon)

    def _request_awareness(self, lat: float, lon: float):
        """
        Call ~/check_awareness for (lat, lon, robot_class).

        The call is ASYNCHRONOUS (send_request_async + add_done_callback) so this node doesn't
        block the ROS spin thread.  The callback fires when the service responds.
        """
        if not self._cli.service_is_ready():
            self.get_logger().warn(
                "check_awareness service not available — is dynamicfeed_awareness running? "
                "Holding clear_to_proceed=False.")
            self._publish_gate(clear=False, reason="service unavailable")
            return

        req = CheckAwareness.Request()
        req.latitude = float(lat)
        req.longitude = float(lon)
        req.robot_class = self._robot_class

        future = self._cli.call_async(req)
        future.add_done_callback(lambda f: self._on_awareness_response(f, lat, lon))

    def _on_awareness_response(self, future, lat, lon):
        """
        Process the CheckAwareness response and update the clear_to_proceed gate.

        Gate logic (both conditions must hold):
          1. verdict is not "no-go"
          2. signature_valid OR require_signature_gate is False

        Log the snapshot_id with every decision — this is your audit trail linking the action
        to the exact signed data that authorised it.
        """
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error("check_awareness call failed: %s — holding False" % e)
            self._publish_gate(clear=False, reason="service call exception: %s" % e)
            return

        # ── Gate evaluation ───────────────────────────────────────────────────────────────────
        verdict_ok = (resp.verdict != "no-go")
        sig_ok = resp.signature_valid or not self._require_sig
        clear = verdict_ok and sig_ok

        # ── Audit log ─────────────────────────────────────────────────────────────────────────
        # Everything a safety auditor needs to reconstruct this decision:
        self.get_logger().info(
            "[DECISION] lat=%.5f lon=%.5f robot=%s | "
            "verdict=%s sig_valid=%s degraded=%s | "
            "clear_to_proceed=%s | snapshot=%s | reason: %s"
            % (lat, lon, self._robot_class,
               resp.verdict, resp.signature_valid, resp.degraded,
               clear, resp.snapshot_id, resp.reason))

        if resp.fact_summaries:
            self.get_logger().info(
                "[FACTS] %s" % " | ".join(resp.fact_summaries))

        if not sig_ok and self._require_sig:
            self.get_logger().warn(
                "Signature verification FAILED (key=%s) — clear_to_proceed=False regardless of verdict"
                % resp.key_id)

        self._publish_gate(
            clear=clear,
            reason="verdict=%s sig=%s snap=%s" % (
                resp.verdict, "valid" if resp.signature_valid else "INVALID", resp.snapshot_id))

    def _publish_gate(self, clear: bool, reason: str = ""):
        """Publish the latched clear_to_proceed Bool."""
        msg = Bool()
        msg.data = clear
        self._pub_clear.publish(msg)
        self.get_logger().info("clear_to_proceed=%s (%s)" % (clear, reason))


def main(args=None):
    rclpy.init(args=args)
    node = HumanoidExampleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

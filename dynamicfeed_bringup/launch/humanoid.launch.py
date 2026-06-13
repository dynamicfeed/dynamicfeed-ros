"""
humanoid.launch.py — bring up Dynamic Feed situational awareness for a bipedal/humanoid robot.

Launches two nodes:
  1. dynamicfeed_awareness  — polls Dynamic Feed, verifies Ed25519 signatures on-robot, publishes
                               topics + exposes ~/check_awareness service.
  2. humanoid_awareness_example — subscribes to /fix (NavSatFix), calls the awareness service,
                                   publishes /dynamic_feed/clear_to_proceed (Bool, latched).

Tune config/humanoid.yaml for your operating site (latitude, longitude, poll_period_s).
For a robot with live GPS, the example node reads position from /fix automatically.
For an indoor robot, set fallback_latitude and fallback_longitude in that file and the example
node will poll at the configured interval without GPS.

SAFETY NOTE
-----------
Dynamic Feed provides verifiable situational data as a decision INPUT.
It is NOT a safety certification or guarantee — your robot's own safety stack owns the final
decision.  Verdicts degrade to "caution" on uncertainty and never fabricate "go".
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("dynamicfeed_bringup")
    humanoid_cfg = os.path.join(share, "config", "humanoid.yaml")

    return LaunchDescription([
        # ── 1. Awareness node ─────────────────────────────────────────────────────────────────
        # Polls /v1/awareness, verifies Ed25519 on-robot, publishes sensor topics + service.
        Node(
            package="dynamicfeed_awareness",
            executable="awareness_node",
            name="dynamicfeed_awareness",
            parameters=[humanoid_cfg],
            output="screen",
        ),

        # ── 2. Humanoid example / planner gate ────────────────────────────────────────────────
        # Subscribes to /fix, calls ~/check_awareness, publishes /dynamic_feed/clear_to_proceed.
        # COPY this pattern into your own planner node — don't depend on this example node in
        # production code.
        Node(
            package="dynamicfeed_bringup",
            executable="humanoid_example_node",
            name="humanoid_awareness_example",
            parameters=[
                humanoid_cfg,
                {
                    # These override what's in humanoid.yaml; here they just make the intent clear.
                    "robot_class": "humanoid",
                    "require_signature_gate": True,
                },
            ],
            output="screen",
        ),
    ])

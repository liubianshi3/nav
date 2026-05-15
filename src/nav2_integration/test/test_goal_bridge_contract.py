from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped

from nav2_integration.goal_bridge import normalize_quaternion, validate_goal_contract


def make_goal(frame_id="map", x=1.0, y=2.0):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = 1
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = 1.0
    return msg


def test_goal_contract_defaults_empty_frame_to_map():
    goal = make_goal(frame_id="")
    validated, reason = validate_goal_contract(
        goal,
        map_frame="map",
        require_map_frame=True,
        max_goal_distance_from_origin=10.0,
    )
    assert validated is goal
    assert goal.header.frame_id == "map"
    assert reason == "action_goal_dispatched"


def test_goal_contract_rejects_bad_frame_nonfinite_and_bounds():
    bad_frame = make_goal(frame_id="odom")
    assert validate_goal_contract(
        bad_frame,
        map_frame="map",
        require_map_frame=True,
        max_goal_distance_from_origin=10.0,
    )[1] == "bad_frame:odom"

    nonfinite = make_goal(x=math.nan)
    assert validate_goal_contract(
        nonfinite,
        map_frame="map",
        require_map_frame=True,
        max_goal_distance_from_origin=10.0,
    )[1] == "nonfinite_goal_position"

    far = make_goal(x=20.0)
    assert validate_goal_contract(
        far,
        map_frame="map",
        require_map_frame=True,
        max_goal_distance_from_origin=10.0,
    )[1] == "goal_out_of_configured_bounds"


def test_zero_quaternion_is_defaulted():
    goal = make_goal()
    goal.pose.orientation.w = 0.0
    assert normalize_quaternion(goal.pose.orientation) is False
    assert goal.pose.orientation.w == 1.0

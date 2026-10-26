"""Confirmed observations abort Exploration through DDS and recover LOITER."""

from copy import deepcopy
import threading
import time

from action_msgs.msg import GoalStatus
from kv_executive.detection_monitor import DetectionMonitor
from kv_executive.mission_geometry import yaw_from_quaternion
from kv_executive.msg import FireSourceReport, MissionPhase, MissionStateRequest
from kv_perception.msg import FireDetection, SmokeDetection
import pytest
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Header

from test_mission_action import (  # noqa: F401
    mission, mission_result, prepare_airborne, received_state, send_goal, wait_for,
)


def observation(source, stamp, confirmed=True):
    header = Header(stamp=stamp, frame_id=f'{source}_camera_optical_frame')
    if source == 'smoke':
        return SmokeDetection(
            header=header, detected=True, background_ready=True,
            status=SmokeDetection.CONFIRMED if confirmed else SmokeDetection.CONFIRMING,
            confirmation_frames=3, region=SmokeDetection.TOP_LEFT,
            regions=[SmokeDetection.TOP_LEFT], coverage=0.01,
            quadrant_coverage=[0.04, 0.0, 0.0, 0.0],
        )
    return FireDetection(
        header=header, detected=True,
        status=FireDetection.CONFIRMED if confirmed else FireDetection.CONFIRMING,
        confirmation_frames=16, persistence_sec=1.5, area_px=64,
        track_id=1 if confirmed else 0, source_count=1 if confirmed else 0,
        image_width=160, image_height=120, bbox_x=30, bbox_y=40,
        bbox_width=8, bbox_height=8, center_x=33.5, center_y=43.5,
    )


def attach_monitor(flight):
    """Keep the monitor alive until the fixture drains all executor callbacks."""
    monitor = DetectionMonitor(namespace=flight.fcu.get_namespace())
    flight.executive.executor.add_node(monitor)
    original_destroy = flight.fcu.destroy_node

    def destroy_monitor_and_fcu():
        try:
            monitor.destroy_node()
        finally:
            original_destroy()

    flight.fcu.destroy_node = destroy_monitor_and_fcu
    requests, phases = [], []
    fcu = flight.fcu
    fcu.create_subscription(
        MissionStateRequest, 'mission_state_request_for_goal',
        lambda message: requests.append(message), 10,
        callback_group=fcu.callbacks,
    )
    fcu.create_subscription(
        MissionPhase, 'mission_phase', lambda message: phases.append(message),
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        callback_group=fcu.callbacks,
    )
    publishers = {
        'smoke': fcu.create_publisher(SmokeDetection, 'smoke/detection', 10),
        'fire': fcu.create_publisher(FireDetection, 'fire/detection', 10),
    }
    wait_for(
        lambda: all(publisher.get_subscription_count() >= 1
                    for publisher in publishers.values())
        and monitor._request_publisher.get_subscription_count() >= 2,
        3.0, 'detector monitor and executive hook discovery',
    )
    return monitor, publishers, requests, phases


def source_time(flight, monitor):
    """Acquire camera time after both consumers and the current phase are ready."""
    def synchronized():
        executive_ns = flight.executive.get_clock().now().nanoseconds
        monitor_ns = monitor.get_clock().now().nanoseconds
        return (
            min(executive_ns, monitor_ns) > 5_000_000_000
            and abs(executive_ns - monitor_ns) < 200_000_000
            and executive_ns > max(monitor._active_goal_stamp_ns, monitor._phase_stamp_ns)
        )

    wait_for(synchronized, 3.0, 'initialized shared detector and executive ROS clocks')
    return flight.executive.get_clock().now()


def assert_aborted_in_loiter(flight, goal):
    """Recovery must finish before releasing the action's exclusive goal slot."""
    started = time.monotonic()
    result = mission_result(goal)
    assert time.monotonic() - started < 8.0
    assert result.status == GoalStatus.STATUS_ABORTED
    assert not result.result.success
    assert 'abort' in result.result.message.lower()
    assert not flight.executive._mission_lock.locked()
    assert flight.executive._state_machine is None
    with flight.fcu.lock:
        assert flight.fcu.state.mode == 'AUTO.LOITER'
        assert flight.fcu.state.armed
        assert not {'AUTO.RTL', 'AUTO.LAND'} & set(flight.fcu.mode_requests)
        assert not flight.fcu.offboard_losses
        assert not flight.fcu.target_requests


@pytest.mark.parametrize('source', ['smoke', 'fire'])
@pytest.mark.parametrize('mission', [True], indirect=True, ids=['sim_clock'])
def test_confirmed_detection_aborts_exploration_and_next_goal_requires_new_images(
    mission, source,  # noqa: F811
):
    executive, fcu = mission.executive, mission.fcu
    # This suite checks event routing/lifetimes with an extra ROS node. The
    # mission action suite independently exercises the 0.5 s stream watchdog.
    fcu.offboard_loss_timeout = 2.0
    fcu.heartbeat_interval = 0.2
    executive.config['sweep_step_duration'] = 1.0
    monitor, publishers, requests, phases = attach_monitor(mission)
    assert monitor.get_parameter('use_sim_time').value

    # A previously recorded fire must never mask an abort in the dormant mode.
    known_fire_publisher = fcu.create_publisher(
        FireSourceReport, 'fire_sources',
        QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    with fcu.lock:
        known_fire = FireSourceReport(
            site_id=1, label='Fire source found', exclusion_radius=100.0,
            bearing_rad=yaw_from_quaternion(fcu.pose.orientation),
        )
        known_fire.local_pose.header.frame_id = 'px4_odom'
        known_fire.local_pose.pose = deepcopy(fcu.pose)
    known_fire_publisher.publish(known_fire)

    # A confirmed observation while idle cannot stop a later action.
    publishers[source].publish(observation(source, source_time(mission, monitor).to_msg()))
    prepare_airborne(mission)
    goal = send_goal(mission, skip_takeoff=True)
    assert goal.accepted
    goal_id = tuple(goal.goal_id.uuid)
    wait_for(
        lambda: monitor._active_goal_id == goal_id
        and monitor._phase_goal_id == goal_id and monitor._phase_state == 'Exploration',
        3.0, 'matching executing action and Exploration phase at the monitor',
    )
    wait_for(lambda: received_state(mission, mode='OFFBOARD'), 8.0, 'Exploration OFFBOARD')
    result_future = goal.get_result_async()
    assert not result_future.done()
    assert requests == []

    # Confirmation flags and acquisition time are checked on real DDS messages.
    now = source_time(mission, monitor)
    publishers[source].publish(observation(source, now.to_msg(), confirmed=False))
    threading.Event().wait(0.05)
    for nanoseconds in (
        max(0, monitor._active_goal_stamp_ns - 1),
        now.nanoseconds - 5_000_000_000,
        now.nanoseconds + 5_000_000_000,
    ):
        publishers[source].publish(observation(source, Time(nanoseconds=nanoseconds).to_msg()))
        threading.Event().wait(0.05)
    assert not result_future.done()
    assert requests == []

    old_frame = observation(source, source_time(mission, monitor).to_msg())
    publishers[source].publish(old_frame)
    wait_for(lambda: len(requests) == 1, 2.0, 'one goal-scoped Abort request')
    assert requests[0].state == 'Abort'
    assert tuple(requests[0].goal_id.uuid) == goal_id
    assert_aborted_in_loiter(mission, goal)
    wait_for(lambda: monitor._active_goal_id is None, 3.0, 'terminal goal status at monitor')
    assert all(phase.state == 'Exploration' for phase in phases)

    # Observations while idle cannot leave an outstanding flag for the next goal.
    for other_source, publisher in publishers.items():
        publisher.publish(observation(other_source, source_time(mission, monitor).to_msg()))
    second_goal = send_goal(mission, skip_takeoff=True)
    assert second_goal.accepted
    second_id = tuple(second_goal.goal_id.uuid)
    wait_for(
        lambda: monitor._active_goal_id == second_id
        and monitor._phase_goal_id == second_id and monitor._phase_state == 'Exploration',
        3.0, 'new executing goal and Exploration phase at the monitor',
    )
    # Even requests delayed by transport from the previous goal are inert.
    delayed_request = fcu.create_publisher(
        MissionStateRequest, 'mission_state_request_for_goal', 10,
    )
    wait_for(lambda: delayed_request.get_subscription_count() >= 2,
             3.0, 'delayed request publisher discovery')
    delayed_request.publish(requests[0])
    publishers[source].publish(old_frame)
    wait_for(lambda: received_state(mission, mode='OFFBOARD'),
             8.0, 'next Exploration starts without an old detection flag')
    wait_for(lambda: len(requests) == 2, 2.0, 'delayed request delivery')
    wait_for(
        lambda: any(text == "MISSION: ignored goal state request 'Abort'"
                    for _, text in fcu.statuses),
        3.0, 'executive rejects the previous goal request',
    )
    assert not second_goal.get_result_async().done()
    assert all(tuple(request.goal_id.uuid) == goal_id for request in requests)

    # Only a fresh confirmed image for this new action can stop it.
    publishers[source].publish(observation(source, source_time(mission, monitor).to_msg()))
    wait_for(lambda: len(requests) == 3, 3.0, 'fresh image triggers the second action abort')
    assert requests[-1].state == 'Abort'
    assert tuple(requests[-1].goal_id.uuid) == second_id
    assert_aborted_in_loiter(mission, second_goal)
    assert all(phase.state == 'Exploration' for phase in phases)

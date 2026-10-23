"""Battery learning and return-budget behavior without an executor or FCU."""

import math

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from kv_executive.battery_model import BatteryUsageModel
import pytest
from std_msgs.msg import Header


def pose(x=0.0, y=0.0, z=0.0, frame='map'):
    return PoseStamped(
        header=Header(frame_id=frame),
        pose=Pose(
            position=Point(x=float(x), y=float(y), z=float(z)),
            orientation=Quaternion(w=1.0),
        ),
    )


def test_record_integrates_three_dimensional_travel_instead_of_displacement():
    model = BatteryUsageModel()
    model.begin(pose(), 80.0)
    model.update(pose(3, 4), 79.0)
    model.update(pose(3, 4, 12), 78.0)
    record = model.finish(pose(), 77.0)

    assert record['sample_id'] == 1
    assert record['distance_m'] == pytest.approx(30.0)
    assert record['consumed_percent'] == pytest.approx(3.0)
    assert record['percentage_per_meter'] == pytest.approx(0.1)
    assert record['valid_for_estimate']
    assert record['outcome'] == 'completed'
    assert not model.active
    assert set(model.records) == {1}


def test_learning_weights_multiple_samples_by_distance():
    model = BatteryUsageModel()
    model.begin(pose(), 90.0)
    model.finish(pose(10), 89.0)
    model.begin(pose(10), 89.0)
    model.finish(pose(100), 71.0)

    estimate = model.estimate(pose(100), pose(), 71.0)
    assert estimate['mean_percent_per_meter'] == pytest.approx(19 / 100)
    assert estimate['return_required_percent'] == pytest.approx(19.0)
    assert estimate['sample_count'] == 2
    assert set(model.records) == {1, 2}


def test_active_sample_supports_first_flight_without_double_counting():
    model = BatteryUsageModel()
    model.begin(pose(), 80.0)
    assert not model.estimate(pose(), pose(), 80)['estimate_available']
    model.update(pose(10), 78.0)
    during = model.estimate(pose(10), pose(), 78.0)
    assert during['sample_count'] == 1
    assert during['mean_percent_per_meter'] == pytest.approx(0.2)
    model.finish(pose(10), 78.0)
    after = model.estimate(pose(10), pose(), 78.0)
    assert after == during


def test_safety_factor_and_absolute_reserve_trigger_at_boundary():
    model = BatteryUsageModel(reserve_percent=15.0, return_safety_factor=1.5)
    model.begin(pose(), 90.0)
    model.finish(pose(10), 88.0)
    estimate = model.estimate(pose(0, 0, 50), pose(), 30.0)
    assert estimate['home_distance_m'] == pytest.approx(50.0)
    assert estimate['return_required_percent'] == pytest.approx(10.0)
    assert estimate['buffered_required_percent'] == pytest.approx(30.0)
    assert estimate['return_margin_percent'] == pytest.approx(0.0)
    assert estimate['return_alert']
    assert not model.estimate(pose(0, 0, 50), pose(), 30.01)['return_alert']


def test_no_usable_rate_is_explicit_but_reserve_alert_is_still_available():
    model = BatteryUsageModel()
    estimate = model.estimate(pose(10), pose(), 15.0)
    assert not estimate['estimate_available']
    assert math.isnan(estimate['mean_percent_per_meter'])
    assert math.isnan(estimate['return_required_percent'])
    assert estimate['buffered_required_percent'] == 15.0
    assert estimate['return_alert']
    assert estimate['sample_count'] == 0


@pytest.mark.parametrize('end, remaining', [
    (pose(), 79.0), (pose(0.5), 79.0), (pose(10), 80.0),
])
def test_stationary_short_and_zero_depletion_samples_do_not_teach(end, remaining):
    model = BatteryUsageModel()
    model.begin(pose(), 80.0)
    record = model.finish(end, remaining)
    assert not record['valid_for_estimate']
    assert record['invalid_reason']
    assert not model.estimate(end, pose(), remaining)['estimate_available']


def test_input_pose_and_returned_record_mutations_do_not_change_stored_data():
    model = BatteryUsageModel()
    start = pose()
    model.begin(start, 80.0)
    start.pose.position.x = 1000.0
    end = pose(10)
    model.update(end, 79.0)
    end.pose.position.x = -1000.0
    record = model.finish(pose(10), 79.0)
    record['start_pose'].pose.position.x = 500.0
    record['end_pose'].header.frame_id = 'altered'
    assert model.records[1]['start_pose'].pose.position.x == 0.0
    assert model.records[1]['end_pose'].pose.position.x == 10.0
    assert model.records[1]['end_pose'].header.frame_id == 'map'
    assert model.records[1]['distance_m'] == 10.0


def test_frame_jump_invalidates_sample_even_if_good_telemetry_resumes():
    model = BatteryUsageModel()
    model.begin(pose(), 80.0)
    model.update(pose(10), 79.0)
    with pytest.raises(ValueError, match='coordinate frame'):
        model.update(pose(20, frame='odom'), 78.0)
    record = model.finish(pose(20), 78.0)
    assert not record['valid_for_estimate']
    assert 'coordinate frame' in record['invalid_reason']
    assert not model.estimate(pose(20), pose(), 78.0)['estimate_available']


def test_tiny_battery_rebound_is_ignored_but_accumulated_increase_is_rejected():
    model = BatteryUsageModel()
    model.begin(pose(), 80.0)
    model.update(pose(10), 79.0)
    model.update(pose(11), 79.05)
    model.update(pose(12), 79.09)
    with pytest.raises(ValueError, match='increased'):
        model.update(pose(13), 79.11)
    record = model.finish(pose(14), 78.5)
    assert not record['valid_for_estimate']


def test_partial_walk_keeps_observed_usage_and_outcome():
    model = BatteryUsageModel()
    model.begin(pose(), 80.0)
    record = model.finish(pose(10), 79.0, outcome='canceled')
    assert record['outcome'] == 'canceled'
    assert record['valid_for_estimate']


def test_telemetry_gap_explicitly_invalidates_and_reset_retains_prior_records():
    model = BatteryUsageModel()
    model.begin(pose(), 80.0)
    model.finish(pose(10), 79.0)
    model.begin(pose(10), 79.0)
    model.update(pose(20), 78.0)
    model.invalidate_active('Battery telemetry timed out')
    assert model.estimate(pose(20), pose(), 78.0)['sample_count'] == 1
    model.reset_active()
    assert not model.active
    assert len(model.records) == 1


@pytest.mark.parametrize('battery', [-1.0, 101.0, float('nan'), float('inf'), None])
def test_invalid_battery_readings_are_rejected(battery):
    model = BatteryUsageModel()
    with pytest.raises(ValueError):
        model.begin(pose(), battery)
    model.begin(pose(), 80.0)
    with pytest.raises(ValueError):
        model.update(pose(10), battery)
    record = model.finish(pose(10), 79.0)
    assert not record['valid_for_estimate']


@pytest.mark.parametrize('bad_pose', [
    None, pose(float('nan')), pose(z=float('inf')), pose(frame=''),
    PoseStamped(
        header=Header(frame_id='map'),
        pose=Pose(orientation=Quaternion(w=0.0)),
    ),
])
def test_malformed_pose_is_rejected_and_invalidates_active_usage(bad_pose):
    model = BatteryUsageModel()
    with pytest.raises(ValueError):
        model.begin(bad_pose, 80.0)
    model.begin(pose(), 80.0)
    with pytest.raises(ValueError):
        model.update(bad_pose, 79.0)
    assert not model.finish(pose(10), 79.0)['valid_for_estimate']


def test_estimation_rejects_different_home_frame_and_invalid_battery():
    model = BatteryUsageModel()
    with pytest.raises(ValueError, match='one frame'):
        model.estimate(pose(), pose(frame='odom'), 80.0)
    with pytest.raises(ValueError):
        model.estimate(pose(), pose(), float('nan'))


@pytest.mark.parametrize('parameters', [
    {'minimum_distance': 0.0}, {'minimum_distance': float('inf')},
    {'reserve_percent': -1.0}, {'reserve_percent': 101.0},
    {'return_safety_factor': 0.99}, {'return_safety_factor': float('nan')},
])
def test_invalid_learning_parameters_are_rejected(parameters):
    with pytest.raises(ValueError):
        BatteryUsageModel(**parameters)


def test_lifecycle_prevents_overwriting_an_active_walk():
    model = BatteryUsageModel()
    with pytest.raises(RuntimeError):
        model.update(pose(), 80.0)
    with pytest.raises(RuntimeError):
        model.finish(pose(), 80.0)
    model.begin(pose(), 80.0)
    with pytest.raises(RuntimeError):
        model.begin(pose(), 80.0)

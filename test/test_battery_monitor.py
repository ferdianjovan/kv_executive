"""Battery monitor lifecycle and ROS publications with deterministic telemetry."""

from copy import deepcopy
from types import MethodType, SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, Quaternion
import kv_executive.battery_monitor as monitor_module
from kv_executive.battery_monitor import BatteryMonitor, DEFAULTS
from mavros_msgs.msg import HomePosition, State
from nav_msgs.msg import Odometry
import pytest
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header, String


class Clock:
    def __init__(self):
        self.seconds = 20.0

    def monotonic(self):
        return self.seconds

    def now(self):
        return self

    def to_msg(self):
        return Time(sec=int(self.seconds))


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(deepcopy(message))


class Harness:
    """Bind real callbacks/helpers without an executor, FCU, or ROS node init."""

    def __init__(self, clock):
        self.config = deepcopy(DEFAULTS)
        self.clock = clock
        self.logs = {'info': [], 'warning': [], 'error': [], 'debug': []}
        self._usage_publisher = Publisher()
        self._estimate_publisher = Publisher()
        self._initialize_tracking()

    def __getattr__(self, name):
        return MethodType(getattr(BatteryMonitor, name), self)

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return SimpleNamespace(**{
            level: lambda message, *args, _level=level, **kwargs:
                self.logs[_level].append(message)
            for level in self.logs
        })

    def status(self, message):
        self._status_callback(String(data=message))

    def battery(self, percentage, **kwargs):
        self._battery_callback(BatteryState(
            percentage=float(percentage), present=True,
            power_supply_status=BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
            **kwargs,
        ))

    def odometry(self, x=0.0, y=0.0, z=0.0, frame='px4_odom'):
        message = Odometry(header=Header(frame_id=frame))
        message.header.stamp = self.clock.to_msg()
        message.pose.pose = Pose(
            position=Point(x=float(x), y=float(y), z=float(z)),
            orientation=Quaternion(w=1.0),
        )
        self._odometry_callback(message)

    def state(self, *, armed=True, connected=True):
        self._state_callback(State(armed=armed, connected=connected))

    def takeoff(self):
        self.state(armed=False)
        self.odometry()
        self.battery(0.95)
        self.state()
        self.odometry(z=5)
        self.battery(0.90)
        self.status('TAKEOFF: done')

    def start_walk(self, percentage=0.80):
        self.status('SWEEP: started')
        self.battery(percentage)
        self.status('SWEEP: done')
        self.status('EXECUTE_PATH: started')


@pytest.fixture
def node(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(
        monitor_module, 'time', SimpleNamespace(monotonic=clock.monotonic),
    )
    return Harness(clock)


def test_only_third_state_records_usage_and_publishes_return_estimates(node):
    node.takeoff()
    node._tick()
    node.status('SWEEP: started')
    node.battery(0.80)
    node._tick()

    assert node.records == {}
    assert node._usage_publisher.messages == []
    assert node._estimate_publisher.messages == []

    node.status('SWEEP: done')
    node.status('EXECUTE_PATH: started')
    node.odometry(3, 4, 5)
    node.battery(0.79)
    node.odometry(0, 0, 5)
    node.battery(0.78)
    node._tick()
    node.status('EXECUTE_PATH: done')

    assert len(node.records) == 1
    sample = node.records[1]
    assert sample['start_battery_percent'] == pytest.approx(80.0)
    assert sample['end_battery_percent'] == pytest.approx(78.0)
    assert sample['consumed_percent'] == pytest.approx(2.0)
    assert sample['distance_m'] == pytest.approx(10.0)
    assert sample['valid_for_estimate']
    assert sample['start_pose'].pose.position == sample['end_pose'].pose.position
    assert sample['start_pose'].pose.position.z == 5.0

    assert len(node._usage_publisher.messages) == 1
    usage = node._usage_publisher.messages[0]
    assert usage.sample_id == 1
    assert usage.start_pose == sample['start_pose']
    assert usage.end_pose == sample['end_pose']
    assert usage.distance_m == pytest.approx(10.0)
    assert usage.consumed_percent == pytest.approx(2.0)
    assert usage.valid_for_estimate
    assert node._estimate_publisher.messages[-1].estimate_available

    published = len(node._estimate_publisher.messages)
    node.odometry(20, 0, 5)
    node.battery(0.77)
    node._tick()
    assert len(node._estimate_publisher.messages) == published
    assert len(node.records) == 1


@pytest.mark.parametrize('terminal, outcome', [
    ('MISSION: canceled', 'canceled'),
    ('MISSION: aborted', 'aborted'),
])
def test_canceled_or_aborted_walk_keeps_observed_partial_sample(node, terminal, outcome):
    node.takeoff()
    node.start_walk()
    node.odometry(10, 0, 5)
    node.battery(0.79)
    node.status(terminal)

    assert len(node.records) == 1
    sample = node.records[1]
    assert sample['outcome'] == outcome
    assert sample['distance_m'] == pytest.approx(10.0)
    assert sample['consumed_percent'] == pytest.approx(1.0)
    assert sample['valid_for_estimate']
    assert node._usage_publisher.messages[0].outcome == outcome
    published = len(node._estimate_publisher.messages)
    node._tick()
    assert len(node._estimate_publisher.messages) == published


def test_home_is_last_disarmed_pose_and_does_not_follow_airborne_telemetry(node):
    node.state(armed=False)
    node.odometry(1, 2, 3)
    node.odometry(2, 2, 3)
    node.battery(0.95)
    node.state()
    node.odometry(2, 2, 8)
    node.battery(0.90)
    node.status('TAKEOFF: done')
    node.start_walk()
    node._home_callback(HomePosition(
        position=Point(x=100.0, y=100.0, z=100.0),
        orientation=Quaternion(w=1.0),
    ))
    node.odometry(5, 6, 8)
    node.battery(0.79)
    node._tick()

    estimate = node._estimate_publisher.messages[-1]
    assert estimate.home_pose.header.frame_id == 'px4_odom'
    assert estimate.home_pose.pose.position == Point(x=2.0, y=2.0, z=3.0)
    assert estimate.home_distance_m == pytest.approx(50 ** 0.5)


def test_starting_airborne_uses_mavros_home_in_local_frame(node):
    node.state()
    node._home_callback(HomePosition(
        position=Point(x=2.0, y=3.0, z=4.0), orientation=Quaternion(w=1.0),
    ))
    node.odometry(2, 3, 9)
    node.battery(0.80)
    node.status('SWEEP: started')
    node.status('SWEEP: done')
    node.status('EXECUTE_PATH: started')
    node.odometry(5, 7, 9)
    node.battery(0.79)
    node._tick()

    estimate = node._estimate_publisher.messages[-1]
    assert estimate.home_pose.pose.position == Point(x=2.0, y=3.0, z=4.0)
    assert estimate.home_pose.header.frame_id == 'px4_odom'
    assert estimate.remaining_percent == pytest.approx(79.0)
    assert estimate.estimate_available


@pytest.mark.parametrize('percentage', [-1.0, -0.01, 1.01, float('nan'), float('inf')])
def test_invalid_battery_fraction_invalidates_active_sample(node, percentage):
    node.takeoff()
    node.start_walk()
    node.odometry(10, 0, 5)
    node.battery(percentage)
    node.battery(0.79)
    node.status('EXECUTE_PATH: done')

    assert len(node.records) == 1
    assert not node.records[1]['valid_for_estimate']
    assert not node._usage_publisher.messages[0].valid_for_estimate


@pytest.mark.parametrize('battery_update', [
    {'present': False},
    {'power_supply_status': BatteryState.POWER_SUPPLY_STATUS_CHARGING},
])
def test_absent_or_charging_battery_cannot_teach_depletion(node, battery_update):
    node.takeoff()
    node.start_walk()
    node.odometry(10, 0, 5)
    values = {
        'percentage': 0.79,
        'present': True,
        'power_supply_status': BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
    }
    values.update(battery_update)
    node._battery_callback(BatteryState(**values))
    node.battery(0.79)
    node.status('EXECUTE_PATH: done')

    assert not node.records[1]['valid_for_estimate']


@pytest.mark.parametrize('stale', ['battery', 'odometry', 'state'])
def test_stale_telemetry_suppresses_estimates_and_alerts_and_invalidates_sample(node, stale):
    node.takeoff()
    node.start_walk(0.20)
    node.odometry(10, 0, 5)
    node.battery(0.19)
    node._tick()
    node.logs['info'].clear()
    published = len(node._estimate_publisher.messages)
    node.clock.seconds += 11.0
    if stale != 'state':
        node.state()
    if stale != 'odometry':
        node.odometry(10, 0, 5)
    if stale != 'battery':
        node.battery(0.18)
    node._tick()

    assert len(node._estimate_publisher.messages) == published
    assert not any('ALERT' in message for message in node.logs['info'])
    node.state()
    node.odometry(10, 0, 5)
    node.battery(0.18)
    node.status('EXECUTE_PATH: done')
    assert not node.records[1]['valid_for_estimate']


def test_return_alert_uses_historical_and_current_usage_with_buffer_and_throttle(node):
    node.takeoff()
    node.start_walk(0.50)
    node.odometry(10, 0, 5)
    node.battery(0.49)
    node.status('EXECUTE_PATH: done')
    node.start_walk(0.20)
    node.odometry(20, 0, 5)
    node.battery(0.18)
    node._tick()

    estimate = node._estimate_publisher.messages[-1]
    assert estimate.sample_count == 2
    assert estimate.mean_percent_per_meter == pytest.approx(3.0 / 20.0)
    assert estimate.return_required_percent == pytest.approx((425 ** 0.5) * 0.15)
    assert estimate.buffered_required_percent == pytest.approx(
        estimate.return_required_percent * 1.3 + 15.0,
    )
    assert estimate.return_alert

    def alerts():
        return [message for message in node.logs['info'] if 'ALERT' in message]

    assert alerts()
    count = len(alerts())
    node._tick()
    node._tick()
    assert len(alerts()) == count
    for _ in range(int(node.config['alert_repeat_period']) + 1):
        node.clock.seconds += 1.0
        node.state()
        node.odometry(20, 0, 5)
        node.battery(0.18)
    node._tick()
    assert len(alerts()) == count + 1


def test_no_fresh_start_baseline_skips_learning_for_whole_interval(node):
    node.takeoff()
    node.clock.seconds += node.config['battery_timeout'] + 0.1
    node.state()
    node.odometry(z=5)
    node.status('EXECUTE_PATH: started')
    node.battery(0.80)
    node.odometry(10, 0, 5)
    node.battery(0.79)
    node._tick()
    node.status('EXECUTE_PATH: done')

    assert node.records == {}
    assert node._usage_publisher.messages == []
    assert not node._estimate_publisher.messages[-1].estimate_available


def test_duplicate_start_and_intermediate_subpath_events_keep_one_full_interval(node):
    node.takeoff()
    node.start_walk()
    node.odometry(10, 0, 5)
    node.battery(0.79)
    node.status('EXECUTE_PATH: sub-path 1/3 done')
    node.status('EXECUTE_PATH: started')
    assert node.records == {}
    node.odometry(20, 0, 5)
    node.battery(0.78)
    node.status('EXECUTE_PATH: done')
    node.status('EXECUTE_PATH: done')

    assert len(node.records) == 1
    assert node.records[1]['distance_m'] == pytest.approx(20.0)
    assert node.records[1]['consumed_percent'] == pytest.approx(2.0)
    assert len(node._usage_publisher.messages) == 1


def test_unknown_home_allows_usage_record_but_suppresses_return_estimate(node):
    node.state()
    node.odometry(z=5)
    node.battery(0.80)
    node.status('EXECUTE_PATH: started')
    node.odometry(10, 0, 5)
    node.battery(0.79)
    node._tick()
    node.status('EXECUTE_PATH: done')

    assert node.records[1]['valid_for_estimate']
    assert len(node._usage_publisher.messages) == 1
    assert node._estimate_publisher.messages == []


@pytest.mark.parametrize('frame, x', [
    ('map', 10.0), ('', 10.0), ('px4_odom', float('nan')),
])
def test_invalid_or_mismatched_odometry_invalidates_active_sample(node, frame, x):
    node.takeoff()
    node.start_walk()
    node.odometry(10, 0, 5)
    node.battery(0.79)
    node.odometry(x, 0, 5, frame=frame)
    node.odometry(20, 0, 5)
    node.battery(0.78)
    node.status('EXECUTE_PATH: done')

    assert not node.records[1]['valid_for_estimate']
    assert node.records[1]['invalid_reason']


def test_disconnected_state_suppresses_return_alert_and_invalidates_interval(node):
    node.takeoff()
    node.start_walk(0.20)
    node.odometry(10, 0, 5)
    node.battery(0.10)
    node.state(connected=False)
    node._tick()

    assert node._estimate_publisher.messages == []
    assert not any('ALERT' in message for message in node.logs['info'])
    node.state()
    node.status('EXECUTE_PATH: done')
    assert not node.records[1]['valid_for_estimate']


def test_learning_status_does_not_repeat_on_every_monitor_tick(node):
    node.takeoff()
    node.start_walk()
    node.logs['info'].clear()
    node._tick()
    node._tick()
    node._tick()

    learning = [message for message in node.logs['info'] if 'Learning' in message]
    assert len(learning) == 1


def test_takeoff_start_freezes_home_before_slow_armed_state_update(node):
    node.state(armed=False)
    node.odometry(1, 2, 3)
    node.battery(0.95)
    node.status('TAKEOFF: started')
    # Odometry can report climbing before the next low-rate /mavros/state.
    node.odometry(1, 2, 4)
    node.state()
    node.odometry(1, 2, 8)
    node.battery(0.90)
    node.status('TAKEOFF: done')
    node.start_walk()
    node.odometry(4, 6, 8)
    node.battery(0.79)
    node._tick()

    assert node._estimate_publisher.messages[-1].home_pose.pose.position == Point(
        x=1.0, y=2.0, z=3.0,
    )


def test_starting_airborne_freezes_first_valid_mavros_home(node):
    node.state()
    node._home_callback(HomePosition(position=Point(x=1.0, y=2.0, z=3.0)))
    node.odometry(1, 2, 8)
    node.battery(0.80)
    node.status('EXECUTE_PATH: started')
    node._home_callback(HomePosition(position=Point(x=100.0, y=200.0, z=300.0)))
    node.odometry(4, 6, 8)
    node.battery(0.79)
    node._tick()

    assert node._estimate_publisher.messages[-1].home_pose.pose.position == Point(
        x=1.0, y=2.0, z=3.0,
    )


@pytest.mark.parametrize('terminal', [
    'MISSION: canceled', 'MISSION: aborted: Arming failed',
])
def test_prearm_mission_failure_unlocks_home_for_the_next_takeoff(node, terminal):
    node.state(armed=False)
    node.odometry(1, 2, 0)
    node.battery(0.95)
    node.status('TAKEOFF: started')
    node.odometry(10, 20, 0)
    node.status(terminal)
    # The vehicle can be moved on the ground after a failed arming attempt.
    node.odometry(30, 40, 0)
    node.status('TAKEOFF: started')
    node.state()
    node.odometry(30, 40, 5)
    node.battery(0.90)
    node.status('TAKEOFF: done')
    node.start_walk()
    node.odometry(33, 44, 5)
    node.battery(0.79)
    node._tick()

    assert node._estimate_publisher.messages[-1].home_pose.pose.position == Point(
        x=30.0, y=40.0, z=0.0,
    )


@pytest.mark.parametrize('gap_source, seconds', [
    ('battery', 6), ('odometry', 3), ('state', 11),
])
def test_telemetry_gap_freezes_record_at_last_trustworthy_endpoint(node, gap_source, seconds):
    node.takeoff()
    node.start_walk()
    node.odometry(10, 0, 5)
    node.battery(0.79)
    for _ in range(seconds):
        node.clock.seconds += 1.0
        if gap_source != 'state':
            node.state()
        if gap_source != 'odometry':
            node.odometry(10, 0, 5)
        if gap_source != 'battery':
            node.battery(0.79)
        node._tick()

    # Even after every source recovers, the missing flight cannot be measured.
    node.state()
    node.odometry(20, 0, 5)
    node.battery(0.77)
    node.odometry(30, 0, 5)
    node.battery(0.75)
    node.status('EXECUTE_PATH: done')

    sample = node.records[1]
    assert not sample['valid_for_estimate']
    assert sample['distance_m'] == pytest.approx(10.0)
    assert sample['end_pose'].pose.position == Point(x=10.0, y=0.0, z=5.0)
    assert sample['end_battery_percent'] == pytest.approx(79.0)
    assert sample['consumed_percent'] == pytest.approx(1.0)
    usage = node._usage_publisher.messages[0]
    assert not usage.valid_for_estimate
    assert usage.distance_m == pytest.approx(10.0)
    assert usage.end_pose == sample['end_pose']
    assert usage.consumed_percent == pytest.approx(1.0)

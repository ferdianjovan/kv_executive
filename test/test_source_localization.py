"""Metric source location requires translated, associated camera observations."""

import math

from kv_executive.source_localization import ThermalBearingLocalizer
import pytest


def observe(localizer, origin, point=(20.0, 0.0, 2.0), stamp=100.0,
            track=7, count=1, camera='thermal'):
    direction = tuple(end - start for end, start in zip(point, origin))
    return localizer.add(stamp=stamp, camera=camera, origin=origin,
                         direction=direction, track_id=track, source_count=count)


def test_three_translated_views_localize_source_with_angular_uncertainty():
    localizer = ThermalBearingLocalizer()
    assert observe(localizer, (0.0, 0.0, 2.0))['point'] is None
    assert observe(localizer, (0.0, 5.0, 2.0), stamp=105.0)['point'] is None
    result = observe(localizer, (0.0, -5.0, 2.0), stamp=110.0)
    assert result['point'] == pytest.approx((20.0, 0.0, 2.0))
    assert 0.5 < result['uncertainty'] < 2.0
    assert result['association'] == 'stable_thermal_track'
    assert result['current_stamp'] == 110.0
    assert result['views'] == 3 and result['baseline'] == 10.0


@pytest.mark.parametrize('positions', [
    [(0.0, 0.0, 2.0)] * 3,
    [(0.0, 0.0, 2.0), (0.0, 0.5, 2.0), (0.0, 1.0, 2.0)],
    [(0.0, 0.0, 2.0), (0.5, 0.0, 2.0), (3.0, 0.0, 2.0)],
])
def test_hold_small_baseline_and_forward_motion_do_not_invent_range(positions):
    localizer = ThermalBearingLocalizer()
    for index, origin in enumerate(positions):
        result = observe(localizer, origin, point=(40.0, 0.0, 2.0), stamp=100 + index)
    assert result['point'] is None


def test_low_parallax_or_long_range_is_not_reported_with_voxel_precision():
    localizer = ThermalBearingLocalizer()
    for index, y in enumerate((0.0, 5.0, -5.0)):
        result = observe(localizer, (0.0, y, 2.0), point=(100.0, 0.0, 2.0), stamp=100 + index)
    assert result['point'] == pytest.approx((100.0, 0.0, 2.0))
    assert result['uncertainty'] > 2.0


def test_changed_ids_require_single_source_geometric_consistency():
    localizer = ThermalBearingLocalizer()
    for index, y in enumerate((0.0, 5.0, -5.0)):
        result = observe(localizer, (0.0, y, 2.0), stamp=100 + index, track=10 + index)
    assert result['point'] == pytest.approx((20.0, 0.0, 2.0))
    assert result['association'] == 'single_source_geometric_match'


def test_changed_ids_in_multi_source_images_are_never_blindly_paired():
    localizer = ThermalBearingLocalizer()
    for index, y in enumerate((0.0, 5.0, -5.0)):
        result = observe(localizer, (0.0, y, 2.0), stamp=100 + index,
                         track=10 + index, count=2)
    assert result['point'] is None


def test_stable_id_in_multi_source_images_remains_localizable():
    localizer = ThermalBearingLocalizer()
    for index, y in enumerate((0.0, 5.0, -5.0)):
        result = observe(localizer, (0.0, y, 2.0), stamp=100 + index, count=2)
    assert result['point'] == pytest.approx((20.0, 0.0, 2.0))


def test_current_inconsistent_ray_invalidates_previously_good_fit():
    localizer = ThermalBearingLocalizer()
    for index, y in enumerate((0.0, 5.0, -5.0)):
        result = observe(localizer, (0.0, y, 2.0), stamp=100 + index)
    assert result['point'] is not None
    result = observe(localizer, (0.0, 10.0, 2.0), point=(20.0, 15.0, 2.0), stamp=103)
    assert result['point'] is None
    assert result['reason'] == 'inconsistent_reprojection'


def test_stationary_updates_keep_prior_translated_views_bounded():
    localizer = ThermalBearingLocalizer(max_observations=8)
    observe(localizer, (0.0, 0.0, 2.0), stamp=100)
    observe(localizer, (0.0, 5.0, 2.0), stamp=101)
    for index in range(40):
        result = observe(localizer, (0.0, -5.0, 2.0), stamp=102 + index * 0.1)
    assert result['point'] is not None
    assert len(localizer._observations) == 3


def test_stale_views_and_other_cameras_cannot_supply_missing_baseline():
    localizer = ThermalBearingLocalizer(max_age=5.0)
    observe(localizer, (0.0, 0.0, 2.0), stamp=100)
    observe(localizer, (0.0, 5.0, 2.0), stamp=101, camera='other_thermal')
    result = observe(localizer, (0.0, -5.0, 2.0), stamp=110)
    assert result['point'] is None


@pytest.mark.parametrize('bad', [0.0, math.nan, math.inf])
def test_invalid_source_stamp_never_produces_a_location(bad):
    result = observe(ThermalBearingLocalizer(), (0.0, 0.0, 0.0), stamp=bad)
    assert result['point'] is None


def test_sources_beyond_configured_range_are_rejected():
    localizer = ThermalBearingLocalizer()
    for index, y in enumerate((0.0, 20.0, -20.0)):
        result = observe(localizer, (0.0, y, 2.0), point=(250.0, 0.0, 2.0), stamp=100 + index)
    assert result['point'] is None and result['reason'] == 'invalid_forward_range'

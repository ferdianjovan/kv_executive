"""Bounded, source-stamped triangulation of confirmed thermal camera bearings.

An occupied map voxel is never a semantic source measurement. This estimator
uses translated views of one image-plane track and includes the current ray in
every returned fit. It deliberately leaves low-parallax observations unknown.
"""

from dataclasses import dataclass
import math


def _dot(left, right):
    return sum(a * b for a, b in zip(left, right))


def _subtract(left, right):
    return tuple(a - b for a, b in zip(left, right))


def _solve(matrix, vector):
    rows = [list(row) + [value] for row, value in zip(matrix, vector)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda index: abs(rows[index][column]))
        if abs(rows[pivot][column]) < 1e-12:
            return None
        rows[column], rows[pivot] = rows[pivot], rows[column]
        divisor = rows[column][column]
        rows[column] = [value / divisor for value in rows[column]]
        for index in range(3):
            if index != column:
                scale = rows[index][column]
                rows[index] = [a - scale * b
                               for a, b in zip(rows[index], rows[column])]
    return tuple(row[3] for row in rows)


@dataclass(frozen=True)
class BearingObservation:
    stamp: float
    camera: str
    origin: tuple[float, float, float]
    direction: tuple[float, float, float]
    track_id: int
    source_count: int


class ThermalBearingLocalizer:
    """Fit recent translated views, preferring stable detector track identity."""

    def __init__(self, *, max_age=90.0, max_observations=32,
                 minimum_baseline=2.0, minimum_parallax=math.radians(2.0),
                 angular_noise=math.radians(0.5), maximum_range=200.0):
        self.max_age = float(max_age)
        self.max_observations = int(max_observations)
        self.minimum_baseline = float(minimum_baseline)
        self.minimum_parallax = float(minimum_parallax)
        self.angular_noise = float(angular_noise)
        self.maximum_range = float(maximum_range)
        self._observations = []

    def add(self, *, stamp, camera, origin, direction, track_id, source_count):
        values = (*origin, *direction, stamp)
        if (not camera or not all(math.isfinite(value) for value in values)
                or stamp <= 0.0 or source_count < 1 or track_id < 0):
            return {'point': None, 'reason': 'invalid_bearing'}
        norm = math.hypot(*direction)
        if norm < 1e-9:
            return {'point': None, 'reason': 'invalid_bearing'}
        current = BearingObservation(
            float(stamp), str(camera), tuple(origin), tuple(value / norm for value in direction),
            int(track_id), int(source_count))
        recent = [ray for ray in self._observations
                  if 0.0 <= current.stamp - ray.stamp <= self.max_age]
        # Repeated calls for one source image cannot manufacture extra views.
        recent = [ray for ray in recent
                  if not (ray.camera == current.camera and ray.stamp == current.stamp)]
        recent = [ray for ray in recent
                  if not (ray.camera == current.camera and ray.track_id == current.track_id
                          and math.dist(ray.origin, current.origin) < 0.5)]
        recent.append(current)
        self._observations = recent[-self.max_observations:]
        camera_rays = [ray for ray in self._observations if ray.camera == current.camera]
        matching = [ray for ray in camera_rays
                    if current.track_id > 0 and ray.track_id == current.track_id]
        result = self._fit(matching, current)
        if result.get('point') is not None:
            return {**result, 'association': 'stable_thermal_track'}
        # A detector may allocate a new ID after a miss. Cross-ID association is
        # allowed only for single-source frames and a fit passing all geometric
        # checks; multi-blob images never become blind cross-track pairings.
        if current.source_count == 1:
            singles = [ray for ray in camera_rays if ray.source_count == 1]
            alternative = self._fit(singles, current)
            if alternative.get('point') is not None:
                return {**alternative, 'association': 'single_source_geometric_match'}
            return alternative
        return result

    def _fit(self, rays, current):
        unknown = {'point': None, 'reason': 'insufficient_separated_views'}
        # Prefer the current ray and translated observations. Holding at one
        # position (or yawing in place) cannot improve metric range confidence.
        selected = [current] if current in rays else []
        for ray in reversed(rays):
            if ray == current:
                continue
            if all(math.dist(ray.origin, previous.origin) >= 0.5 for previous in selected):
                selected.append(ray)
        if len(selected) < 3:
            return unknown
        baseline = max(math.dist(left.origin, right.origin)
                       for left in selected for right in selected)
        parallax = max(math.acos(max(-1.0, min(1.0, _dot(left.direction, right.direction))))
                       for left in selected for right in selected)
        if baseline < self.minimum_baseline or parallax < self.minimum_parallax:
            return {'point': None, 'reason': 'insufficient_baseline_or_parallax'}
        matrix = [[0.0] * 3 for _ in range(3)]
        vector = [0.0] * 3
        for ray in selected:
            for row in range(3):
                for column in range(3):
                    projector = float(row == column) - ray.direction[row] * ray.direction[column]
                    matrix[row][column] += projector
                    vector[row] += projector * ray.origin[column]
        point = _solve(matrix, vector)
        if point is None or not all(math.isfinite(value) for value in point):
            return {'point': None, 'reason': 'degenerate_bearings'}
        ranges, residuals = [], []
        for ray in selected:
            offset = _subtract(point, ray.origin)
            distance = _dot(offset, ray.direction)
            residual = tuple(value - distance * direction
                             for value, direction in zip(offset, ray.direction))
            perpendicular = math.hypot(*residual)
            if not 0.0 < distance <= self.maximum_range:
                return {'point': None, 'reason': 'invalid_forward_range'}
            if math.atan2(perpendicular, distance) > 3.0 * self.angular_noise:
                return {'point': None, 'reason': 'inconsistent_reprojection'}
            ranges.append(distance)
            residuals.append(perpendicular)
        # Linearized ray covariance: angular noise becomes transverse spatial
        # error at each fitted range. Report a conservative two-sigma major-axis
        # bound plus the largest measured residual, rather than voxel precision.
        information = [[0.0] * 3 for _ in range(3)]
        for ray, distance in zip(selected, ranges):
            weight = 1.0 / max((distance * self.angular_noise) ** 2, 1e-12)
            for row in range(3):
                for column in range(3):
                    information[row][column] += weight * (
                        float(row == column) - ray.direction[row] * ray.direction[column])
        columns = [_solve(information, [float(index == column) for index in range(3)])
                   for column in range(3)]
        if any(column is None for column in columns):
            return {'point': None, 'reason': 'degenerate_uncertainty'}
        covariance = [[columns[column][row] for column in range(3)] for row in range(3)]
        # The Frobenius norm bounds the largest eigenvalue of this covariance.
        # Unlike a power iteration this remains conservative for every geometry.
        largest_bound = math.sqrt(sum(value * value for row in covariance for value in row))
        uncertainty = 2.0 * math.sqrt(largest_bound) + max(residuals)
        return {
            'point': point, 'range': ranges[0], 'uncertainty': uncertainty,
            'baseline': baseline, 'parallax': parallax, 'views': len(selected),
            'residual': max(residuals), 'reason': 'triangulated',
            'current_stamp': current.stamp,
        }

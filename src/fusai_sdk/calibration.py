"""Pi-side calibration helpers.

Protocol v1.4 deliberately has no calibration commands and no Flash
persistence.  The Pi owns observation and estimation; the STM32 owns the
real-time actuator loop, limits, and safety.  This module therefore contains
only reusable host-side pieces:

* cross_targets: a small waypoint pattern for an initial gimbal excitation;
* CalibrationSample and JacobianEstimator: a pure-Python least-squares fit of
  observation deltas against joint deltas.

Sampling, vision feature extraction, validation thresholds, versioning, and
persistence remain application responsibilities.  No helper in this module
sends a command or writes firmware storage.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from .exceptions import CalibrationError


@dataclass(frozen=True)
class CalibrationSample:
    """One paired joint/observation measurement."""

    joint: Tuple[float, ...]
    observation: Tuple[float, ...]
    timestamp: float = field(default_factory=time.time)


@dataclass(frozen=True)
class JacobianResult:
    """Least-squares Jacobian and its fit diagnostics.

    matrix has one row per observation component and one column per joint
    component: observation_delta ~= matrix * joint_delta.
    """

    matrix: Tuple[Tuple[float, ...], ...]
    residual_rms: float
    condition_number: float
    sample_count: int


def cross_targets(
    center_pan: int,
    center_tilt: int,
    pan_amplitude: int,
    tilt_amplitude: int,
) -> List[Tuple[int, int]]:
    """Return a nine-point cross pattern around a gimbal center.

    The firmware limits the number of streamed waypoints to ten.  This helper
    only builds coordinates; it does not enforce a particular build's limits,
    because those live in the STM32 configuration.  The caller must choose a
    center and amplitudes that are safe for the actual mechanism.
    """

    if pan_amplitude < 0 or tilt_amplitude < 0:
        raise ValueError("cross amplitudes must be non-negative")
    center = (int(center_pan), int(center_tilt))
    return [
        center,
        (center[0] + int(pan_amplitude), center[1]),
        center,
        (center[0] - int(pan_amplitude), center[1]),
        center,
        (center[0], center[1] + int(tilt_amplitude)),
        center,
        (center[0], center[1] - int(tilt_amplitude)),
        center,
    ]


def cross_waypoints(
    center_pan: int,
    center_tilt: int,
    pan_amplitude: int,
    tilt_amplitude: int,
) -> List[Tuple[int, int]]:
    """Alias for cross_targets."""

    return cross_targets(center_pan, center_tilt, pan_amplitude, tilt_amplitude)


def _transpose(matrix: Sequence[Sequence[float]]) -> List[List[float]]:
    if not matrix:
        return []
    rows = len(matrix)
    cols = len(matrix[0])
    return [[float(matrix[r][c]) for r in range(rows)] for c in range(cols)]


def _matmul(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> List[List[float]]:
    if not a or not b:
        return []
    n = len(a)
    m = len(b)
    p = len(b[0])
    out = [[0.0] * p for _ in range(n)]
    for i in range(n):
        for k in range(m):
            aik = a[i][k]
            if aik == 0.0:
                continue
            for j in range(p):
                out[i][j] += aik * b[k][j]
    return out


def _invert(matrix: Sequence[Sequence[float]]) -> List[List[float]]:
    n = len(matrix)
    if n == 0 or any(len(row) != n for row in matrix):
        raise CalibrationError("matrix must be square")
    augmented = [
        [float(matrix[i][j]) for j in range(n)] + [1.0 if i == j else 0.0 for j in range(n)]
        for i in range(n)
    ]

    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(augmented[row][col]))
        if abs(augmented[pivot][col]) < 1e-12:
            raise CalibrationError("sample geometry is singular")
        if pivot != col:
            augmented[col], augmented[pivot] = augmented[pivot], augmented[col]

        divisor = augmented[col][col]
        for j in range(2 * n):
            augmented[col][j] /= divisor

        for row in range(n):
            if row == col:
                continue
            factor = augmented[row][col]
            if factor == 0.0:
                continue
            for j in range(2 * n):
                augmented[row][j] -= factor * augmented[col][j]

    return [row[n:] for row in augmented]


def _frobenius(matrix: Sequence[Sequence[float]]) -> float:
    return math.sqrt(sum(value * value for row in matrix for value in row))


class JacobianEstimator:
    """Collect paired samples and solve a centered least-squares Jacobian.

    This is a deliberately small numerical helper, not a full system
    identification package.  For a two-axis gimbal, collect at least three
    well-spread samples (more is better), reject singular geometry, and check
    residual RMS before replacing an existing Jacobian.
    """

    def __init__(
        self,
        joint_dim: Optional[int] = None,
        observation_dim: Optional[int] = None,
    ) -> None:
        self._joint_dim = joint_dim
        self._observation_dim = observation_dim
        self._samples: List[CalibrationSample] = []

    def __len__(self) -> int:
        return len(self._samples)

    @property
    def samples(self) -> Tuple[CalibrationSample, ...]:
        return tuple(self._samples)

    def clear(self) -> None:
        self._samples.clear()

    def _check_dimensions(self, joint: Sequence[float], observation: Sequence[float]) -> None:
        if not joint:
            raise ValueError("joint vector must not be empty")
        if not observation:
            raise ValueError("observation vector must not be empty")
        if self._joint_dim is None:
            self._joint_dim = len(joint)
        elif len(joint) != self._joint_dim:
            raise ValueError("joint dimension changed from %d to %d" % (self._joint_dim, len(joint)))
        if self._observation_dim is None:
            self._observation_dim = len(observation)
        elif len(observation) != self._observation_dim:
            raise ValueError(
                "observation dimension changed from %d to %d"
                % (self._observation_dim, len(observation))
            )

    def add(self, joint: Sequence[float], observation: Sequence[float]) -> CalibrationSample:
        joint_tuple = tuple(float(value) for value in joint)
        observation_tuple = tuple(float(value) for value in observation)
        self._check_dimensions(joint_tuple, observation_tuple)
        sample = CalibrationSample(joint=joint_tuple, observation=observation_tuple)
        self._samples.append(sample)
        return sample

    def add_sample(self, sample: CalibrationSample) -> None:
        self._check_dimensions(sample.joint, sample.observation)
        self._samples.append(sample)

    def solve(self) -> JacobianResult:
        if self._joint_dim is None or self._observation_dim is None:
            raise CalibrationError("no samples")
        n = len(self._samples)
        m = self._joint_dim
        p = self._observation_dim
        if n < m + 1:
            raise CalibrationError("need at least %d samples for %d joints" % (m + 1, m))

        joint_mean = [
            sum(sample.joint[j] for sample in self._samples) / n for j in range(m)
        ]
        observation_mean = [
            sum(sample.observation[k] for sample in self._samples) / n for k in range(p)
        ]
        x = [
            [sample.joint[j] - joint_mean[j] for j in range(m)]
            for sample in self._samples
        ]
        y = [
            [sample.observation[k] - observation_mean[k] for k in range(p)]
            for sample in self._samples
        ]

        xt = _transpose(x)
        xtx = _matmul(xt, x)
        xty = _matmul(xt, y)
        try:
            inverse = _invert(xtx)
        except CalibrationError as exc:
            raise CalibrationError("joint excitation does not span the joint space") from exc

        jacobian = _matmul(inverse, xty)  # m x p
        predicted = _matmul(x, jacobian)  # n x p
        error_sum = 0.0
        for i in range(n):
            for k in range(p):
                delta = y[i][k] - predicted[i][k]
                error_sum += delta * delta
        residual_rms = math.sqrt(error_sum / float(n * p))
        condition = _frobenius(xtx) * _frobenius(inverse)

        matrix = tuple(
            tuple(jacobian[j][k] for j in range(m)) for k in range(p)
        )
        return JacobianResult(
            matrix=matrix,
            residual_rms=residual_rms,
            condition_number=condition,
            sample_count=n,
        )

import math

import pytest

from fusai_sdk.calibration import (
    CalibrationSample,
    JacobianEstimator,
    cross_targets,
    cross_waypoints,
)
from fusai_sdk.exceptions import CalibrationError


def test_cross_targets_has_nine_points_and_returns_to_center() -> None:
    points = cross_targets(100, 200, 10, 20)
    assert len(points) == 9
    assert points[0] == (100, 200)
    assert points[-1] == (100, 200)
    assert (110, 200) in points
    assert (90, 200) in points
    assert (100, 220) in points
    assert (100, 180) in points
    assert cross_waypoints(0, 0, 1, 2) == points[:0] + cross_targets(0, 0, 1, 2)


def test_cross_targets_rejects_negative_amplitude() -> None:
    with pytest.raises(ValueError):
        cross_targets(0, 0, -1, 0)


def test_jacobian_recovers_known_linear_map() -> None:
    true_j = ((2.0, 0.5), (0.25, 3.0))
    estimator = JacobianEstimator()
    for joint in ((0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)):
        observation = (
            true_j[0][0] * joint[0] + true_j[0][1] * joint[1],
            true_j[1][0] * joint[0] + true_j[1][1] * joint[1],
        )
        estimator.add(joint, observation)

    result = estimator.solve()
    assert result.sample_count == 4
    assert result.residual_rms == pytest.approx(0.0, abs=1e-9)
    assert result.condition_number > 0.0
    for row in range(2):
        for col in range(2):
            assert result.matrix[row][col] == pytest.approx(true_j[row][col], abs=1e-9)


def test_jacobian_is_centered_so_constant_offsets_do_not_bias_it() -> None:
    true_j = ((1.5, 0.0), (0.0, -2.0))
    estimator = JacobianEstimator(joint_dim=2, observation_dim=2)
    for joint in ((3.0, -4.0), (4.0, -4.0), (3.0, -3.0), (4.0, -3.0)):
        observation = (
            10.0 + true_j[0][0] * joint[0],
            -20.0 + true_j[1][1] * joint[1],
        )
        estimator.add(joint, observation)
    result = estimator.solve()
    assert result.matrix[0][0] == pytest.approx(1.5)
    assert result.matrix[1][1] == pytest.approx(-2.0)
    assert result.residual_rms == pytest.approx(0.0, abs=1e-9)


def test_jacobian_rejects_singular_excitation() -> None:
    estimator = JacobianEstimator()
    for _ in range(4):
        estimator.add((1.0, 1.0), (1.0, 1.0))
    with pytest.raises(CalibrationError):
        estimator.solve()


def test_jacobian_requires_enough_samples() -> None:
    estimator = JacobianEstimator()
    estimator.add((0.0, 0.0), (0.0, 0.0))
    estimator.add((1.0, 0.0), (1.0, 0.0))
    with pytest.raises(CalibrationError):
        estimator.solve()


def test_dimension_mismatch_is_rejected() -> None:
    estimator = JacobianEstimator()
    estimator.add((0.0, 0.0), (0.0, 0.0))
    with pytest.raises(ValueError):
        estimator.add((1.0,), (1.0, 0.0))
    with pytest.raises(ValueError):
        estimator.add((1.0, 1.0), (1.0,))


def test_add_sample_and_clear() -> None:
    estimator = JacobianEstimator()
    sample = CalibrationSample(joint=(1.0, 2.0), observation=(3.0, 4.0))
    estimator.add_sample(sample)
    assert len(estimator) == 1
    assert estimator.samples[0] is sample
    estimator.clear()
    assert len(estimator) == 0

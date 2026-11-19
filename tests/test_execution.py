import numpy as np
import pytest
from dexim.core.execution import ExecutionDescriptor, position_vector


@pytest.mark.parametrize(
    "value",
    [
        [True, 0.5],
        [0.5, np.bool_(True)],
        [0.5, "0.75"],
        np.array([0.5, "0.75"], dtype=object),
        [0.5, 1 + 0j],
        np.array([0.5, 1 + 0j]),
        [0.5, np.nan],
        [0.5, np.inf],
        [0.5, 10**400],
        np.array([0.5, np.finfo(np.longdouble).max], dtype=np.longdouble)
        if np.finfo(np.longdouble).max > np.finfo(float).max
        else [0.5, np.longdouble(np.inf)],
        [[0.5, 0.75]],
        None,
    ],
)
def test_position_vector_rejects_invalid_values_with_boundary_error(value):
    with pytest.raises(ValueError, match="Expected 2 finite positions"):
        position_vector(value, 2)


@pytest.mark.parametrize(
    "value",
    [(1, 0.5, np.int64(2), np.float32(0.25)), np.array([1, 0.5, 2, 0.25])],
)
def test_position_vector_preserves_real_positions(value):
    np.testing.assert_array_equal(position_vector(value, 4), [1, 0.5, 2, 0.25])


def test_execution_descriptor_stores_float_bounds_and_clips():
    descriptor = ExecutionDescriptor(
        "arm", "model", "digest", ("joint",), (np.int64(2),), (np.float32(10),)
    )

    assert type(descriptor.lower[0]) is float
    assert type(descriptor.upper[0]) is float
    assert descriptor.identity.startswith("sha256:")
    np.testing.assert_array_equal(descriptor.apply_limits([15]), [10.0])


def test_execution_descriptor_checks_interval_after_float_conversion():
    with pytest.raises(ValueError, match="Joint limits must have a nonempty interval"):
        ExecutionDescriptor(
            "arm", "model", "digest", ("joint",), (2**53,), (2**53 + 1,)
        )


@pytest.mark.parametrize("lower", [(True,), ("2",), None])
def test_execution_descriptor_rejects_invalid_bounds_with_boundary_error(lower):
    with pytest.raises(ValueError, match="Expected 1 finite positions"):
        ExecutionDescriptor("arm", "model", "digest", ("joint",), lower, (10,))

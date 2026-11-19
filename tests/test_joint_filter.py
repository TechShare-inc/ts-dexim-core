from __future__ import annotations

import numpy as np

from dexim.core.collaborators.joint_filter import JointFilter
from dexim.core.config import FilterConfig


def test_reset_prevents_previous_epoch_from_biasing_first_sample() -> None:
    joint_filter = JointFilter(
        FilterConfig(type="one_euro", freq=30.0, min_cutoff=1.0, beta=0.0),
        data_size=1,
    )
    joint_filter(np.array([1.0]))
    assert not np.allclose(joint_filter(np.array([0.0])), [0.0])

    joint_filter.reset()

    assert np.allclose(joint_filter(np.array([0.0])), [0.0])


def test_reset_is_a_noop_when_filtering_is_disabled() -> None:
    joint_filter = JointFilter(None, data_size=1)

    joint_filter.reset()

    assert np.allclose(joint_filter(np.array([0.5])), [0.5])

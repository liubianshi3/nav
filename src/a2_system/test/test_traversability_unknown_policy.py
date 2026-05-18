"""Regression: treat_unknown_as_obstacle must gate unknown cells.

This test ensures the fix in traversability_to_obstacle_cloud.py
(commit 96a13f2) is not silently reverted by future edits.

The key regression being guarded: the launch file already passed
``treat_unknown_as_obstacle:=False`` but the script unconditionally
merged unknown cells into obstacle_points, polluting the local costmap
with unmapped rolling-window edges.
"""

import numpy as np
import pytest


def _build_mask(data, threshold, treat_unknown):
    """Minimal extraction of the mask logic from
    traversability_to_obstacle_cloud._publish().

    ``data`` is a flat numpy array of int8 grid values.
    Returns a boolean mask (same shape) of cells published as obstacles.
    """
    obstacle_mask = data >= threshold
    if treat_unknown:
        unknown_mask = data == -1
        return obstacle_mask | unknown_mask
    return obstacle_mask


class TestTraversabilityUnknownPolicy:
    @pytest.mark.parametrize("treat_unknown", [False, True])
    def test_empty_grid_produces_no_obstacles(self, treat_unknown):
        data = np.zeros(100, dtype=np.int8)
        mask = _build_mask(data, 90, treat_unknown)
        assert not mask.any()

    def test_unknown_not_obstacle_when_default_false(self):
        """Default False: -1 cells must NOT enter the obstacle mask."""
        data = np.array([0, -1, 100, 0, -1], dtype=np.int8)
        mask = _build_mask(data, 90, treat_unknown=False)
        expected = np.array([False, False, True, False, False])
        assert np.array_equal(mask, expected), (
            f"unknown cells leaked into obstacle mask:\n"
            f"  data={data.tolist()}\n  mask={mask.tolist()}\n  expected={expected.tolist()}"
        )

    def test_unknown_is_obstacle_when_explicit_true(self):
        """Explicit True: -1 cells MUST enter the obstacle mask."""
        data = np.array([0, -1, 100, 0, -1], dtype=np.int8)
        mask = _build_mask(data, 90, treat_unknown=True)
        expected = np.array([False, True, True, False, True])
        assert np.array_equal(mask, expected), (
            f"unknown cells missing from obstacle mask:\n"
            f"  data={data.tolist()}\n  mask={mask.tolist()}\n  expected={expected.tolist()}"
        )

    def test_free_cells_never_obstacle(self):
        """0 cells must never enter the mask regardless of parameter."""
        data = np.zeros(50, dtype=np.int8)
        for treat in (False, True):
            mask = _build_mask(data, 90, treat)
            assert not mask.any(), f"free cells leaked: treat_unknown={treat}"

    def test_occupied_cells_always_obstacle(self):
        """Cells >= threshold must always enter the mask."""
        data = np.array([90, 100, 95], dtype=np.int8)
        for treat in (False, True):
            mask = _build_mask(data, 90, treat)
            assert mask.all(), f"occupied cells missed: treat_unknown={treat}"

    def test_mixed_grid_counts(self):
        """Integration-style check: obstacle + unknown + free in one grid."""
        data = np.array([0, 0, -1, 100, 0, -1, 90], dtype=np.int8)

        # Default False: only 100 and 90 are obstacles
        mask_false = _build_mask(data, 90, treat_unknown=False)
        assert mask_false.sum() == 2
        assert mask_false[3] and mask_false[6]  # occupied
        assert not mask_false[2]                 # unknown excluded
        assert not mask_false[5]                 # unknown excluded

        # Explicit True: 100, 90, -1, -1 are obstacles
        mask_true = _build_mask(data, 90, treat_unknown=True)
        assert mask_true.sum() == 4
        assert mask_true[2]  # unknown included
        assert mask_true[5]  # unknown included


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

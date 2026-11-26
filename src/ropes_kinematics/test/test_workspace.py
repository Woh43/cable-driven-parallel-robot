"""Checks for the nominal and cable-limited static workspace bounds."""

from itertools import product
import json
import math
from pathlib import Path

from ropes_kinematics.model import CableModel


_CONFIG_PATH = Path(__file__).resolve().parents[2] / "ropes_description" / "config" / "cables.json"


def _signed_double_area(xy):
    return sum(
        x0 * y1 - x1 * y0
        for (x0, y0), (x1, y1) in zip(xy, (*xy[1:], xy[0]))
    )


def test_default_static_workspace_is_the_projected_guide_triangle():
    model = CableModel.from_file(_CONFIG_PATH)

    xy, heights = model.static_workspace_bounds()

    assert len(xy) == 3
    assert {tuple(vertex) for vertex in xy} == {anchor[:2] for anchor in model.anchors}
    assert _signed_double_area(xy) > 0.0
    assert tuple(heights) == model.height_range


def test_shorter_cables_produce_a_smaller_prism_within_length_budgets():
    config = json.loads(_CONFIG_PATH.read_text(encoding="utf-8"))
    nominal = CableModel(config)
    nominal_xy, nominal_heights = nominal.static_workspace_bounds()
    for cable, nominal_length, maximum_length in zip(
        config["cables"], nominal.neutral_lengths, nominal.free_length_limits
    ):
        # Keep 30 mm of payout at home and exclude part of the nominal prism.
        cable["cut_length_m"] -= maximum_length - nominal_length - 0.030
    constrained = CableModel(config)

    xy, heights = constrained.static_workspace_bounds()

    assert len(xy) == 3 and len(heights) == 2
    assert all(math.isfinite(coordinate) for vertex in xy for coordinate in vertex)
    assert all(math.isfinite(height) for height in heights)
    assert nominal_heights[0] <= heights[0] < constrained.home[2]
    assert constrained.home[2] < heights[1] <= nominal_heights[1]
    area = _signed_double_area(xy)
    volume_twice = area * (heights[1] - heights[0])
    nominal_volume_twice = _signed_double_area(nominal_xy) * (
        nominal_heights[1] - nominal_heights[0]
    )
    assert 0.0 < volume_twice < nominal_volume_twice
    home_x, home_y, _ = constrained.home
    for (x0, y0), (x1, y1) in zip(xy, (*xy[1:], xy[0])):
        assert (x1 - x0) * (home_y - y0) - (y1 - y0) * (home_x - x0) > 0.0
    for (x, y), z in product(xy, heights):
        for anchor, limit in zip(constrained.anchors, constrained.free_length_limits):
            assert 0.0 < math.dist((x, y, z), anchor) <= limit + 2e-9

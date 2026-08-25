import numpy as np
import pytest

from embodied_swarm.core.frames import (
    ARM_BASE_FRAME,
    ARM_LINK_FRAMES,
    BASE_FRAME,
    END_EFFECTOR_FRAME,
    FRAME_CHAIN,
    HAND_FRAME,
    WORLD_FRAME,
    body_to_world_velocity,
    world_to_body_velocity,
)


def test_frame_names_and_complete_chain():
    assert WORLD_FRAME == "world"
    assert BASE_FRAME == "base_link"
    assert ARM_BASE_FRAME == "panda_link0"
    assert ARM_LINK_FRAMES == tuple(f"panda_link{i}" for i in range(1, 8))
    assert HAND_FRAME == "panda_hand"
    assert END_EFFECTOR_FRAME == "end_effector"
    assert FRAME_CHAIN == (
        "world",
        "base_link",
        "panda_link0",
        "panda_link1",
        "panda_link2",
        "panda_link3",
        "panda_link4",
        "panda_link5",
        "panda_link6",
        "panda_link7",
        "panda_hand",
        "end_effector",
    )


def test_frame_names_are_unique():
    assert len(FRAME_CHAIN) == len(set(FRAME_CHAIN))


@pytest.mark.parametrize("yaw", [0.0, np.pi / 4, np.pi / 2, -np.pi / 3])
def test_planar_velocity_round_trip(yaw):
    world = body_to_world_velocity(0.7, -0.2, yaw)
    body = world_to_body_velocity(*world, yaw)
    assert body == pytest.approx((0.7, -0.2))


def test_body_forward_at_90_degrees_is_world_positive_y():
    assert body_to_world_velocity(1.0, 0.0, np.pi / 2) == pytest.approx(
        (0.0, 1.0), abs=1e-12
    )


def test_velocity_transform_rejects_non_finite_input():
    with pytest.raises(ValueError):
        body_to_world_velocity(np.nan, 0.0, 0.0)

from embodied_swarm.core.frames import (
    WORLD_FRAME,
    BASE_FRAME,
    ARM_BASE_FRAME,
    END_EFFECTOR_FRAME,
    FRAME_CHAIN,
)


def test_frame_names():
    assert WORLD_FRAME == "world"
    assert BASE_FRAME == "base_link"
    assert ARM_BASE_FRAME == "panda_link0"
    assert END_EFFECTOR_FRAME == "end_effector"


def test_frame_chain():
    assert FRAME_CHAIN == (
        "world",
        "base_link",
        "panda_link0",
        "end_effector",
    )


def test_frame_names_are_unique():
    assert len(FRAME_CHAIN) == len(set(FRAME_CHAIN))
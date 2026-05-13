"""
hhand_control.py
----------------
Human hand synergic/single-index controller.
Port of hhand_control.m

Usage
-----
    from hhand_control import hhand_control_step, SYNERGIC_CLOSE, SINGLE_INDEX_CLOSE
    angle_index = hhand_control_step(hhand, mode, angle_index, ...)
"""

import numpy as np
from human_hand import HumanHand

# Mode constants
SYNERGIC_CLOSE     = 1
SINGLE_INDEX_CLOSE = 2


def hhand_control_step(hhand: HumanHand,
                        mode: int,
                        angle_index: int,
                        sim_start: int = 40,
                        sim_end:   int = 85) -> int:
    """
    Advance the human hand one control step.
    Port of hhand_control.m

    Parameters
    ----------
    hhand       : HumanHand instance
    mode        : SYNERGIC_CLOSE or SINGLE_INDEX_CLOSE
    angle_index : current 0-based simulation index
    sim_start   : start index for SYNERGIC_CLOSE wrapping
    sim_end     : end index for SYNERGIC_CLOSE wrapping

    Returns
    -------
    Updated angle_index (int, 0-based)
    """
    if mode == SYNERGIC_CLOSE:
        # Advance index with wrap-around
        if angle_index >= sim_end:
            angle_index = sim_start
        else:
            angle_index += 1

        idx = angle_index

        # Index finger
        hhand.index.prox_rt_Q   = hhand.index.prox_raster[idx]
        hhand.index.media_rt_Q  = hhand.index.media_raster[idx]
        hhand.index.distal_rt_Q = hhand.index.distal_raster[idx]
        hhand.index.abduct_rt_Q = hhand.index.abduct_raster[idx]

        # Middle finger
        hhand.middle.prox_rt_Q   = hhand.middle.prox_raster[idx]
        hhand.middle.media_rt_Q  = hhand.middle.media_raster[idx]
        hhand.middle.distal_rt_Q = hhand.middle.distal_raster[idx]
        hhand.middle.abduct_rt_Q = hhand.middle.abduct_raster[idx]

        # Ring finger
        hhand.ring.prox_rt_Q   = hhand.ring.prox_raster[idx]
        hhand.ring.media_rt_Q  = hhand.ring.media_raster[idx]
        hhand.ring.distal_rt_Q = hhand.ring.distal_raster[idx]
        hhand.ring.abduct_rt_Q = hhand.ring.abduct_raster[idx]

        # Little finger
        hhand.little.prox_rt_Q   = hhand.little.prox_raster[idx]
        hhand.little.media_rt_Q  = hhand.little.media_raster[idx]
        hhand.little.distal_rt_Q = hhand.little.distal_raster[idx]
        hhand.little.abduct_rt_Q = hhand.little.abduct_raster[idx]

        # Thumb
        hhand.thumb.prox_rt_Q   = hhand.thumb.prox_raster[idx]
        hhand.thumb.media_rt_Q  = hhand.thumb.media_raster[idx]
        hhand.thumb.distal_rt_Q = hhand.thumb.distal_raster[idx]
        hhand.thumb.abduct_rt_Q = hhand.thumb.abduct_raster[idx]

        # Forward kinematics for all fingers
        hhand.step_all()

    elif mode == SINGLE_INDEX_CLOSE:
        if angle_index >= 127:
            angle_index = 0
        else:
            angle_index += 1
        idx = angle_index

        hhand.index.prox_rt_Q   = hhand.index.prox_raster[idx]
        hhand.index.media_rt_Q  = hhand.index.media_raster[idx]
        hhand.index.distal_rt_Q = hhand.index.distal_raster[idx]
        hhand.index.abduct_rt_Q = hhand.index.abduct_raster[idx]

        # Other fingers follow index at reduced scale
        hhand.middle.prox_rt_Q   = hhand.index.prox_rt_Q   / 2
        hhand.middle.media_rt_Q  = hhand.index.media_rt_Q  / 4
        hhand.middle.distal_rt_Q = hhand.index.distal_rt_Q / 8
        hhand.middle.abduct_rt_Q = hhand.index.abduct_rt_Q / 4

        hhand.ring.prox_rt_Q   = hhand.index.prox_rt_Q   / 4
        hhand.ring.media_rt_Q  = hhand.index.media_rt_Q  / 4
        hhand.ring.distal_rt_Q = hhand.index.distal_rt_Q / 8
        hhand.ring.abduct_rt_Q = 0.0

        hhand.little.prox_rt_Q   = hhand.index.prox_rt_Q / 4
        hhand.little.media_rt_Q  = hhand.index.media_rt_Q / 4
        hhand.little.distal_rt_Q = 0.0
        hhand.little.abduct_rt_Q = 0.0

        hhand.thumb.prox_rt_Q   = 0.0
        hhand.thumb.media_rt_Q  = 0.0
        hhand.thumb.distal_rt_Q = 0.0
        hhand.thumb.abduct_rt_Q = 0.0

        hhand.step_all()

    return angle_index

"""
mapper_5_to_3.py
----------------
Maps 5-fingertip human hand coordinates → 3 bhand corner points,
then prepares the network input vector.
Port of mapper_5_to_3.m
"""

import numpy as np
from transforms import trans, X_AXIS, Y_AXIS, Z_AXIS


CORNER_COORDINATES       = 2
TRIANGLE_DIMENSIONS      = 1
SYMMETRY_FRAME_COORDS    = 3


def mapper_5_to_3(hhand, mapper_cfg: dict, mode_j: int = CORNER_COORDINATES):
    """
    Reduce the human-hand 5-fingertip triangle to a 3-point bhand target.

    Parameters
    ----------
    hhand      : HumanHand instance (finger.tip must already be updated)
    mapper_cfg : dict with keys
                   index_weight, middle_weight, ring_weight, little_weight
    mode_j     : CORNER_COORDINATES (default), TRIANGLE_DIMENSIONS,
                 or SYMMETRY_FRAME_COORDS

    Returns
    -------
    simnet_input_vector : shape-(9,) array ready for the backward-kin net
    corner_A, corner_B, corner_C : the three bhand target corner points
    """
    iw = mapper_cfg.get('index_weight',  1.0)
    mw = mapper_cfg.get('middle_weight', 1.0)
    rw = mapper_cfg.get('ring_weight',   1.0)
    lw = mapper_cfg.get('little_weight', 1.0)
    dw_sum = iw + mw + rw + lw

    # ── Spread center (weighted average of digit tips) ────────────────────
    spread_center = (iw * hhand.index.tip
                     + mw * hhand.middle.tip
                     + rw * hhand.ring.tip
                     + lw * hhand.little.tip) / dw_sum

    # ── Symmetry frame of reference ───────────────────────────────────────
    def _sym_frame_from(finger, w):
        """Shift finger's tip frame to the spread center."""
        sc_in_tip = finger.tip_A @ np.array([
            spread_center[0], spread_center[1], spread_center[2], 1.0])
        return trans(sc_in_tip[0], sc_in_tip[1], sc_in_tip[2]) @ finger.tip_A

    A1 = _sym_frame_from(hhand.index,  iw)
    A2 = _sym_frame_from(hhand.middle, mw)
    A3 = _sym_frame_from(hhand.ring,   rw)
    A4 = _sym_frame_from(hhand.little, lw)

    A_sym = (iw*A1 + mw*A2 + rw*A3 + lw*A4) / dw_sum

    # ── Thumb projection onto A_sym x-z plane ────────────────────────────
    tt = hhand.thumb.tip
    tt_at_sym = A_sym @ np.array([tt[0], tt[1], tt[2], 1.0])
    projected_thumb_sym = np.array([tt_at_sym[0], 0.0, tt_at_sym[2]])

    # Back to world coordinates
    inv_sym = np.linalg.inv(A_sym)
    proj4   = inv_sym @ np.array([
        projected_thumb_sym[0], projected_thumb_sym[1], projected_thumb_sym[2], 1.0])
    projected_thumb_world = proj4[:3]

    # ── Corner points ─────────────────────────────────────────────────────
    corner_A = (iw * hhand.index.tip  + mw * hhand.middle.tip) / (iw + mw)
    corner_B = (rw * hhand.ring.tip   + lw * hhand.little.tip) / (rw + lw)
    corner_C = projected_thumb_world

    # ── Build simnet_input_vector ─────────────────────────────────────────
    if mode_j in (CORNER_COORDINATES, TRIANGLE_DIMENSIONS):
        simnet_input = np.array([
            corner_A[0], corner_A[1], corner_A[2],
            corner_B[0], corner_B[1], corner_B[2],
            corner_C[0], corner_C[1], corner_C[2],
        ])

    elif mode_j == SYMMETRY_FRAME_COORDS:
        def _to_sym(pt):
            p4 = A_sym @ np.array([pt[0], pt[1], pt[2], 1.0])
            return p4[:3]

        cA_sym = _to_sym(corner_A)
        cB_sym = _to_sym(corner_B)
        cC_sym = projected_thumb_sym

        simnet_input = np.array([
            cA_sym[0], cA_sym[1], cA_sym[2],
            cB_sym[0], cB_sym[1], cB_sym[2],
            cC_sym[0], cC_sym[1], cC_sym[2],
        ])
    else:
        raise ValueError(f"Unknown mode_j: {mode_j}")

    return simnet_input, corner_A, corner_B, corner_C

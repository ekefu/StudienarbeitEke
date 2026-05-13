"""
bhand_control.py
----------------
Barrett hand joint-space controller ported from:
  bhand_control.m, angle2encoder.m

Provides:
  - angle2encoder(bhand, rwl_out)   → motor_step (4-element array)
  - bhand_control_step(bhand, mode, angle_index)
"""

import numpy as np
from barrett_hand import BarrettHand, CONTROL_MODE, CLOSE_SIM_MODE


# ─────────────────────────────────────────────────────────────────────────────
def angle2encoder(bhand: BarrettHand, rwl_out: np.ndarray) -> np.ndarray:
    """
    Convert joint angles to motor encoder steps.
    Port of angle2encoder.m

    Parameters
    ----------
    bhand   : BarrettHand instance (provides R-range bounds)
    rwl_out : shape-(4,) array  [alfa, beta_left, beta_right, beta_ns]

    Returns
    -------
    motor_step : shape-(4,) integer array
    """
    rwl = rwl_out.copy()

    # ── 1) Clip to realtime range ─────────────────────────────────────────
    # Abduction
    rwl[0] = np.clip(rwl[0], bhand.abduct_lower_R, bhand.abduct_border)
    # Flexion joints
    for i in range(1, 4):
        rwl[i] = np.clip(rwl[i], bhand.media_lower_R, bhand.media_upper_R)

    # ── 2) Convert angles to integer encoder steps ────────────────────────
    motor_step = np.zeros(4, dtype=int)
    motor_step[0] = int(rwl[0] * 6366.1977)    # abduct: step = angle * 20000/pi
    motor_step[1] = int(rwl[1] * 8185.1114)    # flex:   step = angle * 20000*9/(7*pi)
    motor_step[2] = int(rwl[2] * 8185.1114)
    motor_step[3] = int(rwl[3] * 8185.1114)

    return motor_step


# ─────────────────────────────────────────────────────────────────────────────
def bhand_control_step(bhand: BarrettHand, mode: int,
                       angle_index: int,
                       alignment_A=None) -> None:
    """
    Set joint angles from encoder values, run forward kinematics.
    Port of bhand_control.m

    Parameters
    ----------
    bhand       : BarrettHand instance
    mode        : CLOSE_SIM_MODE or CONTROL_MODE
    angle_index : 0-based index into angle rasters
    alignment_A : optional 4x4 alignment matrix (CONTROL_MODE only)
    """
    if mode == CLOSE_SIM_MODE:
        idx = angle_index
        bhand.left_abduct_rt_Q   = bhand.q_abduct[idx]
        bhand.left_media_rt_Q    = bhand.q_free_flex1[idx]
        bhand.left_distal_rt_Q   = bhand.q_free_flex2[idx]
        bhand.right_abduct_rt_Q  = bhand.q_abduct[idx]
        bhand.right_media_rt_Q   = bhand.q_free_flex1[idx]
        bhand.right_distal_rt_Q  = bhand.q_free_flex2[idx]
        bhand.ns_media_rt_Q      = bhand.q_free_flex1[idx]
        bhand.ns_distal_rt_Q     = bhand.q_free_flex2[idx]
        bhand.fwd(alignment_A)
        bhand.compute_points()

    elif mode == CONTROL_MODE:
        bhand.left_abduct_rt_Q   = bhand.q_abduct[bhand.ctrl_spread]
        bhand.left_media_rt_Q    = bhand.q_free_flex1[bhand.ctrl_left_m]
        bhand.left_distal_rt_Q   = bhand.q_free_flex2[bhand.ctrl_left_m]
        bhand.right_abduct_rt_Q  = bhand.q_abduct[bhand.ctrl_spread]
        bhand.right_media_rt_Q   = bhand.q_free_flex1[bhand.ctrl_right_m]
        bhand.right_distal_rt_Q  = bhand.q_free_flex2[bhand.ctrl_right_m]
        bhand.ns_media_rt_Q      = bhand.q_free_flex1[bhand.ctrl_nonspread]
        bhand.ns_distal_rt_Q     = bhand.q_free_flex2[bhand.ctrl_nonspread]
        bhand.fwd(alignment_A)
        bhand.compute_points()

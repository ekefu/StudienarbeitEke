"""
human_hand.py
-------------
Human Hand model ported from:
  human_hand.m, index/middle/ring/little/thumb _calibration.m
  index/middle/ring/little/thumb _fwd.m
  index/middle/ring/little/thumb _points.m
  hhand_memory.m
"""

import numpy as np
from Simulation_Python.transforms import rotate, trans, hom2vect, X_AXIS, Y_AXIS, Z_AXIS


class Finger:
    """Stores per-joint parameters and current angles for one finger."""

    def __init__(self, prox_len, media_len, distal_len,
                 base_point, length_resolution,
                 prox_max, media_max, distal_max, abduct_max,
                 abduct_sign=1.0):
        self.prox_len   = prox_len
        self.media_len  = media_len
        self.distal_len = distal_len
        self.base_point = np.array(base_point, dtype=float)
        self.lr         = length_resolution
        self.abduct_sign = abduct_sign

        # Link delta-x sample arrays
        self.prox_delta_x   = np.linspace(0, prox_len,   length_resolution)
        self.media_delta_x  = np.linspace(0, media_len,  length_resolution)
        self.distal_delta_x = np.linspace(0, distal_len, length_resolution)

        # Joint angle ranges
        self.prox_range   = prox_max
        self.media_range  = media_max
        self.distal_range = distal_max
        self.abduct_range = abduct_max

        # Raster arrays (128 steps, matching CyberGlove 8-bit resolution)
        self.prox_raster   = np.linspace(0, prox_max,    128)
        self.media_raster  = np.linspace(0, media_max,   128)
        self.distal_raster = np.linspace(0, distal_max,  128)
        self.abduct_raster = abduct_sign * np.linspace(0, abduct_max, 128)

        # Current joint angles (set by controller)
        self.prox_rt_Q   = 0.0
        self.media_rt_Q  = 0.0
        self.distal_rt_Q = 0.0
        self.abduct_rt_Q = 0.0

        # Homogeneous frames (set by fwd())
        self.prox_A   = np.eye(4)
        self.media_A  = np.eye(4)
        self.distal_A = np.eye(4)
        self.tip_A    = np.eye(4)
        self.base_A   = trans(*base_point)

        # Cartesian output
        self.q2cart        = np.zeros((length_resolution * 3, 3))
        self.tip           = np.zeros(3)
        self.tip_past1     = np.zeros(3)
        self.tip_past2     = np.zeros(3)


class HumanHand:
    """
    Left-hand model.

    Parameters
    ----------
    length_resolution : number of sample points per link (default 5)
    with_memory       : keep past two tip positions (default True)
    """

    def __init__(self, length_resolution: int = 5, with_memory: bool = True):
        lr = length_resolution
        self.with_memory = with_memory

        # ── Finger objects ────────────────────────────────────────────────
        # index_calibration.m
        self.index  = Finger(28, 20, 15, [-30, -30, 0], lr,
                             prox_max=np.pi/2, media_max=5*np.pi/9,
                             distal_max=np.pi/4, abduct_max=np.pi/9,
                             abduct_sign=-1.0)

        # middle_calibration.m
        self.middle = Finger(30, 25, 20, [-35, -10, 0], lr,
                             prox_max=np.pi/2, media_max=5*np.pi/9,
                             distal_max=np.pi/4, abduct_max=np.pi/9,
                             abduct_sign=-1.0)

        # ring_calibration.m
        self.ring   = Finger(30, 25, 20, [-35,  10, 0], lr,
                             prox_max=np.pi/2, media_max=5*np.pi/9,
                             distal_max=np.pi/4, abduct_max=np.pi/9,
                             abduct_sign=-1.0)

        # little_calibration.m
        self.little = Finger(25, 15, 15, [-30,  30, 0], lr,
                             prox_max=np.pi/2, media_max=5*np.pi/9,
                             distal_max=np.pi/4, abduct_max=np.pi/9,
                             abduct_sign=-1.0)

        # thumb_calibration.m (rough approximation; should be re-calibrated)
        self.thumb  = Finger(45, 20, 15, [ 30, -15, 0], lr,
                             prox_max=np.pi/3, media_max=np.pi/3,
                             distal_max=np.pi/4, abduct_max=np.pi/4,
                             abduct_sign=1.0)

        # ── Palm outline ──────────────────────────────────────────────────
        self.palm_x = np.array([-30,  30,  35,  35,  30, -30, -30], dtype=float)
        self.palm_y = np.array([ 15,  30,  10, -10, -30, -30,  15], dtype=float)
        self.palm_z = np.zeros(7)

        # ── Angle resolution (CyberGlove 8-bit) ───────────────────────────
        self.angle_resolution = 128

    # ─────────────────────────────────────────────────────────────────────────
    def fwd_index(self):
        """Port of index_fwd.m"""
        f = self.index
        f.prox_A  = (rotate(Y_AXIS,  np.pi/2 + f.prox_rt_Q)
                     @ rotate(Z_AXIS, f.abduct_rt_Q)
                     @ f.base_A)
        f.media_A = (rotate(Y_AXIS, f.media_rt_Q)
                     @ trans(0, 0, f.prox_len)
                     @ f.prox_A)
        f.distal_A = (rotate(Y_AXIS, f.distal_rt_Q)
                      @ trans(0, 0, f.media_len)
                      @ f.media_A)
        f.tip_A = trans(0, 0, f.distal_len) @ f.distal_A

    def fwd_middle(self):
        """Port of middle_fwd.m"""
        f = self.middle
        f.prox_A  = (rotate(Y_AXIS,  np.pi/2 + f.prox_rt_Q)
                     @ rotate(Z_AXIS, f.abduct_rt_Q)
                     @ f.base_A)
        f.media_A = (rotate(Y_AXIS, f.media_rt_Q)
                     @ trans(0, 0, f.prox_len)
                     @ f.prox_A)
        f.distal_A = (rotate(Y_AXIS, f.distal_rt_Q)
                      @ trans(0, 0, f.media_len)
                      @ f.media_A)
        f.tip_A = trans(0, 0, f.distal_len) @ f.distal_A

    def fwd_ring(self):
        """Port of ring_fwd.m"""
        f = self.ring
        f.prox_A  = (rotate(Y_AXIS,  np.pi/2 + f.prox_rt_Q)
                     @ rotate(Z_AXIS, f.abduct_rt_Q)
                     @ f.base_A)
        f.media_A = (rotate(Y_AXIS, f.media_rt_Q)
                     @ trans(0, 0, f.prox_len)
                     @ f.prox_A)
        f.distal_A = (rotate(Y_AXIS, f.distal_rt_Q)
                      @ trans(0, 0, f.media_len)
                      @ f.media_A)
        f.tip_A = trans(0, 0, f.distal_len) @ f.distal_A

    def fwd_little(self):
        """Port of little_fwd.m"""
        f = self.little
        f.prox_A  = (rotate(Y_AXIS,  np.pi/2 + f.prox_rt_Q)
                     @ rotate(Z_AXIS, f.abduct_rt_Q)
                     @ f.base_A)
        f.media_A = (rotate(Y_AXIS, f.media_rt_Q)
                     @ trans(0, 0, f.prox_len)
                     @ f.prox_A)
        f.distal_A = (rotate(Y_AXIS, f.distal_rt_Q)
                      @ trans(0, 0, f.media_len)
                      @ f.media_A)
        f.tip_A = trans(0, 0, f.distal_len) @ f.distal_A

    def fwd_thumb(self):
        """Port of thumb_fwd.m"""
        f = self.thumb
        f.prox_A = (rotate(X_AXIS, f.prox_rt_Q + np.pi/18)
                    @ rotate(Z_AXIS, f.abduct_rt_Q - 3*np.pi/4)
                    @ f.base_A)
        f.media_A = (rotate(Z_AXIS, f.media_rt_Q)
                     @ trans(0, f.prox_len, 0)
                     @ f.prox_A)
        f.distal_A = (rotate(Z_AXIS, f.distal_rt_Q)
                      @ trans(0, f.media_len, 0)
                      @ f.media_A)
        f.tip_A = trans(0, f.distal_len, 0) @ f.distal_A

    # ── Point computation ────────────────────────────────────────────────────
    def _points_for(self, finger: Finger, delta_along_z: bool = True):
        """
        Compute link sample points in world coordinates.
        Port of *_points.m files.

        delta_along_z : True for digit fingers (index/middle/ring/little)
                        False for thumb (translation is along y axis in thumb_fwd)
        """
        lr  = finger.lr
        inv = np.linalg.inv

        for i in range(lr):
            d = finger.prox_delta_x[i]
            T = trans(0, 0, d) if delta_along_z else trans(0, d, 0)
            finger.q2cart[i, :] = hom2vect(inv(T @ finger.prox_A))

            d = finger.media_delta_x[i]
            T = trans(0, 0, d) if delta_along_z else trans(0, d, 0)
            finger.q2cart[i + lr, :] = hom2vect(inv(T @ finger.media_A))

            d = finger.distal_delta_x[i]
            T = trans(0, 0, d) if delta_along_z else trans(0, d, 0)
            finger.q2cart[i + 2*lr, :] = hom2vect(inv(T @ finger.distal_A))

        if self.with_memory:
            finger.tip_past2 = finger.tip_past1.copy()
            finger.tip_past1 = finger.tip.copy()
        finger.tip = hom2vect(inv(finger.tip_A))

    def points_index(self):  self._points_for(self.index)
    def points_middle(self): self._points_for(self.middle)
    def points_ring(self):   self._points_for(self.ring)
    def points_little(self): self._points_for(self.little)
    def points_thumb(self):  self._points_for(self.thumb, delta_along_z=False)

    # ── Combined forward + points ────────────────────────────────────────────
    def step_all(self):
        """Run all forward kinematics and compute all link points."""
        self.fwd_index();   self.points_index()
        self.fwd_middle();  self.points_middle()
        self.fwd_ring();    self.points_ring()
        self.fwd_little();  self.points_little()
        self.fwd_thumb();   self.points_thumb()

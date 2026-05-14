"""
barrett_hand.py
---------------
Barrett Hand model ported from:
  barrett_hand.m

Builds all link geometry, joint-space rasters, and loads (or trains)
the backward-kinematics MLP.
"""

import os
import numpy as np
import joblib

from Simulation_Python.transforms import trans, rotate, X_AXIS, Y_AXIS, Z_AXIS

# ── Mode constants (must match initials.py) ──────────────────────────────────
CONTROL_MODE   = 1
CLOSE_SIM_MODE = 2


class BarrettHand:
    """
    Barrett Hand model.

    Parameters
    ----------
    mode_b     : CONTROL_MODE or CLOSE_SIM_MODE
    mapper     : mapper configuration dict (used in CONTROL_MODE)
    length_resolution : number of sample points per link
    """

    def __init__(self, mode_b: int = CLOSE_SIM_MODE, mapper: dict = None,
                 length_resolution: int = 5):
        self.mode_b = mode_b
        self.mapper = mapper or {}
        self.length_resolution = length_resolution

        # ── Link lengths (mm) ─────────────────────────────────────────────
        self.left_abduct_length   = 50.0
        self.right_abduct_length  = 50.0
        self.left_media_length    = 70.0
        self.right_media_length   = 70.0
        self.ns_media_length      = 70.0   # nonspread
        self.left_distal_length   = 56.0
        self.right_distal_length  = 56.0
        self.ns_distal_length     = 56.0

        # ── Angle resolution ──────────────────────────────────────────────
        self.angle_resolution = 20000 if mode_b == CONTROL_MODE else 128

        # ── Angle range P ─────────────────────────────────────────────────
        self.abduct_border = np.pi
        self.q_abduct      = np.linspace(0, self.abduct_border,
                                         self.angle_resolution)

        self.q_free_flex1  = np.linspace(0, 7 * np.pi / 9,
                                         self.angle_resolution)
        self.q_free_flex2  = np.array([
            np.pi / 2
            if (np.pi / 4 + 4.0 * q / 3.0) >= np.pi / 2
            else (np.pi / 4 + 4.0 * q / 3.0)
            for q in self.q_free_flex1
        ])

        # ── Training range T ──────────────────────────────────────────────
        self.abduct_lower_T = np.pi / 2
        self.media_lower_T  = np.pi / 2 - np.arctan2(56, 70)
        self.media_upper_T  = 135 * np.pi / 180 - np.arctan2(56, 70)

        # ── Realtime range R ──────────────────────────────────────────────
        self.abduct_lower_R = 92 * np.pi / 180
        self.media_lower_R  = 92 * np.pi / 180 - np.arctan2(56, 70)
        self.media_upper_R  = 133 * np.pi / 180 - np.arctan2(56, 70)

        # ── Training rasters ──────────────────────────────────────────────
        self.train_pair_resolution = 23
        self.abduct_raster = np.linspace(
            self.abduct_lower_T, self.abduct_border,
            self.train_pair_resolution)
        self.raster_flex   = np.linspace(
            self.media_lower_T, self.media_upper_T,
            self.train_pair_resolution)

        # ── Link delta-x sample arrays ────────────────────────────────────
        lr = self.length_resolution
        self.left_abduct_delta_x  = np.linspace(0, self.left_abduct_length,  lr)
        self.left_media_delta_x   = np.linspace(0, self.left_media_length,   lr)
        self.left_distal_delta_x  = np.linspace(0, self.left_distal_length,  lr)
        self.right_abduct_delta_x = np.linspace(0, self.right_abduct_length, lr)
        self.right_media_delta_x  = np.linspace(0, self.right_media_length,  lr)
        self.right_distal_delta_x = np.linspace(0, self.right_distal_length, lr)
        self.ns_media_delta_x     = np.linspace(0, self.ns_media_length,     lr)
        self.ns_distal_delta_x    = np.linspace(0, self.ns_distal_length,    lr)

        # ── Realtime joint angles (set by controller) ─────────────────────
        self.left_abduct_rt_Q   = 0.0
        self.left_media_rt_Q    = 0.0
        self.left_distal_rt_Q   = 0.0
        self.right_abduct_rt_Q  = 0.0
        self.right_media_rt_Q   = 0.0
        self.right_distal_rt_Q  = 0.0
        self.ns_media_rt_Q      = 0.0
        self.ns_distal_rt_Q     = 0.0

        # ── Control encoder indices (integer) ─────────────────────────────
        self.ctrl_spread     = 0
        self.ctrl_left_m     = 0
        self.ctrl_right_m    = 0
        self.ctrl_nonspread  = 0

        # ── Cartesian output (filled by barrett_fwd + barrett_points) ─────
        self.q2cart_left      = np.zeros((lr * 3, 3))
        self.q2cart_right     = np.zeros((lr * 3, 3))
        self.q2cart_nonspread = np.zeros((lr * 2, 3))
        self.left_tip         = np.zeros(3)
        self.right_tip        = np.zeros(3)
        self.ns_tip           = np.zeros(3)
        self.palm_x = self.palm_y = self.palm_z = None

        # ── Neural network (backward kinematics) ──────────────────────────
        self.net = None
        self._load_or_train_net()

    # ─────────────────────────────────────────────────────────────────────────
    def _load_or_train_net(self):
        """Load a pre-trained MLP or train one from scratch."""
        model_path = "load_net_rwl_points_elman.pkl"
        if os.path.exists(model_path):
            self.net = joblib.load(model_path)
            print(f"[BarrettHand] Loaded backward-kinematics net from '{model_path}'.")
        else:
            print("[BarrettHand] No saved net found – training a new one …")
            self._train_net()
            joblib.dump(self.net, model_path)
            print(f"[BarrettHand] Saved trained net to '{model_path}'.")

    def _train_net(self):
        """
        Train the backward kinematics MLP.
        Port of train_rwl_points_elman.m

        Generates random trajectories in joint space → forward kinematics →
        Cartesian finger-tip coordinates, then trains an MLPRegressor to map
        Cartesian tip coords back to joint angles.
        """
        from sklearn.neural_network import MLPRegressor
        from sklearn.preprocessing import MinMaxScaler

        max_jump_width     = 7
        trajectory_count   = 50
        trajectory_length  = 50

        X_all = []   # inputs  : 9 Cartesian coordinates
        y_all = []   # outputs : 4 joint angles

        alfa_idx    = self.train_pair_resolution // 2
        bl_idx = br_idx = bns_idx = 0

        for _ in range(trajectory_count):
            left_x = np.zeros(trajectory_length)
            left_y = np.zeros(trajectory_length)
            left_z = np.zeros(trajectory_length)
            right_x = np.zeros(trajectory_length)
            right_y = np.zeros(trajectory_length)
            right_z = np.zeros(trajectory_length)
            ns_x = np.zeros(trajectory_length)
            ns_y = np.zeros(trajectory_length)
            ns_z = np.zeros(trajectory_length)
            alfa_t  = np.zeros(trajectory_length)
            bl_t    = np.zeros(trajectory_length)
            br_t    = np.zeros(trajectory_length)
            bns_t   = np.zeros(trajectory_length)

            for i in range(trajectory_length):
                # Random walk in index space
                alfa_idx = int(np.clip(
                    alfa_idx + int(max_jump_width * (np.random.rand() - 0.5)),
                    0, self.train_pair_resolution - 1))
                bl_idx = int(np.clip(
                    bl_idx + int(max_jump_width * (np.random.rand() - 0.5)),
                    0, self.train_pair_resolution - 1))
                br_idx = int(np.clip(
                    br_idx + int(max_jump_width * (np.random.rand() - 0.5)),
                    0, self.train_pair_resolution - 1))
                bns_idx = int(np.clip(
                    bns_idx + int(max_jump_width * (np.random.rand() - 0.5)),
                    0, self.train_pair_resolution - 1))

                alfa       = self.abduct_raster[alfa_idx]
                beta_left  = self.raster_flex[bl_idx]
                beta_right = self.raster_flex[br_idx]
                beta_ns    = self.raster_flex[bns_idx]

                # Set joint angles
                self.left_abduct_rt_Q  = alfa
                self.left_media_rt_Q   = beta_left
                self.left_distal_rt_Q  = np.pi / 2
                self.right_abduct_rt_Q = alfa
                self.right_media_rt_Q  = beta_right
                self.right_distal_rt_Q = np.pi / 2
                self.ns_media_rt_Q     = beta_ns
                self.ns_distal_rt_Q    = np.pi / 2

                self.fwd()
                self.compute_points()

                left_x[i]  = self.left_tip[X_AXIS - 1]
                left_y[i]  = self.left_tip[Y_AXIS - 1]
                left_z[i]  = self.left_tip[Z_AXIS - 1]
                right_x[i] = self.right_tip[X_AXIS - 1]
                right_y[i] = self.right_tip[Y_AXIS - 1]
                right_z[i] = self.right_tip[Z_AXIS - 1]
                ns_x[i]    = self.ns_tip[X_AXIS - 1]
                ns_y[i]    = 0.0
                ns_z[i]    = self.ns_tip[Z_AXIS - 1]
                alfa_t[i]  = alfa
                bl_t[i]    = beta_left
                br_t[i]    = beta_right
                bns_t[i]   = beta_ns

            # Build training pairs for this trajectory
            for i in range(trajectory_length):
                X_all.append([left_x[i],  left_y[i],  left_z[i],
                               right_x[i], right_y[i], right_z[i],
                               ns_x[i],    ns_y[i],    ns_z[i]])
                y_all.append([alfa_t[i], bl_t[i], br_t[i], bns_t[i]])

        X_arr = np.array(X_all)
        y_arr = np.array(y_all)

        # Scale inputs to [-1, 1] (mirrors MATLAB newelm input range)
        self._scaler = MinMaxScaler(feature_range=(-1, 1))
        X_scaled = self._scaler.fit_transform(X_arr)

        # MLP architecture: [240, 480, 128, 4] with tansig≈tanh activations
        # 'lbfgs' is a good quasi-Newton solver for medium-sized problems.
        self.net = MLPRegressor(
            hidden_layer_sizes=(240, 480, 128),
            activation='tanh',
            solver='adam',
            learning_rate_init=0.001,
            max_iter=500,
            random_state=42,
            verbose=False,
        )
        self.net.fit(X_scaled, y_arr)

        # Persist scaler together with model so predict() can use it
        self.net._input_scaler = self._scaler

    # ─────────────────────────────────────────────────────────────────────────
    def predict_rwl(self, simnet_input: np.ndarray) -> np.ndarray:
        """
        Run the backward kinematics network.
        Replaces:  rwl_out = sim(net_rwl_points_elman, simnet_input_vector)

        Parameters
        ----------
        simnet_input : shape (9,) array of [lx,ly,lz, rx,ry,rz, nx,ny,nz]

        Returns
        -------
        shape (4,) array [alfa, beta_left, beta_right, beta_ns]
        """
        scaler = self.net._input_scaler
        X_scaled = scaler.transform(simnet_input.reshape(1, -1))
        return self.net.predict(X_scaled).flatten()

    # ─────────────────────────────────────────────────────────────────────────
    def fwd(self, alignment_A: np.ndarray = None):
        """
        Forward kinematics.  Port of barrett_fwd.m

        Parameters
        ----------
        alignment_A : optional 4x4 alignment matrix (used in CONTROL_MODE).
                      When None, identity is used (CLOSE_SIM_MODE).
        """
        if alignment_A is None:
            alignment_A = np.eye(4)

        # Base frames
        left_base_A  = trans(-20, -25, 0) @ alignment_A
        right_base_A = trans(-20,  25, 0) @ alignment_A
        ns_base_A    = trans( 30,   0, 0) @ alignment_A

        # Abduction
        self._left_abduct_A  = rotate(Z_AXIS,  self.left_abduct_rt_Q)  @ left_base_A
        self._right_abduct_A = rotate(Z_AXIS, -self.right_abduct_rt_Q) @ right_base_A

        # Medial
        self._left_media_A   = (rotate(Y_AXIS, -self.left_media_rt_Q)
                                @ trans(self.left_abduct_length, 0, 0)
                                @ self._left_abduct_A)
        self._right_media_A  = (rotate(Y_AXIS, -self.right_media_rt_Q)
                                @ trans(self.right_abduct_length, 0, 0)
                                @ self._right_abduct_A)
        self._ns_media_A     = rotate(Y_AXIS, -self.ns_media_rt_Q) @ ns_base_A

        # Distal
        self._left_distal_A  = (rotate(Y_AXIS, -self.left_distal_rt_Q)
                                @ trans(self.left_media_length, 0, 0)
                                @ self._left_media_A)
        self._right_distal_A = (rotate(Y_AXIS, -self.right_distal_rt_Q)
                                @ trans(self.right_media_length, 0, 0)
                                @ self._right_media_A)
        self._ns_distal_A    = (rotate(Y_AXIS, -self.ns_distal_rt_Q)
                                @ trans(self.ns_media_length, 0, 0)
                                @ self._ns_media_A)

        # Tip frames
        self._left_tip_A  = trans(self.left_distal_length,  0, 0) @ self._left_distal_A
        self._right_tip_A = trans(self.right_distal_length, 0, 0) @ self._right_distal_A
        self._ns_tip_A    = trans(self.ns_distal_length,    0, 0) @ self._ns_distal_A

        # Cached for point-geometry
        self._left_base_A  = left_base_A
        self._right_base_A = right_base_A
        self._ns_base_A    = ns_base_A

    # ─────────────────────────────────────────────────────────────────────────
    def compute_points(self):
        """
        Compute Cartesian link sample points and tip coordinates.
        Port of barrett_points.m
        """
        lr  = self.length_resolution
        inv = np.linalg.inv

        # Palm corner points
        def base_pt(base_A):
            return hom2vect_local(inv(trans(0.0001, 0, 0) @ base_A))

        bpalm = np.column_stack([
            base_pt(self._left_base_A),
            base_pt(self._right_base_A),
            base_pt(self._ns_base_A),
            base_pt(self._left_base_A),
        ])
        self.palm_x = bpalm[0, :]
        self.palm_y = bpalm[1, :]
        self.palm_z = bpalm[2, :]

        # Left finger
        for i in range(lr):
            self.q2cart_left[i, :] = hom2vect_local(
                inv(trans(self.left_abduct_delta_x[i], 0, 0) @ self._left_abduct_A))
            self.q2cart_left[i + lr, :] = hom2vect_local(
                inv(trans(self.left_media_delta_x[i], 0, 0) @ self._left_media_A))
            self.q2cart_left[i + 2*lr, :] = hom2vect_local(
                inv(trans(self.left_distal_delta_x[i], 0, 0) @ self._left_distal_A))
        self.left_tip = hom2vect_local(inv(self._left_tip_A))

        # Right finger
        for i in range(lr):
            self.q2cart_right[i, :] = hom2vect_local(
                inv(trans(self.right_abduct_delta_x[i], 0, 0) @ self._right_abduct_A))
            self.q2cart_right[i + lr, :] = hom2vect_local(
                inv(trans(self.right_media_delta_x[i], 0, 0) @ self._right_media_A))
            self.q2cart_right[i + 2*lr, :] = hom2vect_local(
                inv(trans(self.right_distal_delta_x[i], 0, 0) @ self._right_distal_A))
        self.right_tip = hom2vect_local(inv(self._right_tip_A))

        # Nonspread finger
        for i in range(lr):
            self.q2cart_nonspread[i, :] = hom2vect_local(
                inv(trans(self.ns_media_delta_x[i], 0, 0) @ self._ns_media_A))
            self.q2cart_nonspread[i + lr, :] = hom2vect_local(
                inv(trans(self.ns_distal_delta_x[i], 0, 0) @ self._ns_distal_A))
        self.ns_tip = hom2vect_local(inv(self._ns_tip_A))

    # ─────────────────────────────────────────────────────────────────────────
    # Convenience properties (keep legacy names used across scripts)
    @property
    def left_abduct_length(self):  return self.left_abduct_length_val
    @left_abduct_length.setter
    def left_abduct_length(self, v): self.left_abduct_length_val = v

    @property
    def right_abduct_length(self): return self.right_abduct_length_val
    @right_abduct_length.setter
    def right_abduct_length(self, v): self.right_abduct_length_val = v

    @property
    def left_media_length(self):   return self.left_media_length_val
    @left_media_length.setter
    def left_media_length(self, v): self.left_media_length_val = v

    @property
    def right_media_length(self):  return self.right_media_length_val
    @right_media_length.setter
    def right_media_length(self, v): self.right_media_length_val = v

    @property
    def ns_media_length(self):     return self.ns_media_length_val
    @ns_media_length.setter
    def ns_media_length(self, v):  self.ns_media_length_val = v

    @property
    def left_distal_length(self):  return self.left_distal_length_val
    @left_distal_length.setter
    def left_distal_length(self, v): self.left_distal_length_val = v

    @property
    def right_distal_length(self): return self.right_distal_length_val
    @right_distal_length.setter
    def right_distal_length(self, v): self.right_distal_length_val = v

    @property
    def ns_distal_length(self):    return self.ns_distal_length_val
    @ns_distal_length.setter
    def ns_distal_length(self, v): self.ns_distal_length_val = v


def hom2vect_local(m: np.ndarray) -> np.ndarray:
    """Return the (3,) translation part of a 4x4 matrix."""
    return m[:3, 3]

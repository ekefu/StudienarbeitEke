"""
initials_for_robotics.py
------------------------
Top-level simulation launcher.  Port of initials_for_robotics.m

Run this file directly:
    python initials_for_robotics.py

Toggle the demo flags below to select which simulation to run.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from transforms import X_AXIS, Y_AXIS, Z_AXIS
from human_hand   import HumanHand
from barrett_hand import BarrettHand, CONTROL_MODE, CLOSE_SIM_MODE
from hhand_control import (hhand_control_step,
                            SYNERGIC_CLOSE, SINGLE_INDEX_CLOSE)
from bhand_control import bhand_control_step, angle2encoder
from mapper_5_to_3 import mapper_5_to_3, CORNER_COORDINATES


# ── Demo mode switches (set exactly one to True) ──────────────────────────────
hhand_fwd_demo_mode  = False   # closing demo of the human hand
bhand_fwd_demo_mode  = False   # closing demo of the barrett hand
bhand_rwl_demo_mode  = False   # backward kinematics demo
teleopera_demo_mode  = True    # full teleoperation demo  ← default
mapper_optimise_mode = False   # mapper parameter optimisation


# ── Shared globals ────────────────────────────────────────────────────────────
LENGTH_RESOLUTION   = 5
WITH_MEMORY         = True
SIM_ANGLE_START     = 40
SIM_ANGLE_END       = 85


# ─────────────────────────────────────────────────────────────────────────────
def run_hhand_fwd_demo():
    """Port of the hhand_fwd_demo block in initials_for_robotics.m"""
    print("Running hhand_fwd_demo …")
    hhand = HumanHand(LENGTH_RESOLUTION, WITH_MEMORY)
    angle_index = 15

    fig = plt.figure()
    ax  = fig.add_subplot(111, projection='3d')
    plt.ion()
    plt.show()

    for _ in range(300):
        angle_index = hhand_control_step(
            hhand, SYNERGIC_CLOSE, angle_index,
            sim_start=15, sim_end=115)
        ax.cla()
        ax.plot(hhand.palm_x, hhand.palm_y, hhand.palm_z, 'r')
        for f in [hhand.index, hhand.middle, hhand.ring,
                  hhand.little, hhand.thumb]:
            ax.plot(f.q2cart[:, 0], f.q2cart[:, 1], f.q2cart[:, 2])
        ax.set_xlim(-100, 100); ax.set_ylim(-100, 100); ax.set_zlim(-100, 100)
        ax.set_xlabel('dimensions in mm')
        plt.pause(0.02)
    plt.ioff()
    plt.show()


# ─────────────────────────────────────────────────────────────────────────────
def run_bhand_fwd_demo():
    """Port of bhand_fwd_demo.m"""
    print("Running bhand_fwd_demo …")
    bhand = BarrettHand(CLOSE_SIM_MODE, length_resolution=LENGTH_RESOLUTION)

    fig = plt.figure()
    ax  = fig.add_subplot(111, projection='3d')
    plt.ion()
    plt.show()

    for angle_index in range(bhand.angle_resolution):
        bhand_control_step(bhand, CLOSE_SIM_MODE, angle_index)
        ax.cla()
        ax.plot(bhand.palm_x, bhand.palm_y, bhand.palm_z, 'r')
        ax.plot(bhand.q2cart_left[:, 0],      bhand.q2cart_left[:, 1],      bhand.q2cart_left[:, 2])
        ax.plot(bhand.q2cart_right[:, 0],     bhand.q2cart_right[:, 1],     bhand.q2cart_right[:, 2])
        ax.plot(bhand.q2cart_nonspread[:, 0], bhand.q2cart_nonspread[:, 1], bhand.q2cart_nonspread[:, 2])
        ax.set_xlim(-100, 100); ax.set_ylim(-100, 100); ax.set_zlim(-100, 100)
        ax.set_xlabel('dimensions in mm')
        plt.pause(0.01)
    plt.ioff()
    plt.show()


# ─────────────────────────────────────────────────────────────────────────────
def run_bhand_rwl_demo():
    """Port of bhand_rwl_demo.m"""
    print("Running bhand_rwl_demo …")
    bhand = BarrettHand(CONTROL_MODE, length_resolution=LENGTH_RESOLUTION)

    max_jump_angle    = 5       # degrees
    trajectory_count  = 5
    trajectory_length = 500

    # Error arrays [trajectory, step, xyz]
    left_err  = np.zeros((trajectory_count, trajectory_length, 3))
    right_err = np.zeros((trajectory_count, trajectory_length, 3))
    ns_err    = np.zeros((trajectory_count, trajectory_length, 3))

    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    plt.ion(); plt.show()

    for ti in range(trajectory_count):
        rwl_out_sim = np.array([
            bhand.abduct_lower_R + (bhand.abduct_border - bhand.abduct_lower_R) / 2,
            bhand.media_lower_R,
            bhand.media_lower_R,
            bhand.media_lower_R,
        ])

        for it in range(trajectory_length):
            rwl_out = rwl_out_sim + (max_jump_angle * np.pi / 180) * (np.random.rand(4) - 0.5)
            rwl_out_sim = rwl_out.copy()

            motor_step = angle2encoder(bhand, rwl_out)
            bhand.ctrl_spread    = motor_step[0]
            bhand.ctrl_left_m    = motor_step[1]
            bhand.ctrl_right_m   = motor_step[2]
            bhand.ctrl_nonspread = motor_step[3]

            bhand_control_step(bhand, CONTROL_MODE, 0)

            left_x_test  = bhand.left_tip[X_AXIS - 1]
            left_y_test  = bhand.left_tip[Y_AXIS - 1]
            left_z_test  = bhand.left_tip[Z_AXIS - 1]
            right_x_test = bhand.right_tip[X_AXIS - 1]
            right_y_test = bhand.right_tip[Y_AXIS - 1]
            right_z_test = bhand.right_tip[Z_AXIS - 1]
            ns_x_test    = bhand.ns_tip[X_AXIS - 1]
            ns_z_test    = bhand.ns_tip[Z_AXIS - 1]

            desired_left = bhand.q2cart_nonspread.shape[0]  # last row index
            des_left  = bhand.q2cart_left[desired_left - 1, :]
            des_right = bhand.q2cart_right[desired_left - 1, :]
            des_ns    = bhand.q2cart_nonspread[desired_left - 1, :]

            # Backward kinematics
            simnet_input = np.array([
                left_x_test,  left_y_test,  left_z_test,
                right_x_test, right_y_test, right_z_test,
                ns_x_test,    0.0,          ns_z_test,
            ])
            rwl_out_net = bhand.predict_rwl(simnet_input)

            motor_step = angle2encoder(bhand, rwl_out_net)
            bhand.ctrl_spread    = motor_step[0]
            bhand.ctrl_left_m    = motor_step[1]
            bhand.ctrl_right_m   = motor_step[2]
            bhand.ctrl_nonspread = motor_step[3]
            bhand_control_step(bhand, CONTROL_MODE, 0)

            est_left  = bhand.q2cart_left[desired_left - 1, :]
            est_right = bhand.q2cart_right[desired_left - 1, :]
            est_ns    = bhand.q2cart_nonspread[desired_left - 1, :]

            left_err[ti, it, :]  = est_left  - des_left
            right_err[ti, it, :] = est_right - des_right
            ns_err[ti, it, :]    = est_ns    - des_ns

        # Visualise after each trajectory
        for ax_obj in axes.flat:
            ax_obj.cla()
        axes[0,0].bar([1,2,3], left_err[ti, -1, :], color='b')
        axes[0,0].set_title('Left finger error (x,y,z) mm')
        axes[0,1].bar([1,2,3], right_err[ti, -1, :], color='g')
        axes[0,1].set_title('Right finger error (x,y,z) mm')
        axes[0,2].bar([1,2,3], ns_err[ti, -1, :], color='r')
        axes[0,2].set_title('Nonspread error (x,y,z) mm')
        plt.pause(0.1)

    plt.ioff(); plt.show()
    plot_rwl_error_logs(left_err, right_err, ns_err)


# ─────────────────────────────────────────────────────────────────────────────
def plot_rwl_error_logs(left_err, right_err, ns_err):
    """Port of plot_rwl_error_logs.m"""
    traj_count, traj_len, _ = left_err.shape
    timeline = np.arange(1, traj_len + 1)

    fig, axes = plt.subplots(3, 3, figsize=(15, 10))
    labels = ['x_axis', 'y_axis', 'z_axis']
    colors = {'left': 'rx', 'right': 'b+', 'ns': 'g*'}

    for col, (err, label) in enumerate([(left_err, 'left'),
                                         (right_err, 'right'),
                                         (ns_err, 'nonspread')]):
        for row in range(3):
            ax = axes[row, col]
            for ti in range(traj_count):
                ax.plot(timeline, err[ti, :, row], colors[label.split()[0]
                    if label != 'nonspread' else 'ns'])
            ax.set_ylim(-60, 60)
            ax.set_xlim(0, traj_len)
            ax.grid(True)
            ax.set_xlabel('time')
            ax.set_ylabel(f'e_{{{label}}} [{labels[row]}] mm')
    plt.tight_layout()
    plt.savefig('rwl_error_logs.png', dpi=100)
    plt.show()


# ─────────────────────────────────────────────────────────────────────────────
def run_teleopera_mode(mapper_cfg=None, save_avi=False):
    """
    Full teleoperation simulation loop.
    Port of teleopera_mode.m

    Parameters
    ----------
    mapper_cfg : dict of mapper parameters; uses defaults if None
    save_avi   : save frames to 'barrett_hand.avi' (not implemented here)

    Returns
    -------
    loop_fitness : cumulative squared tip-position error (used by optimiser)
    """
    print("Running teleopera_mode …")

    if mapper_cfg is None:
        mapper_cfg = {
            'barrett': {
                'rotate_z': 0.0, 'rotate_y': 0.0, 'rotate_x': 0.0,
                'trans_x': 0.0,  'trans_y': 0.0,  'trans_z': 0.0,
                'scaler': 1.0,
            },
            'hhand': {
                'index_weight': 1.0, 'middle_weight': 1.0,
                'ring_weight':  1.0, 'little_weight': 1.0,
            },
        }

    from transforms import rotate as rot, trans as T_mat
    from barrett_hand import BarrettHand

    hhand = HumanHand(LENGTH_RESOLUTION, WITH_MEMORY)
    bhand = BarrettHand(CONTROL_MODE, mapper_cfg['barrett'],
                        length_resolution=LENGTH_RESOLUTION)

    demo_map_count = 40
    angle_index    = SIM_ANGLE_START
    loop_fitness   = 0.0

    # Build barrett alignment matrix
    def _make_alignment(bc):
        from transforms import rotate as R, trans as T
        A = (R(3, bc['rotate_z']) @ R(2, bc['rotate_y']) @ R(1, bc['rotate_x'])
             @ T(bc['trans_x'], bc['trans_y'], bc['trans_z']))
        scaling = bc['scaler'] * np.eye(4); scaling[3, 3] = 1.0
        return A @ scaling

    alignment_A = _make_alignment(mapper_cfg['barrett'])

    fig, (ax_h, ax_b) = plt.subplots(1, 2, figsize=(12, 5),
                                     subplot_kw={'projection': '3d'})
    plt.ion(); plt.show()

    for step in range(demo_map_count):
        # ── 1) Human hand forward kinematics ──────────────────────────────
        angle_index = hhand_control_step(
            hhand, SYNERGIC_CLOSE, angle_index,
            sim_start=SIM_ANGLE_START, sim_end=SIM_ANGLE_END)

        # ── 2) Map 5 tips → 3 bhand corners + network input ───────────────
        simnet_input, corner_A, corner_B, corner_C = mapper_5_to_3(
            hhand, mapper_cfg['hhand'], CORNER_COORDINATES)

        # ── 3) Backward kinematics ─────────────────────────────────────────
        rwl_out    = bhand.predict_rwl(simnet_input)
        motor_step = angle2encoder(bhand, rwl_out)
        bhand.ctrl_spread    = motor_step[0]
        bhand.ctrl_left_m    = motor_step[1]
        bhand.ctrl_right_m   = motor_step[2]
        bhand.ctrl_nonspread = motor_step[3]

        # ── 4) Barrett forward kinematics ─────────────────────────────────
        bhand_control_step(bhand, CONTROL_MODE, 0, alignment_A)

        # ── 5) Fitness (if optimisation is active) ─────────────────────────
        if mapper_optimise_mode:
            single_fit = (
                np.sum((corner_A - bhand.left_tip) ** 2)
                + np.sum((corner_B - bhand.right_tip) ** 2)
                + np.sum((corner_C - bhand.ns_tip) ** 2)
            ) / 9.0
            loop_fitness += single_fit

        # ── 6) Visualise ───────────────────────────────────────────────────
        ax_h.cla()
        ax_h.plot(hhand.palm_x, hhand.palm_y, hhand.palm_z, 'r')
        for f in [hhand.index, hhand.middle, hhand.ring,
                  hhand.little, hhand.thumb]:
            ax_h.plot(f.q2cart[:, 0], f.q2cart[:, 1], f.q2cart[:, 2])
        ax_h.set_xlim(-100, 100); ax_h.set_ylim(-100, 100); ax_h.set_zlim(-100, 100)
        ax_h.set_ylabel('dimensions in mm')
        ax_h.set_title(f'Human Hand  (step {step+1})')

        ax_b.cla()
        ax_b.plot(bhand.palm_x, bhand.palm_y, bhand.palm_z, 'r')
        ax_b.plot(bhand.q2cart_left[:, 0],      bhand.q2cart_left[:, 1],      bhand.q2cart_left[:, 2])
        ax_b.plot(bhand.q2cart_right[:, 0],     bhand.q2cart_right[:, 1],     bhand.q2cart_right[:, 2])
        ax_b.plot(bhand.q2cart_nonspread[:, 0], bhand.q2cart_nonspread[:, 1], bhand.q2cart_nonspread[:, 2])
        ax_b.set_xlim(-100, 100); ax_b.set_ylim(-100, 100); ax_b.set_zlim(-100, 100)
        ax_b.set_ylabel('dimensions in mm')
        ax_b.set_title('Barrett Hand')

        plt.pause(0.05)

    plt.ioff(); plt.show()
    return loop_fitness


# ─────────────────────────────────────────────────────────────────────────────
def run_mapper_optimisation():
    """
    Mutation-selection optimisation of mapper parameters.
    Port of mapper_optimisation.m
    """
    print("Running mapper_optimisation (1500 iterations) …")

    best_fitness = float('inf')
    best_cfg = {
        'barrett': {
            'rotate_z': -0.1, 'rotate_y': 0.1, 'rotate_x': 0.0,
            'trans_x': 10.0,  'trans_y': 0.0,  'trans_z': 10.0,
            'scaler': 1.0,
        },
        'hhand': {
            'index_weight': 1.0, 'middle_weight': 1.0,
            'ring_weight':  1.0, 'little_weight': 1.0,
        },
    }

    import copy

    for search_count in range(1500):
        cfg = copy.deepcopy(best_cfg)
        bc  = cfg['barrett']
        hc  = cfg['hhand']

        # Random perturbation
        bc['rotate_z'] += 0.01 * (np.random.rand() - 0.5)
        bc['rotate_y'] += 0.01 * (np.random.rand() - 0.5)
        bc['rotate_x'] += 0.01 * (np.random.rand() - 0.5)
        bc['trans_x']  += 2.0  * (np.random.rand() - 0.5)
        bc['trans_y']  += 2.0  * (np.random.rand() - 0.5)
        bc['trans_z']  += 2.0  * (np.random.rand() - 0.5)
        bc['scaler']   += 0.1  * (np.random.rand() - 0.5)
        hc['index_weight']  += 0.01 * (np.random.rand() - 0.5)
        hc['middle_weight'] += 0.01 * (np.random.rand() - 0.5)
        hc['ring_weight']   += 0.01 * (np.random.rand() - 0.5)
        hc['little_weight'] += 0.01 * (np.random.rand() - 0.5)

        fitness = run_teleopera_mode(mapper_cfg=cfg)

        if fitness < best_fitness:
            best_fitness = fitness
            best_cfg = cfg
            print(f"  [{search_count+1:4d}] New best fitness = {best_fitness:.4f}")

    import joblib
    joblib.dump(best_cfg, 'mapper_optimised.pkl')
    print(f"Optimisation done. Best fitness = {best_fitness:.4f}")
    print(f"Saved to 'mapper_optimised.pkl'")


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    if hhand_fwd_demo_mode:
        run_hhand_fwd_demo()

    if bhand_fwd_demo_mode:
        run_bhand_fwd_demo()

    if bhand_rwl_demo_mode:
        run_bhand_rwl_demo()

    if teleopera_demo_mode:
        run_teleopera_mode()

    if mapper_optimise_mode:
        run_mapper_optimisation()

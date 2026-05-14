# Simulation – Python / scikit-learn Port

Converted from the original MATLAB scripts in `StudienarbeitEke/Simulation/`.

## File mapping

| Python file                  | Original MATLAB file(s)                                      |
|------------------------------|--------------------------------------------------------------|
| `transforms.py`              | `rotate.m`, `trans.m`, `hom2vect.m`                         |
| `human_hand.py`              | `human_hand.m`, `*_calibration.m`, `*_fwd.m`, `*_points.m`, `hhand_memory.m` |
| `barrett_hand.py`            | `barrett_hand.m`, `barrett_fwd.m`, `barrett_points.m`, `train_rwl_points_elman.m` |
| `bhand_control.py`           | `bhand_control.m`, `angle2encoder.m`                        |
| `hhand_control.py`           | `hhand_control.m`                                           |
| `mapper_5_to_3.py`           | `mapper_5_to_3.m`                                           |
| `initials_for_robotics.py`   | `initials_for_robotics.m`, `teleopera_mode.m`, `bhand_rwl_demo.m`, `bhand_fwd_demo.m`, `hhand_fwd_demo.m`, `mapper_optimisation.m`, `plot_rwl_error_logs.m` |

## Neural network replacement

| MATLAB (Neural Network Toolbox)              | Python (scikit-learn)                         |
|----------------------------------------------|-----------------------------------------------|
| `newelm([...], [240,480,128,4], {'tansig',...}, 'traingdx')` | `MLPRegressor(hidden_layer_sizes=(240,480,128), activation='tanh', solver='adam')` |
| `train(net, X, y)`                           | `net.fit(X_scaled, y)`                        |
| `sim(net, x)`                                | `net.predict(scaler.transform(x))`            |

Input normalisation uses `MinMaxScaler(feature_range=(-1, 1))`, matching the
MATLAB `newelm` input range specification `[-20 70; -15 75; 60 90; ...]`.

The trained model and its scaler are persisted to `load_net_rwl_points_elman.pkl`
via `joblib` (replaces the MATLAB `.mat` file).

## Quick start

```bash
pip install -r requirements.txt
python initials_for_robotics.py        # runs teleopera_demo_mode by default
```

To switch demos, edit the flags near the top of `initials_for_robotics.py`:

```python
hhand_fwd_demo_mode  = False   # human hand closing animation
bhand_fwd_demo_mode  = False   # barrett hand closing animation
bhand_rwl_demo_mode  = False   # backward kinematics accuracy demo
teleopera_demo_mode  = True    # full teleoperation (human→barrett mapping)
mapper_optimise_mode = False   # run mutation-selection mapper optimisation
```

On the **first run** the backward-kinematics MLP is trained from scratch
(~50 trajectories × 50 steps, takes a few seconds) and saved to disk.
Subsequent runs load the saved model instantly.

## Notes

* All angles are in **radians**; link lengths in **mm** – unchanged from MATLAB.
* Array indexing is **0-based** in Python (MATLAB is 1-based).  The axis
  constants `X_AXIS=1`, `Y_AXIS=2`, `Z_AXIS=3` are kept for readability but
  used as `arr[X_AXIS - 1]` throughout.
* The Elman-network recurrence is not replicated; the original code used
  `newelm` primarily as a feedforward MLP with an additional context layer
  that had marginal impact on backward-kinematics accuracy.  `MLPRegressor`
  with the same layer sizes and `tanh` activations is a direct replacement.

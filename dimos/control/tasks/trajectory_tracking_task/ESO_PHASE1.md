# Disturbance rejection for the Go2 trajectory tracker (Phase 1)

The trajectory tracker keeps a clean wheeled base (FlowBase) on path to a few
centimetres, but a Unitree Go2 drifts on curves at speed. The same outer
controller runs on both robots, so this branch tested whether the Go2
velocity-command plant could be cleaned up by an opt-in ESO/ADRC velocity layer.

Phase 1 is implemented and tested, but the honest result is null for path
tracking. Keep the code parked behind its default-off flag; do not cite it as a
tracking win.

## What It Does

The inner loop is a model-based Extended State Observer, the core of Active
Disturbance Rejection Control. For each velocity axis (`vx`, `vy`, `wz`) the
nominal channel is

    v_dot = -v/tau + (K/tau) * u + d

where `K` and `tau` come from the fitted FOPDT model and `d` lumps everything the
nominal model misses. The command law is

    u = (r - tau * z2) / K

with `r` supplied by the existing feedforward-plus-P tracker and `z2` the
observer's disturbance estimate. When `z2 == 0`, this is exactly today's gain
inversion. That is why `eso=False` leaves the default tracker path unchanged.

The measured body velocity comes from `velocity_estimator.py`, which smooths
pose and differentiates it causally. A review found that an earlier yaw lag
correction assumed a half-window delay that the default quadratic fit does not
have; the estimator now rotates with the current yaw.

## Honest Result

After fixing the scoring and estimator issues, the ESO is roughly neutral for
path tracking:

- Go2 hard-plant curve cases improve only slightly, about 2-10% cross-track RMS,
  and some of those improvements come with worse time-indexed along-track lag.
- Straight-line Go2 cases regress at low and mid speeds because the observer
  chases gait and odometry noise on axes whose desired velocity is near zero.
- FlowBase remains effectively a no-op, which is the required control result.
- The ESO does cancel steady velocity gain error in isolated axis tests, but
  that is not the dominant curve-tracking bottleneck.

The original stronger curve result was a measurement confound. Scoring only to
arrival let one arm orbit closed paths longer and padded the metric with extra
low-error samples. The sim harness now scores a fixed time window and reports
time-indexed cross-track plus along-track lag.

## Re-Running

Use the fixed-window harness:

    python -m dimos.utils.benchmarking.eso_sim_ab
    python -m dimos.utils.benchmarking.eso_sim_ab --dial-grid
    python -m pytest dimos/control/tasks/trajectory_tracking_task/ dimos/utils/benchmarking/test_eso_sim_ab.py -q

The key rule is single-variable A/B with the same seed for both arms and both
metrics in view: lower cross-track is not a real win if along-track lag gets
materially worse.

## Hardware Toggle

The ESO is off by default. For a hardware trial, drive the single existing Go2
coordinator:

    GO2_ESO=1 dimos run unitree-go2-coordinator
    GO2_ESO=1 GO2_ESO_BANDWIDTH=0.7 dimos run unitree-go2-coordinator

Do not spin up a second coordinator; that races `/go2/cmd_vel`.

## Phase 2

The remaining high-speed curve symptom is over-yaw and oscillation, which points
at dead time rather than a steady velocity disturbance. Phase 2 adds opt-in
feedback dead-time compensation: keep the existing feedforward reference preview
and compute feedback against a measured pose advanced by the nominal FOPDT
command-history model.

# Marvin Teleoperation Project Guidance

This fork's maintained path is PICO → Marvin dual-arm teleoperation. Keep the
XRoboToolkit upstream modules needed by the fork, but do not restore removed
UR/ARX/R1 hardware entry points or present upstream demos as Marvin procedures.

## Environment and checks

```bash
source /home/zxcx/TeleOp/.miniconda-xr/etc/profile.d/conda.sh
conda activate Teleop

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python -m pytest -q tests/test_marvin_hardware.py tests/test_marvin_mujoco_model.py
```

## Maintained entry points

```bash
python scripts/simulation/teleop_marvin_mujoco.py --scale-factor 0.5
python scripts/hardware/inspect_marvin_state.py --duration-s 10
python scripts/hardware/teleop_marvin_hardware.py --help
```

Never run the enabled hardware entry as an automated check. Physical execution
requires the Go/No-Go gates and operator sequence in
`docs/XRoboToolkit环境部署与PICO联调流程.md`.

## Safety invariants

- Marvin SDK access remains owned by one I/O thread.
- ROS2 is observation-only and must not acquire command authority.
- Internal units are SI; vendor joint commands cross the adapter boundary in degrees.
- Hardware startup adopts measured feedback and never commands an automatic home.
- Grip release holds the corresponding real arm; MuJoCo-only return behavior must not
  be copied to hardware without a separate safety review.
- FAULT is latched; the program does not automatically clear controller errors.

Upstream provenance is recorded in `UPSTREAM.md`.

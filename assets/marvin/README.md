# Marvin 14-DoF simulation assets

`marvin_dual.mujoco.xml` is the physics/rendering model used by the Marvin
simulation. `marvin_dual.urdf` is retained as a checksum-tracked geometry and
joint-contract reference; hardware FK/IK/Jacobian and the simulated target IK
now use the Marvin vendor kinematics SDK and do not load this URDF at runtime.

The common `world_to_base` transform applies a 180-degree yaw about world Z so
the simulated robot faces the PICO operator. The same rigid transform is applied
to the runtime MJCF roots and target markers; vendor arm kinematics are unchanged.

## Model provenance

- Arm links, joint transforms, axes, limits, masses and inertias come unchanged
  from the imported vendor models `Marvin M6S-Lite-L/R-CCS-680-V1.0` recorded
  in `marvin_dual.manifest.json`.
- `base_to_left_arm` and `base_to_right_arm`, the support dimensions, and the
  gripper transforms now follow the exact model loaded by
  `DEMO_PYTHON/showcase_pln_multi_segment_linear_two_arms_classes.py sim`:
  `DEMO_PYTHON/striding_doc/marvin_m6s_lite_dual_ccs_680.xml`. The base mesh
  centers are 120 mm apart and the two Joint1 axes are 437.2 mm apart.
- The Demo coordinates are rotated 180 degrees around world Z in this project so
  the robot faces the PICO operator. This changes only the shared world frame;
  the relative arm and gripper transforms remain numerically identical to the
  Demo model.
- The base inertial values follow the same public MoveIt model. Base visual and
  collision geometry are omitted because the reference repository does not state
  asset redistribution terms.
- The vendor TCP marker meshes are non-watertight. They are replaced by 1 mm
  collision spheres, matching the existing single-arm MuJoCo conversion. The
  original fixed `JointTCP_L/R` transform (87 mm from Link7) is preserved
  exactly. The Demo planner does not configure an extra tool transform, so this
  SDK `FlangeTip` remains the Cartesian control frame used by Pico teleoperation.
- The four Demo gripper meshes are mounted at their exact zero-position poses in
  both URDF and MJCF. They are fixed, visual-only geometry: gripper actuation and
  gripper collision are not part of the current 14-DoF controller. The identified
  tool payload below remains the dynamics source, avoiding double-counting the
  gripper mass already identified on the physical arms.
- `left_tool_payload` and `right_tool_payload` use the active `tool-1`
  identification in `TJArm/tools_cfg.json` (SHA-256
  `3b67eb4a39e5f4ac3bd370fe87581c2fadee85c29b5a8591c96eb2d9a499f535`).
  Mass is in kg, COM offsets are converted from flange-relative mm to m, and
  inertia is in kg·m². The identified left principal moments
  `[0.001, 0.016, 0.002]` violate the rigid-body triangle inequality; the model
  uses their nearest Euclidean projection
  `[0.005333333333, 0.011666666665, 0.006333333333]`. This regularization and
  the tool identity must be confirmed again after any physical tool change.

Confirm redistribution rights for the vendor meshes before publishing this
directory outside the project.

## Rebuild

This directory is an imported, checksum-tracked simulation snapshot. The source
generator is intentionally not duplicated into this repository. Rebuild only
from the two vendor URDFs and the Demo MJCF/gripper URDF recorded in the manifest,
then refresh and review every source/output/mesh SHA-256 before replacing this
baseline.

## Validate

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_marvin_mujoco_model.py
```

The checks cover the 14-DoF joint/actuator contract, finite limits, payload
dynamics, Demo-derived 120 mm/437.2 mm spacing, the unchanged 87 mm flange TCP,
exact gripper mesh checksums, vendor-FK alignment, and model loading in MuJoCo.

Validated in this workspace with MuJoCo 3.12.0 and the CCS-680 vendor kinematics
SDK. The runtime MJCF adds two raw-target mocap bodies, two limited-command mocap
bodies, and two flange payload bodies while preserving the 14-DoF contract:

```text
nq=14, nv=14, nu=14, nbody=21, nsite=2, nmesh=20
```

## Canonical joint order

```text
Joint1_L Joint2_L Joint3_L Joint4_L Joint5_L Joint6_L Joint7_L
Joint1_R Joint2_R Joint3_R Joint4_R Joint5_R Joint6_R Joint7_R
```

This is the URDF/WBC order only. Do not assume that it matches MarvinSDK A/B
index, direction, or encoder zero. Those mappings require a read-only hardware
check followed by single-joint motion tests.

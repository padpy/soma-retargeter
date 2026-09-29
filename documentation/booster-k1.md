# Booster K1

`booster_k1` retargets SOMA motion to the Booster K1: 22 revolute joints (2 head, 4 per arm, 6 per leg), no waist, no wrists and no hands, about 0.95 m tall.
- **Description and meshes:** `K1_22dof.urdf` and its meshes, unchanged from [booster_assets](https://github.com/BoosterRobotics/booster_assets) (BSD-3-Clause, `licenses/booster-LICENSE.txt`).
- **Structure:** the package follows the `booster_t1` layout. Its tuning starts from `booster_t1`'s weights.

## Mapping

| SOMA joint | K1 body | Notes |
|---|---|---|
| `Hips` | `trunk` | K1's root; there is no separate pelvis or waist |
| `Chest` | (none) | no waist joint, so the trunk is both pelvis and chest |
| `Head` | `aahead_pitch_link` | |
| `LeftArm` / `RightArm` | `left/right_shoulder_roll_link` | shoulder, as `AL2`/`AR2` on T1 |
| `LeftForeArm` / `RightForeArm` | `left/right_elbow_yaw_link` | origin at the elbow bend (K1's `elbow_pitch` is an upper-arm twist near the shoulder) |
| `LeftHand` / `RightHand` | (none) | no wrist or hand |
| `LeftLeg` / `RightLeg` | `left/right_hip_roll_link` | as `Hip_Roll_*` on T1 |
| `LeftShin` / `RightShin` | `left/right_knee_pitch_link` | |
| `LeftFoot` / `RightFoot` | `left/right_ankle_roll_link` | terminal foot body |
| toes | (none) | no toe links |

Weights come from the `booster_t1` rows with the same role. The smooth joint-limit masks mirror T1's on the shoulder pitch, shoulder roll and upper-arm twist. The roll mask is one-sided, so the hanging-arm pose, which sits near K1's roll limit, is not pushed outward.

## Calibration

The scaler was produced headlessly with `app/tools/robot_calibration_headless.py`. It uses the same math as the Robot Configurator's **Save Configs**, and it reproduces the bundled `booster_t1` scaler exactly from that robot's stored pose and scale.

```bash
uv run python app/tools/robot_calibration_headless.py --robot booster_k1 \
    --pose soma_retargeter/assets/robotics/booster/k1/configs/booster_k1_zero_pose.json \
    --scale pelvis --write
```

- **Reference pose** (`configs/booster_k1_zero_pose.json`): this matches the SOMA zero pose.
  - Upper arms hang 9.5° out from vertical and 2.8° forward: shoulder roll ∓1.405 rad, shoulder pitch −0.05 rad.
  - Elbows are bent 90° with the forearms pointing forward: elbow bend ∓π/2.
  - Legs are straight and the head is level.
- **Scale:** `pelvis`, K1 root height / SOMA hips height = 0.5465. This is the same method as `booster_t1`. The bounding-box method gives 0.540.
- **Proportions:** K1's hips (±0.096 m) and shoulders (±0.145 m) are wider than a scaled human's (±0.055 and ±0.089 m). The per-joint offsets absorb this.

## Validation

The 10 bundled BVH motions were retargeted to `booster_k1` and, as the reference, to `booster_t1` with identical converter settings. Both were scored with `app/tools/validate_retarget.py`:

```bash
uv run python app/bvh_to_csv_converter.py --config local-configs/validate_booster_k1.json   # import assets/motions/bvh
uv run python app/tools/validate_retarget.py --robot booster_k1 --csv-dir output/validate/booster_k1
```

Each cell reads K1 / T1:
- **limit:** share of frames with a joint within 0.02 rad of a limit.
- **pen:** deepest foot-collision bounding-box corner below the ground, in cm.
- **skate:** mean horizontal foot speed while the foot's lowest point is within 1.5 cm of the ground, in cm/s.
- **pops:** share of frames with any joint faster than 15 rad/s.
- **root:** K1 root height range, in m.

| Motion | s | limit | pen cm | skate cm/s | pops | root m |
|---|---|---|---|---|---|---|
| `Neutral_throw_ball_001__A057` | 2.9 | 0.74 / 0.66 | 2.2 / 3.5 | 8 / 7 | 0.052 / 0.052 | 0.50–0.54 |
| `Neutral_walk_forward_002__A057` | 9.1 | 0.03 / 0.12 | 5.1 / 5.5 | 53 / 64 | 0.003 / 0.008 | 0.50–0.54 |
| `big_light_one_hand_pick_up_front_low_R_005__A508` | 7.6 | 0.00 / 0.33 | 1.0 / 1.0 | 0 / 0 | 0.000 / 0.000 | 0.54–0.54 |
| `body_stretch_1_004__A069` | 6.2 | 0.70 / 0.30 | 1.3 / 1.8 | 3 / 3 | 0.000 / 0.000 | 0.52–0.54 |
| `dance_basic_chaines_180_R_001__A306` | 4.1 | 0.53 / 0.61 | 1.3 / 3.7 | 50 / 69 | 0.000 / 0.012 | 0.47–0.57 |
| `dance_hiphop_shuffle_square_R_fast_002__A318` | 5.8 | 0.36 / 0.24 | 1.8 / 4.1 | 24 / 32 | 0.019 / 0.045 | 0.46–0.58 |
| `high_jump_R_001__A277` | 2.7 | 0.29 / 0.32 | 2.6 / 4.8 | 6 / 5 | 0.053 / 0.041 | 0.43–0.78 |
| `item_pick_up_standing_R_001__A410` | 3.5 | 0.13 / 0.00 | 0.7 / 0.9 | 0 / 0 | 0.000 / 0.000 | 0.33–0.55 |
| `small_light_one_hand_pick_up_front_low_002__A507` | 7.2 | 0.00 / 0.62 | 0.5 / 0.6 | 0 / 0 | 0.000 / 0.000 | 0.44–0.55 |
| `wave_R_001__A428` | 4.4 | 0.22 / 0.51 | 2.9 / 4.0 | 8 / 8 | 0.000 / 0.000 | 0.53–0.54 |

Findings:
- **K1 is comparable to T1 across the set.**
  - Foot penetration is lower on 9 of 10 clips, and equal within 0.1 cm on the tenth.
  - Foot sliding is lower or equal on 8 of 10 clips. It is higher on the ball throw (8 vs 7 cm/s) and the high jump (6 vs 5 cm/s).
  - Joint-speed spikes are similar.
  - Both robots show the same slight crouch-into-floor on standing clips. The penetration and skate measures use bounding boxes, so they overstate absolute values; compare them between robots, not against zero.
- **Joint limits are where K1 is tighter.**
  - The ankles roll only ±0.345 rad, so they saturate in stretches and dance steps.
  - The elbow bend saturates in throws.
  - Head pitch saturates when looking up or down.
  - T1 saturates the same kinds of joints on the same clips. No joint is locked across whole motions: no shoulder-roll pinning, and no mirrored or twisted limbs.
- **Left/right correctness:** in `wave_R`, the right forearm tip rises to 0.85 m while the left stays at 0.51 m. The `_R` pick-ups reach with the right hand.
- **Facing:** over 5.4 m of `Neutral_walk_forward`, the trunk faces within a median 6° (90th percentile 11°) of the travel direction.
- **Visual check:** frames rendered in MuJoCo from the official `K1_22dof.xml` show a normal walking gait, a squat pick-up, a jump with tucked legs, overhead stretches and a right-hand wave.

## Limitations

- **No chest target:** without a waist, human torso twist and lean go only through the trunk's root orientation.
- **No hand target:** only the forearm link's orientation constrains the elbow bend. Hand reaches are approximate.
- **Narrow ankle roll:** ankle roll (±20°) and inward hip roll (0.375 rad) are much narrower than a human's. Wide stances and ankle-heavy moves saturate.
- **Kinematic only:** scaling shrinks distances but not timing, so jumps and flight phases become more demanding for a 0.95 m robot. Dynamic feasibility is not checked here.
- **Tuning scope:** weights were not re-optimized for K1. `app/tools/ik_weight_optimizer.py` is the next step if a motion class needs it.

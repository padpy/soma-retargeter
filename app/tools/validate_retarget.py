# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Kinematic quality metrics for retargeted robot CSVs (no viewer needed).

For every CSV in a folder, replays the motion through the robot's Newton model (forward
kinematics only) and reports, per clip:

- ``limit_frames``: fraction of frames with any joint within ``--limit-margin`` rad of a limit,
  and the joints that sit there most often;
- ``penetration_cm`` / ``penetration_frames``: deepest point of the foot collision geometry
  below the ground plane, and the fraction of frames more than 1 cm below it;
- ``skate_cm_s``: horizontal speed of a foot while its lowest point is within
  ``--contact-height`` of the ground (mean and 95th percentile over contact frames);
- ``airborne_frames``: fraction of frames with both feet above ``--contact-height``;
- ``root_height_m``: min / mean / max height of the root body;
- ``max_joint_speed`` and ``pop_frames``: peak joint speed (rad/s) and the fraction of frames
  where any joint exceeds ``--pop-speed`` rad/s.

Foot bodies come from the robot's post-processing config (the last effector of each
``ik_limbs`` entry). Run the same motions through two robots to compare a new target with
an established one:

    uv run python app/tools/validate_retarget.py --robot booster_k1 \\
        --csv-dir output/validate/booster_k1 --out output/validate/booster_k1_metrics.json
"""

import argparse
import json
import pathlib

import newton
import numpy as np
import warp as wp

import soma_retargeter.io.csv as csv_utils
import soma_retargeter.pipelines.utils as pipeline_utils
import soma_retargeter.utils.newton_utils as newton_utils
from soma_retargeter.io.utils import load_json
from soma_retargeter.robotics.robot_registry import register_robots_path
from soma_retargeter.robotics.robot_registry import registry as robot_registry


def foot_bodies(name: str) -> list[str]:
    entry = robot_registry.get(name)
    manifest_dir = pathlib.Path(entry.manifest_dir)
    retargeter = load_json(manifest_dir / entry.retarget_configs["soma"])
    pp = load_json(manifest_dir / retargeter["post_processing"]["robot_config"])
    stab = pp["limb_stabilizer"]
    names = list(stab["effectors"])
    return [names[limb["effectors"][-1]] for limb in stab["ik_limbs"].values()]


def shape_corners(model, body: int) -> np.ndarray:
    """(K, 3) collision-AABB corners of every shape on ``body``, in the body frame."""
    lower = model.shape_collision_aabb_lower.numpy()
    upper = model.shape_collision_aabb_upper.numpy()
    bodies = model.shape_body.numpy()
    tfs = model.shape_transform.numpy()
    points = []
    for i in np.flatnonzero(bodies == body):
        tf = wp.transform(*tfs[i])
        for cx in (lower[i][0], upper[i][0]):
            for cy in (lower[i][1], upper[i][1]):
                for cz in (lower[i][2], upper[i][2]):
                    points.append(np.array(wp.transform_point(tf, wp.vec3(cx, cy, cz))))
    if not points:
        raise ValueError(f"body {body} has no collision shapes")
    return np.stack(points)


def quat_rotate_np(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate (K, 3) points by an (x, y, z, w) quaternion."""
    xyz, w = q[:3], q[3]
    t = 2.0 * np.cross(xyz, v)
    return v + w * t + np.cross(xyz, t)


def clip_metrics(model, state, csv_path: pathlib.Path, feet: list[str], args) -> dict:
    header = csv_path.read_text().splitlines()[0].split(",")
    joint_cols = header[7:]
    buffer = csv_utils.load_csv(str(csv_path), args.fps)
    n = buffer.num_frames
    names = [newton_utils.get_name_from_label(label) for label in model.joint_label]
    starts = model.joint_q_start.numpy()
    q_index = {names[i]: int(starts[i]) for i in range(len(names))}
    # Limits are per DOF (a free joint has 7 coordinates but 6 DOFs), so index them separately.
    dof_starts = model.joint_qd_start.numpy()
    dof_index = {names[i]: int(dof_starts[i]) for i in range(len(names))}
    missing = [c for c in joint_cols if c not in q_index]
    if missing:
        raise KeyError(f"{csv_path.name}: joints not in model: {missing}")
    lo = model.joint_limit_lower.numpy()
    hi = model.joint_limit_upper.numpy()
    body_index = {newton_utils.get_name_from_label(label): i for i, label in enumerate(model.body_label)}
    foot_ids = [body_index[f] for f in feet]
    corners = [shape_corners(model, b) for b in foot_ids]
    # Floating root: the free joint's 7 coordinates (x, y, z, qx, qy, qz, qw) and its child body.
    free = int(np.flatnonzero(model.joint_type.numpy() == newton.JointType.FREE)[0])
    root_start = int(starts[free])
    root_body = int(model.joint_child.numpy()[free])

    joint_q = model.joint_q.numpy().copy()
    cols = np.array([q_index[c] for c in joint_cols])
    q_all = np.zeros((n, len(cols)))
    foot_low = np.zeros((n, len(feet)))
    foot_xy = np.zeros((n, len(feet), 2))
    root_z = np.zeros(n)
    for f in range(n):
        data = buffer.get_data(f)
        joint_q[root_start : root_start + 7] = data[0:7]
        joint_q[cols] = data[7:]
        q_all[f] = data[7:]
        model.joint_q.assign(joint_q)
        newton.eval_fk(model, model.joint_q, model.joint_qd, state)
        body_q = state.body_q.numpy()
        root_z[f] = body_q[root_body][2]
        for k, (b, pts) in enumerate(zip(foot_ids, corners, strict=True)):
            p, q = body_q[b][:3], body_q[b][3:7]
            world = quat_rotate_np(q, pts) + p
            foot_low[f, k] = world[:, 2].min()
            foot_xy[f, k] = p[:2]

    dofs = np.array([dof_index[c] for c in joint_cols])
    limits_lo = lo[dofs]
    limits_hi = hi[dofs]
    at_limit = (q_all <= limits_lo + args.limit_margin) | (q_all >= limits_hi - args.limit_margin)
    per_joint = at_limit.mean(axis=0)
    top = np.argsort(-per_joint)[:3]
    dt = 1.0 / args.fps
    speed = np.abs(np.diff(q_all, axis=0)) / dt if n > 1 else np.zeros((1, len(cols)))
    contact = foot_low < args.contact_height
    foot_speed = np.linalg.norm(np.diff(foot_xy, axis=0), axis=2) / dt if n > 1 else np.zeros((1, len(feet)))
    skate = foot_speed[contact[1:]] * 100.0 if n > 1 else np.zeros(0)
    return {
        "frames": n,
        "seconds": n / args.fps,
        "limit_frames": float(at_limit.any(axis=1).mean()),
        "limit_top_joints": {joint_cols[i]: round(float(per_joint[i]), 3) for i in top if per_joint[i] > 0},
        "penetration_cm": float(max(0.0, -foot_low.min()) * 100.0),
        "penetration_frames": float((foot_low.min(axis=1) < -0.01).mean()),
        "skate_cm_s": {
            "mean": float(skate.mean()) if skate.size else 0.0,
            "p95": float(np.percentile(skate, 95)) if skate.size else 0.0,
        },
        "contact_frames": float(contact.any(axis=1).mean()),
        "airborne_frames": float((~contact).all(axis=1).mean()),
        "root_height_m": {"min": float(root_z.min()), "mean": float(root_z.mean()), "max": float(root_z.max())},
        "max_joint_speed": float(speed.max()),
        "pop_frames": float((speed > args.pop_speed).any(axis=1).mean()),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot", required=True)
    ap.add_argument("--csv-dir", type=pathlib.Path, required=True)
    ap.add_argument("--out", type=pathlib.Path)
    ap.add_argument("--fps", type=float, default=120.0)
    ap.add_argument("--limit-margin", type=float, default=0.02, help="rad")
    ap.add_argument("--contact-height", type=float, default=0.015, help="m")
    ap.add_argument("--pop-speed", type=float, default=15.0, help="rad/s")
    ap.add_argument("--extra-robot-paths", nargs="*", default=[])
    args = ap.parse_args()
    for path in args.extra_robot_paths:
        register_robots_path(path)

    builder = newton.ModelBuilder()
    builder.add_ground_plane()
    builder.add_world(pipeline_utils.create_robot_builder(args.robot), wp.transform_identity())
    model = builder.finalize()
    state = model.state()
    feet = foot_bodies(args.robot)

    results = {}
    for csv_path in sorted(args.csv_dir.rglob("*.csv")):
        results[csv_path.stem] = clip_metrics(model, state, csv_path, feet, args)
        r = results[csv_path.stem]
        print(
            f"{csv_path.stem[:48]:48s} limit {r['limit_frames']:.2f}  pen {r['penetration_cm']:5.2f}cm "
            f"({r['penetration_frames']:.2f})  skate {r['skate_cm_s']['mean']:5.1f}/{r['skate_cm_s']['p95']:5.1f}cm/s  "
            f"air {r['airborne_frames']:.2f}  root {r['root_height_m']['min']:.2f}-{r['root_height_m']['max']:.2f}m  "
            f"vmax {r['max_joint_speed']:5.1f}  pops {r['pop_frames']:.3f}"
        )
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps({"robot": args.robot, "feet": feet, "clips": results}, indent=2))
        print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

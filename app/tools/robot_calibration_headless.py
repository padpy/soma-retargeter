# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Headless scaler calibration: the Robot Configurator's Save Configs step without a window.

Computes the SOMA-to-robot scaler config (``human_scale_ratio``, ``reference_joint_q``,
``human_joint_offsets``) for a registered robot from:

- the robot's reference (zero) pose, given as named joint angles in a JSON file;
- the link mapping already in the robot's retargeter config (``ik_match_table``);
- a scale method: ``bbox`` (robot collision bounding-box height / SOMA skeleton height,
  the configurator's default), ``pelvis`` (robot Hips-link height / SOMA Hips height), or
  an explicit number.

The math is the configurator's (``app/tools/robot_config_generator.py``): the SOMA zero
pose is loaded with the same MuJoCo facing conversion and 90 degree Z rotation, and each
offset is ``_compute_local_offset(soma_joint_tx, robot_body_tx)``. Useful on machines with
no display, and for recording exactly how a calibration was produced.

    uv run python app/tools/robot_calibration_headless.py --robot booster_k1 \\
        --pose soma_retargeter/assets/robotics/booster/k1/configs/booster_k1_zero_pose.json \\
        --scale bbox --write
"""

import argparse
import json
import pathlib

import newton
import numpy as np
import warp as wp

import soma_retargeter.io.bvh as bvh_utils
import soma_retargeter.pipelines.utils as pipeline_utils
import soma_retargeter.utils.newton_utils as newton_utils
from soma_retargeter.animation.skeleton import SkeletonInstance
from soma_retargeter.io.utils import load_json, save_json
from soma_retargeter.robotics.robot_registry import register_robots_path
from soma_retargeter.robotics.robot_registry import registry as robot_registry
from soma_retargeter.utils.space_conversion_utils import FacingDirectionType, SpaceConverter


def compute_local_offset(source_tx: wp.transform, target_tx: wp.transform) -> wp.transform:
    """Same as robot_config_generator._compute_local_offset."""
    offset_q = wp.normalize(wp.mul(wp.quat_inverse(source_tx.q), target_tx.q))
    offset_p = wp.quat_rotate(wp.quat_inverse(target_tx.q), target_tx.p - source_tx.p)
    return wp.transform(offset_p, offset_q)


def load_soma_instance() -> tuple[object, SkeletonInstance]:
    """SOMA zero pose placed exactly as the configurator places it."""
    converter = SpaceConverter(FacingDirectionType.MUJOCO)
    skeleton, animation = bvh_utils.load_bvh(
        pipeline_utils.get_source_zero_pose_asset_path(pipeline_utils.SourceType.SOMA))
    z_rot = wp.transform(wp.vec3(0, 0, 0), wp.quat_from_axis_angle(wp.vec3(0, 0, 1), wp.radians(90.0)))
    xform = wp.mul(z_rot, converter.transform(wp.transform_identity()))
    instance = SkeletonInstance(skeleton, wp.vec3(1.0, 1.0, 1.0), xform)
    instance.set_local_transforms(animation.get_local_transforms(0))
    return skeleton, instance


def build_robot(name: str):
    builder = newton.ModelBuilder()
    builder.add_ground_plane()
    builder.add_world(pipeline_utils.create_robot_builder(name), wp.transform_identity())
    model = builder.finalize()
    return model, model.state()


def set_named_pose(model, joint_q: np.ndarray, pose: dict[str, float]) -> np.ndarray:
    """Write named 1-DoF joint angles (radians) into a copy of ``joint_q``."""
    q = joint_q.copy()
    starts = model.joint_q_start.numpy()
    names = [newton_utils.get_name_from_label(label) for label in model.joint_label]
    unknown = sorted(set(pose) - set(names))
    if unknown:
        raise KeyError(f"pose names joints the model does not have: {unknown}")
    for i, name in enumerate(names):
        if name in pose:
            q[int(starts[i])] = float(pose[name])
    return q


def robot_bbox_height(model, state) -> float:
    """World Z extent of all robot collision shapes (configurator's bounding-box scale)."""
    lower = model.shape_collision_aabb_lower.numpy()
    upper = model.shape_collision_aabb_upper.numpy()
    bodies = model.shape_body.numpy()
    shape_tfs = model.shape_transform.numpy()
    body_q = state.body_q.numpy()
    zs = []
    for i in range(model.shape_count):
        b = int(bodies[i])
        if b < 0:
            continue
        world = wp.transform_multiply(wp.transform(*body_q[b]), wp.transform(*shape_tfs[i]))
        for cx in (lower[i][0], upper[i][0]):
            for cy in (lower[i][1], upper[i][1]):
                for cz in (lower[i][2], upper[i][2]):
                    zs.append(float(wp.transform_point(world, wp.vec3(cx, cy, cz))[2]))
    return max(zs) - min(zs)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot", required=True)
    ap.add_argument("--pose", type=pathlib.Path, help="JSON {joint_name: radians}; default: URDF zero")
    ap.add_argument("--pose-from-scaler", action="store_true",
                    help="use reference_joint_q from the robot's existing scaler config (for re-checking it)")
    ap.add_argument("--print-soma", action="store_true", help="print every SOMA zero-pose joint position")
    ap.add_argument("--scale", default="bbox", help="bbox, pelvis, or a number")
    ap.add_argument("--extra-robot-paths", nargs="*", default=[])
    ap.add_argument("--write", action="store_true", help="write the scaler config the retargeter config names")
    args = ap.parse_args()

    for path in args.extra_robot_paths:
        register_robots_path(path)
    entry = robot_registry.get(args.robot)
    manifest_dir = pathlib.Path(entry.manifest_dir)
    retargeter = load_json(manifest_dir / entry.retarget_configs["soma"])
    table = retargeter["ik_match_table"]

    model, state = build_robot(args.robot)
    if args.pose_from_scaler:
        existing = load_json(manifest_dir / retargeter["human_robot_scaler_config"])
        joint_q = np.array(existing["reference_joint_q"], dtype=np.float32)
        if joint_q.shape != model.joint_q.numpy().shape:
            raise ValueError(f"reference_joint_q has {joint_q.shape}, model {model.joint_q.numpy().shape}")
    else:
        pose = json.loads(args.pose.read_text()) if args.pose else {}
        pose = {k: v for k, v in pose.items() if not k.startswith("_")}
        joint_q = set_named_pose(model, model.joint_q.numpy(), pose)
    model.joint_q.assign(joint_q)
    newton.eval_fk(model, model.joint_q, model.joint_qd, state)
    body_q = state.body_q.numpy()
    body_index = {newton_utils.get_name_from_label(label): i for i, label in enumerate(model.body_label)}

    skeleton, soma = load_soma_instance()
    soma_unit = soma.compute_global_transforms(scale=1.0)
    if args.scale == "bbox":
        z = soma_unit[:, 2]
        ratio = robot_bbox_height(model, state) / float(z.max() - z.min())
    elif args.scale == "pelvis":
        hips_link = table["Hips"]["t_body"]
        ratio = float(body_q[body_index[hips_link]][2]) / float(soma_unit[skeleton.joint_index("Hips")][2])
    else:
        ratio = float(args.scale)
    soma.scale = ratio
    soma_tx = soma.compute_global_transforms()
    if args.print_soma:
        for j in range(skeleton.num_joints):
            print(f"  SOMA {skeleton.joint_name(j):16s} unit-scale pos {np.round(soma_unit[j][:3], 4)}")

    offsets: dict[str, list[list[float]]] = {}
    print(f"[calibration] {args.robot}: scale {args.scale} -> human_scale_ratio {ratio:.6f}")
    print(f"{'SOMA joint':14s} {'robot body':26s} {'SOMA pos (scaled)':30s} {'robot pos':30s} dist")
    for joint, row in table.items():
        link = row["t_body"]
        j = skeleton.joint_index(joint)
        if j < 0 or link not in body_index:
            raise KeyError(f"unknown mapping {joint} -> {link}")
        s_tx = wp.transform(*soma_tx[j])
        r_tx = wp.transform(*body_q[body_index[link]])
        off = compute_local_offset(s_tx, r_tx)
        offsets[joint] = [
            [round(float(off.p[0]), 4), round(float(off.p[1]), 4), round(float(off.p[2]), 4)],
            [round(float(off.q[0]), 4), round(float(off.q[1]), 4), round(float(off.q[2]), 4), round(float(off.q[3]), 4)],
        ]
        sp = np.array(s_tx.p)
        rp = np.array(r_tx.p)
        print(f"{joint:14s} {link:26s} {np.round(sp, 3)!s:30s} {np.round(rp, 3)!s:30s} {np.linalg.norm(sp - rp):.3f}")

    scaler = {
        "human_type": "soma",
        "human_root": "Hips",
        "robot_type": args.robot,
        "robot_root": table["Hips"]["t_body"],
        "human_scale_ratio": float(ratio),
        "reference_joint_q": [round(float(v), 10) for v in joint_q],
        "human_joint_offsets": offsets,
    }
    out = manifest_dir / retargeter["human_robot_scaler_config"]
    if args.write:
        save_json(out, scaler)
        print(f"[calibration] wrote {out}")
    else:
        print(json.dumps(scaler, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

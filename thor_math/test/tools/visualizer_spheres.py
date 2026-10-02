#!/usr/bin/env python3
"""Replay a trajectory with Meshcat markers on UR10 joints and intermediate frames."""

from __future__ import annotations

import argparse
from pathlib import Path
import time


def parse_args() -> argparse.Namespace:
    test_dir = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path, help="trajectory CSV to replay")
    parser.add_argument(
        "--urdf",
        type=Path,
        default=test_dir / "data" / "ur10" / "ur10_with_intermediates.urdf",
    )
    parser.add_argument("--period", type=float, default=0.002)
    parser.add_argument("--loop", action="store_true", help="repeat continuously")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.period < 0:
        raise SystemExit("--period must be non-negative")
    if not args.trajectory.is_file():
        raise SystemExit(f"trajectory not found: {args.trajectory}")
    if not args.urdf.is_file():
        raise SystemExit(f"URDF not found: {args.urdf}")

    try:
        import meshcat.geometry as geometry
        import numpy as np
        import pandas as pd
        import pinocchio as pin
        from meshcat.visualizer import Visualizer
        from pinocchio.visualize import MeshcatVisualizer
    except ImportError as error:
        raise SystemExit(f"missing visualization dependency: {error}") from error

    trajectory = pd.read_csv(args.trajectory)
    joint_columns = [column for column in trajectory if column.startswith("q")]
    required = {"ph_x", "ph_y", "ph_z"}
    missing = required.difference(trajectory.columns)
    if not joint_columns or missing:
        raise SystemExit(f"invalid trajectory columns; missing: {sorted(missing)}")

    model, _, _ = pin.buildModelsFromUrdf(str(args.urdf))
    visualizer = MeshcatVisualizer(model)
    visualizer.viewer = Visualizer()
    visualizer.viewer.open()

    frame_ids = [
        index
        for index, frame in enumerate(model.frames)
        if frame.type == pin.FrameType.JOINT or "intermediate" in frame.name
    ]
    sphere = geometry.Sphere(0.03)
    material = geometry.MeshPhongMaterial(color=0xFFFF00)
    for frame_id in frame_ids:
        visualizer.viewer[f"sphere/frame_{frame_id}"].set_object(sphere, material)
    visualizer.viewer["human"].set_object(
        geometry.Sphere(0.05), geometry.MeshLambertMaterial(color=0xFF0000)
    )

    data = model.createData()
    while True:
        for _, row in trajectory.iterrows():
            configuration = row[joint_columns].to_numpy(float)
            pin.forwardKinematics(model, data, configuration)
            pin.updateFramePlacements(model, data)
            for frame_id in frame_ids:
                visualizer.viewer[f"sphere/frame_{frame_id}"].set_transform(
                    data.oMf[frame_id].homogeneous
                )
            human = row[["ph_x", "ph_y", "ph_z"]].to_numpy(float)
            visualizer.viewer["human"].set_transform(
                pin.SE3(np.eye(3), human).homogeneous
            )
            time.sleep(args.period)
        if not args.loop:
            break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Insert fixed intermediate frames before every movable joint in a URDF."""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def divide_xyz(xyz: str, parts: int) -> tuple[float, float, float]:
    values = tuple(float(value) for value in xyz.split())
    if len(values) != 3:
        raise ValueError(f"Expected three XYZ values, got: {xyz!r}")
    return tuple(value / parts for value in values)  # type: ignore[return-value]


def format_xyz(xyz: tuple[float, float, float]) -> str:
    return " ".join(f"{value:.12g}" for value in xyz)


def add_intermediate_frames(root: ET.Element, frames_per_joint: int) -> None:
    """Mutate *root*, inserting equally spaced frames before movable joints."""
    if frames_per_joint < 1:
        raise ValueError("frames_per_joint must be at least 1")

    for joint in list(root.findall("joint")):
        if joint.get("type") == "fixed":
            continue

        name = joint.get("name")
        parent_element = joint.find("parent")
        child_element = joint.find("child")
        if not name or parent_element is None or child_element is None:
            raise ValueError("Every movable joint must have a name, parent and child")

        parent = parent_element.get("link")
        child = child_element.get("link")
        if not parent or not child:
            raise ValueError(f"Joint {name!r} has an invalid parent or child")

        origin = joint.find("origin")
        xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
        rpy = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
        step_xyz = format_xyz(divide_xyz(xyz, frames_per_joint + 1))
        current_parent = parent

        for index in range(1, frames_per_joint + 1):
            intermediate = f"{name}_intermediate_{index}"
            root.append(ET.Element("link", name=intermediate))

            fixed_joint = ET.Element(
                "joint", name=f"{name}_mid_{index}", type="fixed"
            )
            ET.SubElement(fixed_joint, "parent", link=current_parent)
            ET.SubElement(fixed_joint, "child", link=intermediate)
            ET.SubElement(fixed_joint, "origin", xyz=step_xyz, rpy="0 0 0")
            root.append(fixed_joint)
            current_parent = intermediate

        parent_element.set("link", current_parent)
        if origin is None:
            origin = ET.SubElement(joint, "origin")
        origin.set("xyz", step_xyz)
        origin.set("rpy", rpy)


def transform_urdf(input_path: Path, output_path: Path, frames: int) -> None:
    tree = ET.parse(input_path)
    add_intermediate_frames(tree.getroot(), frames)
    # ElementTree.indent was introduced in Python 3.9; Ubuntu 20.04 uses 3.8.
    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def parse_args() -> argparse.Namespace:
    test_dir = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input",
        nargs="?",
        type=Path,
        default=test_dir / "data" / "ur10" / "ur10.urdf",
        help="input URDF (default: test/data/ur10/ur10.urdf)",
    )
    parser.add_argument(
        "output",
        nargs="?",
        type=Path,
        default=test_dir / "data" / "ur10" / "ur10_with_intermediates.urdf",
        help="output URDF",
    )
    parser.add_argument("--frames", type=int, default=1)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        transform_urdf(args.input, args.output, args.frames)
    except (OSError, ET.ParseError, ValueError) as error:
        print(f"error: {error}")
        return 1
    print(f"Wrote {args.output} with {args.frames} intermediate frame(s) per joint")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Generate ArUco SVGs and engraved or multi-material Gridfinity STLs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import struct
import subprocess

import numpy as np

from aruco_marker import ArucoMarker


SCRIPT_DIR = Path(__file__).resolve().parent


def stl_bounds(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Read bounding coordinates from a binary or ASCII STL."""
    data = path.read_bytes()
    if len(data) >= 84:
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        if 84 + triangle_count * 50 == len(data):
            record = np.dtype(
                [
                    ("normal", "<f4", 3),
                    ("vertices", "<f4", (3, 3)),
                    ("attribute", "<u2"),
                ]
            )
            triangles = np.frombuffer(data, dtype=record, count=triangle_count, offset=84)
            vertices = triangles["vertices"].reshape(-1, 3).astype(float)
            return vertices.min(axis=0), vertices.max(axis=0)

    text = data.decode("ascii", errors="strict")
    vertices = np.asarray(
        [
            [float(x), float(y), float(z)]
            for x, y, z in re.findall(
                r"\bvertex\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)",
                text,
            )
        ],
        dtype=float,
    )
    if vertices.size == 0:
        raise ValueError(f"No STL vertices found in {path}")
    return vertices.min(axis=0), vertices.max(axis=0)


def infer_scale(minimum: np.ndarray, maximum: np.ndarray) -> float:
    """Convert metre-exported CAD STLs to millimetres when appropriate."""
    largest_dimension = float(np.max(maximum - minimum))
    return 1000.0 if largest_dimension < 1.0 else 1.0


def build_marker(
    marker_id: int,
    args: argparse.Namespace,
    base_scales: np.ndarray,
    top_z: float,
    center_x: float,
    center_y: float,
) -> list[Path]:
    marker = ArucoMarker(
        marker_id=marker_id,
        size_mm=args.size_mm,
        dictionary=args.dictionary,
        border_bits=args.border_bits,
    )

    svg_path = args.svg_dir / f"{args.dictionary.lower()}_id_{marker_id:03d}.svg"
    marker.to_svg(svg_path)
    recognized_from_svg = marker.recognize_id(svg_path)
    recognized_from_array = marker.recognize_id(marker.as_array())
    if recognized_from_svg != marker_id or recognized_from_array != marker_id:
        raise RuntimeError(
            f"Recognition check failed for marker {marker_id}: "
            f"SVG={recognized_from_svg}, array={recognized_from_array}"
        )

    definitions = {
        "base_file": json.dumps(str(args.base_stl.resolve())),
        "base_scale_x": f"{base_scales[0]:.12g}",
        "base_scale_y": f"{base_scales[1]:.12g}",
        "base_scale_z": f"{base_scales[2]:.12g}",
        "marker_bits": marker.to_openscad_array(),
        "marker_size_mm": f"{args.size_mm:.12g}",
        "marker_center_x": f"{center_x:.12g}",
        "marker_center_y": f"{center_y:.12g}",
        "top_z": f"{top_z:.12g}",
        "engrave_depth_mm": f"{args.engrave_depth_mm:.12g}",
        "mmu_depth_mm": f"{args.layer_height_mm * args.mmu_layers:.12g}",
    }

    if args.mmu:
        outputs = []
        for color in ("white", "black"):
            output_path = (
                args.output_dir / f"gridfinity_aruco_{marker_id:03d}_{color}.stl"
            )
            run_openscad(
                output_path,
                definitions | {"output_part": json.dumps(f"mmu_{color}")},
                args,
            )
            outputs.append(output_path)
        print(
            f"ID {marker_id}: {svg_path.name} -> "
            f"{outputs[0].name} + {outputs[1].name} "
            f"({args.mmu_layers} MMU layers, "
            f"+{args.fit_expansion_mm:g} mm/side fit, "
            f"recognized SVG and array)"
        )
        return outputs

    output_path = args.output_dir / f"gridfinity_aruco_{marker_id:03d}.stl"
    run_openscad(
        output_path,
        definitions | {"output_part": json.dumps("engraved")},
        args,
    )
    print(
        f"ID {marker_id}: {svg_path.name} -> {output_path.name} "
        f"(recognized SVG and array)"
    )
    return [output_path]


def run_openscad(
    output_path: Path, definitions: dict[str, str], args: argparse.Namespace
) -> None:
    command = [args.openscad, "-o", str(output_path)]
    for name, value in definitions.items():
        command.extend(["-D", f"{name}={value}"])
    command.append(str(args.scad_file))
    subprocess.run(command, cwd=SCRIPT_DIR, check=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate ArUco SVGs and engraved or multi-material Gridfinity STL files."
        )
    )
    parser.add_argument("--ids", nargs="+", type=int, default=list(range(5)))
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--size-mm", type=float, default=32.0)
    parser.add_argument("--border-bits", type=int, default=1)
    parser.add_argument("--engrave-depth-mm", type=float, default=0.6)
    parser.add_argument(
        "--mmu",
        action="store_true",
        help="Export aligned white-base and black-marker STLs for multi-material printing.",
    )
    parser.add_argument(
        "--layer-height-mm",
        type=float,
        default=0.2,
        help="Print layer height used to calculate MMU inlay depth (default: 0.2).",
    )
    parser.add_argument(
        "--mmu-layers",
        type=int,
        default=2,
        help="Number of top layers containing both colors (default: 2).",
    )
    parser.add_argument("--base-stl", type=Path, default=SCRIPT_DIR / "aruco_base.stl")
    parser.add_argument(
        "--scad-file", type=Path, default=SCRIPT_DIR / "aruco_gridfinity.scad"
    )
    parser.add_argument("--svg-dir", type=Path, default=SCRIPT_DIR / "img")
    parser.add_argument("--output-dir", type=Path, default=SCRIPT_DIR / "output")
    parser.add_argument("--openscad", default="openscad")
    parser.add_argument(
        "--base-scale",
        type=float,
        help="Override STL scale. By default metre-sized STLs are scaled by 1000.",
    )
    parser.add_argument(
        "--fit-expansion-mm",
        type=float,
        default=0.0,
        help=(
            "Enlarge only the base mesh in X/Y by this amount on each side; "
            "the marker and Z dimensions remain unchanged (default: 0)."
        ),
    )
    parser.add_argument("--top-z", type=float, help="Override the scaled top Z in mm.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.layer_height_mm <= 0:
        raise ValueError("--layer-height-mm must be greater than zero")
    if args.mmu_layers < 1:
        raise ValueError("--mmu-layers must be at least 1")
    if args.fit_expansion_mm < 0:
        raise ValueError("--fit-expansion-mm cannot be negative")
    args.base_stl = args.base_stl.resolve()
    args.scad_file = args.scad_file.resolve()
    args.svg_dir = args.svg_dir.resolve()
    args.output_dir = args.output_dir.resolve()
    args.svg_dir.mkdir(parents=True, exist_ok=True)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    minimum, maximum = stl_bounds(args.base_stl)
    unit_scale = (
        args.base_scale
        if args.base_scale is not None
        else infer_scale(minimum, maximum)
    )
    unexpanded_size = (maximum - minimum) * unit_scale
    base_scales = np.array(
        [
            unit_scale
            * (unexpanded_size[0] + 2 * args.fit_expansion_mm)
            / unexpanded_size[0],
            unit_scale
            * (unexpanded_size[1] + 2 * args.fit_expansion_mm)
            / unexpanded_size[1],
            unit_scale,
        ]
    )
    scaled_minimum = minimum * base_scales
    scaled_maximum = maximum * base_scales
    top_z = args.top_z if args.top_z is not None else float(scaled_maximum[2])
    center_x = float((scaled_minimum[0] + scaled_maximum[0]) / 2)
    center_y = float((scaled_minimum[1] + scaled_maximum[1]) / 2)

    print(
        f"Base bounds (mm): min={scaled_minimum.tolist()}, "
        f"max={scaled_maximum.tolist()}, scales={base_scales.tolist()}, "
        f"fit expansion={args.fit_expansion_mm:g} mm/side"
    )
    for marker_id in args.ids:
        build_marker(marker_id, args, base_scales, top_z, center_x, center_y)


if __name__ == "__main__":
    main()

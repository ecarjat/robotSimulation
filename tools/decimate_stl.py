#!/usr/bin/env python3
"""Decimate STL meshes to a target triangle count using Open3D."""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Decimate STL files in a directory tree to a target number of faces."
    )
    parser.add_argument(
        "--input-dir",
        default="myRobot/assets",
        help="Root directory to search for STL files (default: myRobot/assets)",
    )
    parser.add_argument(
        "--target-faces",
        type=int,
        default=5000,
        help="Target number of faces (triangles) per mesh (default: 5000)",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help=(
            "Directory to write decimated files, preserving relative paths. "
            "If omitted, writes alongside originals with a suffix."
        ),
    )
    parser.add_argument(
        "--suffix",
        default="_decimated",
        help="Suffix to append when writing next to originals (default: _decimated)",
    )
    parser.add_argument(
        "--in-place",
        action="store_true",
        help="Overwrite original files instead of writing new ones.",
    )
    return parser.parse_args()


def _iter_stl_files(root: Path) -> list[Path]:
    return [p for p in root.rglob("*") if p.is_file() and p.suffix.lower() == ".stl"]


def main() -> int:
    args = _parse_args()

    input_dir = Path(args.input_dir)
    if not input_dir.exists() or not input_dir.is_dir():
        print(f"Input directory not found: {input_dir}", file=sys.stderr)
        return 2

    if args.target_faces <= 0:
        print("--target-faces must be a positive integer.", file=sys.stderr)
        return 2

    if args.in_place and args.output_dir is not None:
        print("Use either --in-place or --output-dir, not both.", file=sys.stderr)
        return 2

    output_dir = Path(args.output_dir) if args.output_dir else None
    if output_dir:
        output_dir.mkdir(parents=True, exist_ok=True)

    try:
        import open3d as o3d
    except Exception as exc:  # pragma: no cover - runtime import guard
        print(
            "Failed to import open3d. Activate your .venv and retry.",
            file=sys.stderr,
        )
        print(str(exc), file=sys.stderr)
        return 3

    stl_files = _iter_stl_files(input_dir)
    if not stl_files:
        print(f"No STL files found under {input_dir}")
        return 0

    processed = 0
    skipped = 0
    copied = 0

    for stl_path in stl_files:
        try:
            mesh = o3d.io.read_triangle_mesh(str(stl_path))
        except Exception as exc:
            print(f"Failed to read {stl_path}: {exc}", file=sys.stderr)
            skipped += 1
            continue

        if not mesh.has_triangles():
            print(f"Skipping (no triangles): {stl_path}")
            skipped += 1
            continue

        tri_count = len(mesh.triangles)

        if tri_count <= args.target_faces:
            if args.in_place:
                print(f"Already <= target ({tri_count}): {stl_path}")
                skipped += 1
                continue

            if output_dir:
                out_path = output_dir / stl_path.relative_to(input_dir)
                out_path.parent.mkdir(parents=True, exist_ok=True)
            else:
                out_path = stl_path.with_name(f"{stl_path.stem}{args.suffix}{stl_path.suffix}")

            try:
                shutil.copy2(stl_path, out_path)
                print(f"Copied (already <= target): {stl_path} -> {out_path}")
                copied += 1
            except Exception as exc:
                print(f"Failed to copy {stl_path}: {exc}", file=sys.stderr)
                skipped += 1
            continue

        mesh_simplified = mesh.simplify_quadric_decimation(
            target_number_of_triangles=args.target_faces
        )
        mesh_simplified.compute_vertex_normals()

        if output_dir:
            out_path = output_dir / stl_path.relative_to(input_dir)
            out_path.parent.mkdir(parents=True, exist_ok=True)
        elif args.in_place:
            out_path = stl_path
        else:
            out_path = stl_path.with_name(f"{stl_path.stem}{args.suffix}{stl_path.suffix}")

        success = o3d.io.write_triangle_mesh(str(out_path), mesh_simplified)
        if not success:
            print(f"Failed to write {out_path}", file=sys.stderr)
            skipped += 1
            continue

        print(
            f"Decimated {stl_path} ({tri_count} -> {len(mesh_simplified.triangles)}): {out_path}"
        )
        processed += 1

    print(
        "Done. "
        f"Processed: {processed}, Copied: {copied}, Skipped: {skipped}, Total: {len(stl_files)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

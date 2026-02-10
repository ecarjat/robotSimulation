#!/usr/bin/env python3
import argparse
import csv
import sys
import math
import os
import xml.etree.ElementTree as ET

try:
    import numpy as np
except Exception:
    np = None

# CAD world origin is H_L (left hip). Coordinates are in mm by default.
# This map defines each body's local frame origin expressed in CAD world coords (meters).
ORIGINS_M = {
    # torso frame origin in CAD: H_L is at (0, +0.155, 0) in torso frame
    # so torso origin in CAD = (0, -0.155, 0)
    "torso": (0.0, -0.155, 0.0),
    # battery is centered between left/right at same torso frame origin
    "battery": (0.0, -0.155, 0.0),

    # split torso boxes (origins relative to torso frame)
    "torso_L": (0.0, -0.155 + 0.095066, 0.0),
    "torso_R": (0.0, -0.155 - 0.095066, 0.0),

    # left leg chain
    "upper_L": (0.0, 0.0, 0.0),
    "lower_L": (0.39609, 0.0, 0.0),
    "wheel_L": (0.01613, 0.0, 0.0),
    "rod_L": (0.09893, 0.0, 0.11576),

    # right leg chain (offset -0.28 in Y from H_L, per CAD)
    "upper_R": (0.0, -0.28, 0.0),
    "lower_R": (0.39609, -0.28, 0.0),
    "wheel_R": (0.01613, -0.28, 0.0),
    "rod_R": (0.09893, -0.28, 0.11576),
}

# Some components may report CAD COM in their own local frame rather than
# the global CAD origin (H_L). Map those names to a MJCF body whose world
# origin should be used as the CAD origin offset for that row.
ORIGIN_BODY_OVERRIDE = {}


def parse_args():
    ap = argparse.ArgumentParser(
        description="Convert CAD world COM (H_L origin) to MuJoCo body-local inertial pos."
    )
    ap.add_argument(
        "--units",
        choices=["mm", "m"],
        default="mm",
        help="Input units for COM coordinates (default: mm)",
    )
    ap.add_argument(
        "--format",
        choices=["csv", "tsv"],
        default="csv",
        help="Input delimiter format (default: csv)",
    )
    ap.add_argument(
        "--header",
        action="store_true",
        help="Input has a header row",
    )
    ap.add_argument(
        "--xml",
        action="store_true",
        help="Output MuJoCo <inertial ...> snippets (default: CSV table)",
    )
    ap.add_argument(
        "--cad-values",
        action="store_true",
        help="Parse cad values.csv format (name,mass_g,com_x_mm,com_y_mm,com_z_mm,...)",
    )
    ap.add_argument(
        "--mjcf",
        help="MJCF file to derive body world origins (overrides ORIGINS_M / origin_* columns).",
    )
    ap.add_argument(
        "--origin-site",
        help="Site name in MJCF that corresponds to CAD world origin (e.g., H_L).",
    )
    ap.add_argument(
        "--mirror-right-y",
        action="store_true",
        help="Extra mirror for *_R bodies after CAD->MJ transform (legacy).",
    )
    ap.add_argument(
        "--mirror-all-y",
        action="store_true",
        help="Extra mirror for all bodies after CAD->MJ transform (legacy).",
    )
    ap.add_argument(
        "--mirror-left-to-right",
        action="store_true",
        help="Ignore *_R rows and generate *_R from *_L by mirroring Y (and Ixy/Iyz).",
    )
    ap.add_argument(
        "input",
        nargs="?",
        help="Input file (name,x,y,z). If omitted, reads stdin.",
    )
    return ap.parse_args()


def to_meters(val, units):
    return val / 1000.0 if units == "mm" else val


def cad_to_mj_vec(x, y, z):
    # Explicit CAD->MJCF axis mapping.
    # CAD: +X forward, +Y right (H_R at +Y), +Z up
    # MJCF: +X forward, +Y left, +Z up
    # Therefore: x' = x, y' = -y, z' = z
    return (x, -y, z)


def _normalize_name(raw):
    base = raw.split("::", 1)[0]
    base = base.split(":", 1)[0]
    return base.strip()


def _row_kind(raw):
    lower = raw.lower()
    if "::aggregate" in lower:
        return "aggregate"
    if "::component" in lower:
        return "component"
    return "unknown"


def _safe_float(val):
    try:
        return float(val)
    except Exception:
        return None


def _parse_pos(attr):
    if not attr:
        return (0.0, 0.0, 0.0)
    parts = attr.strip().split()
    if len(parts) != 3:
        return (0.0, 0.0, 0.0)
    return (float(parts[0]), float(parts[1]), float(parts[2]))


def _matmul3(a, b):
    out = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
    return out


def _matvec3(r, v):
    return (
        r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
        r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
        r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
    )


def _transpose3(r):
    return [
        [r[0][0], r[1][0], r[2][0]],
        [r[0][1], r[1][1], r[2][1]],
        [r[0][2], r[1][2], r[2][2]],
    ]


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def _quat_to_mat(qw, qx, qy, qz):
    # normalized quaternion to rotation matrix
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    if n == 0:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
    return [
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ]


def _parse_rotation(elem):
    # MJCF: quat="w x y z" or euler="x y z" (XYZ order)
    if elem is None:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    quat = elem.get("quat")
    if quat:
        parts = quat.strip().split()
        if len(parts) == 4:
            qw, qx, qy, qz = [float(p) for p in parts]
            return _quat_to_mat(qw, qx, qy, qz)
    euler = elem.get("euler")
    if euler:
        parts = euler.strip().split()
        if len(parts) == 3:
            ex, ey, ez = [float(p) for p in parts]
            return _matmul3(_matmul3(_rot_x(ex), _rot_y(ey)), _rot_z(ez))
    return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def _accumulate_body_positions(worldbody):
    # Returns dict: body_name -> (world position, world rotation)
    out = {}

    def walk(body, parent_pos):
        name = body.get("name")
        pos = _parse_pos(body.get("pos"))
        r_local = _parse_rotation(body)
        # parent_pos is in world; we need parent's rotation to rotate pos, but we
        # only store positions here, so this function should be replaced by a full pose walk.
        world_pos = (parent_pos[0] + pos[0],
                     parent_pos[1] + pos[1],
                     parent_pos[2] + pos[2])
        if name:
            out[name] = (world_pos, r_local)
        for child in body.findall("body"):
            walk(child, world_pos)

    for body in worldbody.findall("body"):
        walk(body, (0.0, 0.0, 0.0))
    return out


def _load_mjcf_origins(path):
    tree = ET.parse(path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        return {}
    return _accumulate_body_poses(worldbody)


def _accumulate_body_poses(worldbody):
    # Returns dict: body_name -> (world_pos, world_rot)
    out = {}

    def walk(body, parent_pos, parent_rot):
        name = body.get("name")
        pos = _parse_pos(body.get("pos"))
        r_local = _parse_rotation(body)
        world_pos = _matvec3(parent_rot, pos)
        world_pos = (parent_pos[0] + world_pos[0],
                     parent_pos[1] + world_pos[1],
                     parent_pos[2] + world_pos[2])
        world_rot = _matmul3(parent_rot, r_local)
        if name:
            out[name] = (world_pos, world_rot)
        for child in body.findall("body"):
            walk(child, world_pos, world_rot)

    for body in worldbody.findall("body"):
        walk(body, (0.0, 0.0, 0.0), [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    return out


def _accumulate_site_positions(worldbody):
    # Returns dict: site_name -> world position (meters)
    sites = {}

    body_poses = _accumulate_body_poses(worldbody)

    # Walk bodies again to place sites using their body's world pose.
    def walk(body):
        name = body.get("name")
        if name and name in body_poses:
            bpos, brot = body_poses[name]
        else:
            bpos, brot = (0.0, 0.0, 0.0), [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

        for site in body.findall("site"):
            sname = site.get("name")
            spos = _parse_pos(site.get("pos"))
            sw = _matvec3(brot, spos)
            if sname:
                sites[sname] = (bpos[0] + sw[0], bpos[1] + sw[1], bpos[2] + sw[2])

        for child in body.findall("body"):
            walk(child)

    for body in worldbody.findall("body"):
        walk(body)
    return sites


def _load_mjcf_site_positions(path):
    tree = ET.parse(path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        return {}
    return _accumulate_site_positions(worldbody)


def main():
    args = parse_args()
    if os.getenv("CONVERT_COM_DEBUG") == "1":
        print(f"DEBUG args.xml={args.xml}", file=sys.stderr)
    delim = "," if args.format == "csv" else "\t"

    fh = open(args.input, "r", newline="") if args.input else sys.stdin
    reader = csv.reader(fh, delimiter=delim)

    mjcf_origins = _load_mjcf_origins(args.mjcf) if args.mjcf else {}
    mjcf_sites = _load_mjcf_site_positions(args.mjcf) if args.mjcf else {}
    origin_offset_base = (0.0, 0.0, 0.0)
    if args.origin_site:
        if args.origin_site not in mjcf_sites:
            print(f"Origin site '{args.origin_site}' not found in MJCF.", file=sys.stderr)
        else:
            origin_offset_base = mjcf_sites[args.origin_site]

    # Auto-detect cad values format from header if present
    header_map = None
    if args.header:
        header = next(reader, None)
        if header:
            lower = [h.strip().lower() for h in header]
            header_map = {h: i for i, h in enumerate(lower)}
            if not args.cad_values and "mass_g" in lower and "com_x_mm" in lower:
                args.cad_values = True
            if "ixx_gmm2" in lower:
                # If inertia columns are present, force XML output.
                args.xml = True

    out = None
    if not args.xml:
        # If cad-values with inertia columns is provided, require --xml to avoid mixed output.
        if header_map and "ixx_gmm2" in header_map:
            print("Error: cad values input includes inertia; run with --xml.", file=sys.stderr)
            sys.exit(2)
        out = csv.writer(sys.stdout)
        out.writerow(["name", "x_local_m", "y_local_m", "z_local_m"])

    for row in reader:
        if not row or row[0].strip().startswith("#"):
            continue
        raw_name = row[0]
        name = _normalize_name(raw_name)
        row_kind = _row_kind(raw_name)
        if name not in ORIGINS_M:
            print(f"Unknown body name '{name}'. Known: {', '.join(ORIGINS_M.keys())}", file=sys.stderr)
            continue
        if args.mirror_left_to_right and name.endswith("_R"):
            # Skip right-side rows; they'll be generated from left-side rows.
            continue
        # Auto-detect cad values format if not explicitly set.
        cad_values = args.cad_values
        if not cad_values and len(row) >= 11:
            try:
                maybe_mass = float(row[1])
                maybe_comx = float(row[2])
                # Heuristic: mass in grams typically > 1, COM in mm typically within +-1e5.
                if abs(maybe_mass) > 1.0 and abs(maybe_comx) < 1.0e5:
                    cad_values = True
            except Exception:
                cad_values = False

        if cad_values:
            # name, mass_g, com_x_mm, com_y_mm, com_z_mm, ...
            if len(row) < 5:
                print(f"Row for '{name}' missing COM columns; need at least 5 columns.", file=sys.stderr)
                continue
            if header_map and "com_x_mm" in header_map:
                x_raw = _safe_float(row[header_map.get("com_x_mm")])
                y_raw = _safe_float(row[header_map.get("com_y_mm")])
                z_raw = _safe_float(row[header_map.get("com_z_mm")])
            else:
                x_raw = _safe_float(row[2])
                y_raw = _safe_float(row[3])
                z_raw = _safe_float(row[4])
        else:
            if len(row) < 4:
                print(f"Row for '{name}' missing COM columns; need at least 4 columns.", file=sys.stderr)
                continue
            x_raw = _safe_float(row[1])
            y_raw = _safe_float(row[2])
            z_raw = _safe_float(row[3])
        if x_raw is None or y_raw is None or z_raw is None:
            # likely a header row
            continue

        # CAD world COM (meters).
        x_cad = to_meters(x_raw, args.units)
        y_cad = to_meters(y_raw, args.units)
        z_cad = to_meters(z_raw, args.units)

        # CAD world origin for this component (meters), if provided.
        origin_override = None
        if header_map and "origin_x_mm" in header_map:
            ox_raw = _safe_float(row[header_map.get("origin_x_mm")])
            oy_raw = _safe_float(row[header_map.get("origin_y_mm")])
            oz_raw = _safe_float(row[header_map.get("origin_z_mm")])
            if ox_raw is not None and oy_raw is not None and oz_raw is not None:
                ox_cad = to_meters(ox_raw, args.units)
                oy_cad = to_meters(oy_raw, args.units)
                oz_cad = to_meters(oz_raw, args.units)
                origin_override = (ox_cad, oy_cad, oz_cad)

        # Convert CAD world -> MJCF world for COM and origin.
        x_mj, y_mj, z_mj = cad_to_mj_vec(x_cad, y_cad, z_cad)
        if origin_override is not None:
            ox_mj, oy_mj, oz_mj = cad_to_mj_vec(*origin_override)
        else:
            # fall back to legacy per-body origin map (in CAD world coords).
            ox_mj, oy_mj, oz_mj = cad_to_mj_vec(*ORIGINS_M[name])

        # Optional extra mirror for Y axis (legacy).
        if args.mirror_all_y:
            y_mj = -y_mj
            oy_mj = -oy_mj
        elif args.mirror_right_y and name.endswith("_R"):
            y_mj = -y_mj
            oy_mj = -oy_mj

        # Apply global origin offset (CAD origin -> MJCF world), with optional per-body override.
        origin_offset = origin_offset_base
        override_body = ORIGIN_BODY_OVERRIDE.get(name)
        if override_body and args.mjcf and override_body in mjcf_origins:
            origin_offset = mjcf_origins[override_body][0]
        x_mj += origin_offset[0]
        y_mj += origin_offset[1]
        z_mj += origin_offset[2]
        ox_mj += origin_offset[0]
        oy_mj += origin_offset[1]
        oz_mj += origin_offset[2]

        # Local COM in MJCF world axes.
        dx = x_mj - ox_mj
        dy = y_mj - oy_mj
        dz = z_mj - oz_mj

        # Convert to body-local frame if MJCF body has rotation.
        if args.mjcf and name in mjcf_origins:
            r_world = mjcf_origins[name][1]
            r_t = _transpose3(r_world)
            lx, ly, lz = _matvec3(r_t, (dx, dy, dz))
        else:
            lx, ly, lz = dx, dy, dz

        def emit_one(out_name, lx, ly, lz, mass_kg, ixx, iyy, izz, ixy, ixz, iyz):
            if args.xml:
                # Always emit fullinertia as requested.
                print(
                    f'<!-- {out_name} -->\\n'
                    f'<inertial pos="{lx:.6f} {ly:.6f} {lz:.6f}" mass="{mass_kg:.6f}" '
                    f'fullinertia="{ixx:.6f} {iyy:.6f} {izz:.6f} {ixy:.6f} {ixz:.6f} {iyz:.6f}"/>'
                )
            else:
                if out is None:
                    out = csv.writer(sys.stdout)
                    out.writerow(["name", "x_local_m", "y_local_m", "z_local_m"])
                out.writerow([out_name, f"{lx:.6f}", f"{ly:.6f}", f"{lz:.6f}"])

        if args.xml:
            # Expecting input columns:
            # name, mass_g, com_x_mm, com_y_mm, com_z_mm, Ixx_gmm2, Iyy_gmm2, Izz_gmm2, Ixy_gmm2, Ixz_gmm2, Iyz_gmm2
            if len(row) < 11:
                print(f"Row for '{name}' missing inertia columns; need 11 columns.", file=sys.stderr)
                continue
            if header_map and "mass_g" in header_map:
                mass_g = float(row[header_map.get("mass_g")])
                ixx = float(row[header_map.get("ixx_gmm2")])
                iyy = float(row[header_map.get("iyy_gmm2")])
                izz = float(row[header_map.get("izz_gmm2")])
                ixy = float(row[header_map.get("ixy_gmm2")])
                ixz = float(row[header_map.get("ixz_gmm2")])
                iyz = float(row[header_map.get("iyz_gmm2")])
            else:
                mass_g = float(row[1])
                ixx = float(row[5])
                iyy = float(row[6])
                izz = float(row[7])
                ixy = float(row[8])
                ixz = float(row[9])
                iyz = float(row[10])

        if args.mirror_all_y:
            # Mirror across Y for all bodies: flip off-diagonals with Y.
            ixy = -ixy
            iyz = -iyz
        elif args.mirror_right_y and name.endswith("_R"):
            # Mirror across Y for right-side bodies: flip off-diagonals with Y.
            ixy = -ixy
            iyz = -iyz

        mass_kg = mass_g / 1000.0
        # g*mm^2 -> kg*m^2 (1e-9)
        ixx *= 1.0e-9
        iyy *= 1.0e-9
        izz *= 1.0e-9
        ixy *= 1.0e-9
        ixz *= 1.0e-9
        iyz *= 1.0e-9

        emit_one(name, lx, ly, lz, mass_kg, ixx, iyy, izz, ixy, ixz, iyz)

        if args.mirror_left_to_right and name.endswith("_L"):
            # Generate right-side body from left-side by mirroring Y.
            name_r = name[:-2] + "_R"
            # Mirror world COM Y.
            mx, my, mz = x, -y, z
            # Recompute local position for right-side body.
            if args.mjcf and name_r in mjcf_origins:
                (ox_r, oy_r, oz_r), r_world_r = mjcf_origins[name_r]
                dx_r = mx - ox_r
                dy_r = my - oy_r
                dz_r = mz - oz_r
                r_t_r = _transpose3(r_world_r)
                lx_r, ly_r, lz_r = _matvec3(r_t_r, (dx_r, dy_r, dz_r))
            elif header_map and "origin_x_mm" in header_map and use_origin_cols:
                # Use per-row origin if provided; mirror Y for origin as well.
                ox_raw = _safe_float(row[header_map.get("origin_x_mm")])
                oy_raw = _safe_float(row[header_map.get("origin_y_mm")])
                oz_raw = _safe_float(row[header_map.get("origin_z_mm")])
                if ox_raw is not None and oy_raw is not None and oz_raw is not None:
                    ox = to_meters(ox_raw, args.units)
                    oy = -to_meters(oy_raw, args.units)
                    oz = to_meters(oz_raw, args.units)
                    # Apply the same origin offset used for COM conversion.
                    origin_offset_r = origin_offset_base
                    override_body_r = ORIGIN_BODY_OVERRIDE.get(name_r)
                    if override_body_r and args.mjcf and override_body_r in mjcf_origins:
                        origin_offset_r = mjcf_origins[override_body_r][0]
                    ox += origin_offset_r[0]
                    oy += origin_offset_r[1]
                    oz += origin_offset_r[2]
                    lx_r = mx - ox
                    ly_r = my - oy
                    lz_r = mz - oz
                else:
                    ox, oy, oz = ORIGINS_M[name_r]
                    lx_r = mx - ox
                    ly_r = my - oy
                    lz_r = mz - oz
            else:
                ox, oy, oz = ORIGINS_M[name_r]
                lx_r = mx - ox
                ly_r = my - oy
                lz_r = mz - oz

            # Mirror inertia: flip off-diagonals with Y.
            ixy_r = -ixy
            iyz_r = -iyz
            emit_one(name_r, lx_r, ly_r, lz_r, mass_kg, ixx, iyy, izz, ixy_r, ixz, iyz_r)


if __name__ == "__main__":
    main()

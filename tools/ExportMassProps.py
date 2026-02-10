import adsk.core, adsk.fusion, adsk.cam, traceback, csv

# Components to aggregate (container components made of sub-components).
# Edit this list to match your MJCF body/component names.
TARGET_COMPONENTS = [
    'torso_L', 'torso_R',
    'upper_L', 'upper_R',
    'lower_L', 'lower_R',
    'rod_L', 'rod_R',
    'wheel_L', 'wheel_R',
]

def _matrix_to_rotation_and_translation(mtx):
    # adsk.core.Matrix3D has getCell(row, col) for a 4x4 matrix
    r = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            r[i][j] = mtx.getCell(i, j)
    t = (mtx.getCell(0, 3), mtx.getCell(1, 3), mtx.getCell(2, 3))
    return r, t


def _transform_point(r, t, p):
    x = r[0][0] * p[0] + r[0][1] * p[1] + r[0][2] * p[2] + t[0]
    y = r[1][0] * p[0] + r[1][1] * p[1] + r[1][2] * p[2] + t[1]
    z = r[2][0] * p[0] + r[2][1] * p[1] + r[2][2] * p[2] + t[2]
    return (x, y, z)


def _collect_descendant_occurrences(occ):
    out = []
    stack = [occ]
    while stack:
        cur = stack.pop()
        out.append(cur)
        for child in cur.childOccurrences:
            stack.append(child)
    return out


def _rotate_tensor(r, I):
    # I_world = R * I * R^T, with R 3x3 and I 3x3
    ri = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            ri[i][j] = r[i][0] * I[0][j] + r[i][1] * I[1][j] + r[i][2] * I[2][j]
    i_world = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            i_world[i][j] = ri[i][0] * r[j][0] + ri[i][1] * r[j][1] + ri[i][2] * r[j][2]
    return i_world


def _shift_inertia_com_to_origin(I_com, mass_kg, d_cm):
    # Parallel axis theorem: I_origin = I_com + m * (d^2*I - d*d^T)
    dx, dy, dz = d_cm
    d2 = dx * dx + dy * dy + dz * dz
    m = mass_kg
    shift = [
        [m * (d2 - dx * dx), -m * (dx * dy),     -m * (dx * dz)],
        [-m * (dy * dx),     m * (d2 - dy * dy), -m * (dy * dz)],
        [-m * (dz * dx),     -m * (dz * dy),     m * (d2 - dz * dz)],
    ]
    return [
        [I_com[0][0] + shift[0][0], I_com[0][1] + shift[0][1], I_com[0][2] + shift[0][2]],
        [I_com[1][0] + shift[1][0], I_com[1][1] + shift[1][1], I_com[1][2] + shift[1][2]],
        [I_com[2][0] + shift[2][0], I_com[2][1] + shift[2][1], I_com[2][2] + shift[2][2]],
    ]


def _shift_inertia_origin_to_com(I_origin, mass_kg, d_cm):
    # Inverse of parallel axis: I_com = I_origin - m * (d^2*I - d*d^T)
    dx, dy, dz = d_cm
    d2 = dx * dx + dy * dy + dz * dz
    m = mass_kg
    shift = [
        [m * (d2 - dx * dx), -m * (dx * dy),     -m * (dx * dz)],
        [-m * (dy * dx),     m * (d2 - dy * dy), -m * (dy * dz)],
        [-m * (dz * dx),     -m * (dz * dy),     m * (d2 - dz * dz)],
    ]
    return [
        [I_origin[0][0] - shift[0][0], I_origin[0][1] - shift[0][1], I_origin[0][2] - shift[0][2]],
        [I_origin[1][0] - shift[1][0], I_origin[1][1] - shift[1][1], I_origin[1][2] - shift[1][2]],
        [I_origin[2][0] - shift[2][0], I_origin[2][1] - shift[2][1], I_origin[2][2] - shift[2][2]],
    ]


def _transpose_r(r):
    return [
        [r[0][0], r[1][0], r[2][0]],
        [r[0][1], r[1][1], r[2][1]],
        [r[0][2], r[1][2], r[2][2]],
    ]


def _matmul_r(a, b):
    out = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
    return out


def _transform_point_rel(r, t, p):
    # p' = R*p + t
    return _transform_point(r, t, p)


def _normalize_moi_list(moi):
    # Fusion API can return [valid, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    # or just [Ixx, Iyy, Izz, Ixy, Ixz, Iyz].
    if isinstance(moi, (list, tuple)) and len(moi) == 7 and isinstance(moi[0], bool):
        return list(moi[1:])
    return list(moi)

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        if not isinstance(design, adsk.fusion.Design):
            ui.messageBox('No active Fusion design')
            return

        # choose output file
        dlg = ui.createFileDialog()
        dlg.isMultiSelectEnabled = False
        dlg.title = 'Save mass properties CSV (world COM + COM inertia)'
        dlg.filter = 'CSV (*.csv)'
        if dlg.showSave() != adsk.core.DialogResults.DialogOK:
            return
        out_path = dlg.filename
        debug_path = out_path.replace('.csv', '_debug.csv')

        with open(out_path, 'w', newline='') as f, open(debug_path, 'w', newline='') as df:
            writer = csv.writer(f)
            debug_writer = csv.writer(df)
            writer.writerow([
                'name',
                'mass_g',
                'com_x_mm', 'com_y_mm', 'com_z_mm',
                'origin_x_mm', 'origin_y_mm', 'origin_z_mm',
                'Ixx_gmm2', 'Iyy_gmm2', 'Izz_gmm2',
                'Ixy_gmm2', 'Ixz_gmm2', 'Iyz_gmm2'
            ])
            debug_writer.writerow([
                'component',
                'body',
                'mass_g',
                'com_x_mm', 'com_y_mm', 'com_z_mm',
                'Ixx_gmm2', 'Iyy_gmm2', 'Izz_gmm2',
                'Ixy_gmm2', 'Ixz_gmm2', 'Iyz_gmm2',
                'moi_raw'
            ])

            # Component aggregates computed from their bodies (world coordinates)
            root = design.rootComponent
            for occ in root.allOccurrences:
                bodies = [b for b in occ.bRepBodies]
                if not bodies:
                    continue

                # First pass: compute total mass and COM in world (cm)
                _, t_occ = _matrix_to_rotation_and_translation(occ.transform)
                total_mass = 0.0
                com_sum = [0.0, 0.0, 0.0]
                body_data = []

                for body in bodies:
                    props = body.physicalProperties
                    mass = props.mass  # kg
                    com = props.centerOfMass  # cm in occ space
                    moi = _normalize_moi_list(props.getXYZMomentsOfInertia())  # kg*cm^2 about origin in occ axes

                    # Fusion returns world-space COM for occurrence bodies.
                    com_w = (com.x, com.y, com.z)
                    total_mass += mass
                    com_sum[0] += mass * com_w[0]
                    com_sum[1] += mass * com_w[1]
                    com_sum[2] += mass * com_w[2]

                    # Fusion API order (empirical): [Ixx, Iyy, Izz, Ixy, Iyz, Ixz]
                    I_origin = [
                        [moi[0], moi[3], moi[5]],
                        [moi[3], moi[1], moi[4]],
                        [moi[5], moi[4], moi[2]],
                    ]
                    I_world_com = _shift_inertia_origin_to_com(I_origin, mass, (com.x, com.y, com.z))
                    body_data.append((mass, com_w, I_world_com))

                if total_mass <= 0:
                    continue

                com_total = [c / total_mass for c in com_sum]

                # Second pass: sum inertia about component COM (world axes)
                I_total = [[0.0] * 3 for _ in range(3)]
                for mass, com_w, I_world in body_data:
                    dx = com_w[0] - com_total[0]
                    dy = com_w[1] - com_total[1]
                    dz = com_w[2] - com_total[2]
                    I_shifted = _shift_inertia_com_to_origin(I_world, mass, (dx, dy, dz))
                    for i in range(3):
                        for j in range(3):
                            I_total[i][j] += I_shifted[i][j]

                mass_g = total_mass * 1000.0
                com_mm = (com_total[0] * 10.0, com_total[1] * 10.0, com_total[2] * 10.0)
                origin_mm = (t_occ[0] * 10.0, t_occ[1] * 10.0, t_occ[2] * 10.0)
                # kg*cm^2 -> g*mm^2 : 1e5
                moi_gmm2 = (
                    I_total[0][0] * 1.0e5, I_total[1][1] * 1.0e5, I_total[2][2] * 1.0e5,
                    I_total[0][1] * 1.0e5, I_total[0][2] * 1.0e5, I_total[1][2] * 1.0e5
                )

                writer.writerow([
                    f'{occ.name}::component',
                    mass_g,
                    com_mm[0], com_mm[1], com_mm[2],
                    origin_mm[0], origin_mm[1], origin_mm[2],
                    moi_gmm2[0], moi_gmm2[1], moi_gmm2[2],
                    moi_gmm2[3], moi_gmm2[4], moi_gmm2[5]
                ])

            # Aggregate container components by name (even if they have no bodies)
            target_set = set(TARGET_COMPONENTS)
            for occ in root.allOccurrences:
                comp_name = occ.component.name
                if comp_name not in target_set:
                    continue

                # Aggregate in world frame (for convert_com.py inputs).
                _, t_comp = _matrix_to_rotation_and_translation(occ.transform)
                occs = _collect_descendant_occurrences(occ)
                body_entries = []
                total_mass = 0.0
                com_sum = [0.0, 0.0, 0.0]

                for sub in occs:
                    for body in sub.bRepBodies:
                        props = body.physicalProperties
                        mass = props.mass  # kg
                        com = props.centerOfMass  # cm in sub occ space
                        moi = _normalize_moi_list(props.getXYZMomentsOfInertia())  # kg*cm^2 about origin in sub-occ axes

                        # Fusion returns world-space COM for occurrence bodies.
                        com_c = (com.x, com.y, com.z)
                        total_mass += mass
                        com_sum[0] += mass * com_c[0]
                        com_sum[1] += mass * com_c[1]
                        com_sum[2] += mass * com_c[2]

                        I_origin = [
                            [moi[0], moi[3], moi[5]],
                            [moi[3], moi[1], moi[4]],
                            [moi[5], moi[4], moi[2]],
                        ]
                        # Convert inertia about origin to inertia about COM (world frame)
                        I_comp = _shift_inertia_origin_to_com(I_origin, mass, (com.x, com.y, com.z))
                        body_entries.append((mass, com_c, I_comp))

                        # Debug per-body contribution (component frame, COM inertia)
                        mass_g = mass * 1000.0
                        com_mm = (com_c[0] * 10.0, com_c[1] * 10.0, com_c[2] * 10.0)
                        moi_gmm2 = (
                            I_comp[0][0] * 1.0e5, I_comp[1][1] * 1.0e5, I_comp[2][2] * 1.0e5,
                            I_comp[0][1] * 1.0e5, I_comp[0][2] * 1.0e5, I_comp[1][2] * 1.0e5
                        )
                        debug_writer.writerow([
                            comp_name,
                            body.name,
                            mass_g,
                            com_mm[0], com_mm[1], com_mm[2],
                            moi_gmm2[0], moi_gmm2[1], moi_gmm2[2],
                            moi_gmm2[3], moi_gmm2[4], moi_gmm2[5],
                            list(moi)
                        ])

                if total_mass <= 0:
                    continue

                com_total = [c / total_mass for c in com_sum]
                I_total = [[0.0] * 3 for _ in range(3)]
                for mass, com_c, I_comp in body_entries:
                    dx = com_c[0] - com_total[0]
                    dy = com_c[1] - com_total[1]
                    dz = com_c[2] - com_total[2]
                    I_shifted = _shift_inertia_com_to_origin(I_comp, mass, (dx, dy, dz))
                    for i in range(3):
                        for j in range(3):
                            I_total[i][j] += I_shifted[i][j]

                mass_g = total_mass * 1000.0
                com_mm = (com_total[0] * 10.0, com_total[1] * 10.0, com_total[2] * 10.0)
                origin_mm = (t_comp[0] * 10.0, t_comp[1] * 10.0, t_comp[2] * 10.0)
                moi_gmm2 = (
                    I_total[0][0] * 1.0e5, I_total[1][1] * 1.0e5, I_total[2][2] * 1.0e5,
                    I_total[0][1] * 1.0e5, I_total[0][2] * 1.0e5, I_total[1][2] * 1.0e5
                )

                writer.writerow([
                    f'{comp_name}::aggregate',
                    mass_g,
                    com_mm[0], com_mm[1], com_mm[2],
                    origin_mm[0], origin_mm[1], origin_mm[2],
                    moi_gmm2[0], moi_gmm2[1], moi_gmm2[2],
                    moi_gmm2[3], moi_gmm2[4], moi_gmm2[5]
                ])

        ui.messageBox(f'Wrote mass properties to {out_path}\\nDebug: {debug_path}')

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

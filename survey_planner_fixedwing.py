import math

try:
    from shapely.affinity import rotate as shapely_rotate
    from shapely.geometry import GeometryCollection, LineString, MultiLineString, Point, Polygon,MultiPoint ,box
    from shapely.ops import unary_union
except ImportError as exc:
    shapely_rotate = None
    SHAPELY_IMPORT_ERROR = exc
else:
    SHAPELY_IMPORT_ERROR = None

def regularize_four_point_rectangle(points):
    """
    Kullanıcının yaklaşık koyduğu dört köşeyi kapsayan,
    en küçük alanlı döndürülmüş düzgün dikdörtgeni üretir.

    Sonuç:
    - Tam 4 köşe
    - Karşı kenarlar paralel
    - Komşu kenarlar 90 derece
    - Noktaların tıklama sırasından etkilenmez
    """
    if shapely_rotate is None:
        raise RuntimeError(
            "Dikdörtgen hesabı için Shapely gerekli."
        ) from SHAPELY_IMPORT_ERROR

    if len(points) != 4:
        raise ValueError(
            "Dikdörtgen oluşturmak için tam 4 nokta gerekli."
        )

    normalized = [
        (float(x), float(y))
        for x, y in points
    ]

    hull = MultiPoint(normalized).convex_hull

    if hull.geom_type != "Polygon" or hull.area <= 0.25:
        raise ValueError(
            "Dört nokta düzgün bir alan oluşturmuyor."
        )

    rectangle = hull.minimum_rotated_rectangle

    corners = [
        (float(x), float(y))
        for x, y in list(rectangle.exterior.coords)[:-1]
    ]

    if len(corners) != 4:
        raise ValueError(
            "Dikdörtgenin dört köşesi üretilemedi."
        )

    # İlk köşe, kullanıcının P1 noktasına en yakın köşe olsun.
    first_x, first_y = normalized[0]

    start_index = min(
        range(4),
        key=lambda index: math.hypot(
            corners[index][0] - first_x,
            corners[index][1] - first_y,
        ),
    )

    return (
        corners[start_index:]
        + corners[:start_index]
    )

def mod2pi(theta):
    return theta % (2.0 * math.pi)


def _lsl(alpha, beta, d):
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sa - sb)
    if p2 < -1e-10:
        return None
    p = math.sqrt(max(0.0, p2))
    tmp = math.atan2(cb - ca, d + sa - sb)
    return mod2pi(-alpha + tmp), p, mod2pi(beta - tmp), "LSL"


def _rsr(alpha, beta, d):
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sb - sa)
    if p2 < -1e-10:
        return None
    p = math.sqrt(max(0.0, p2))
    tmp = math.atan2(ca - cb, d - sa + sb)
    return mod2pi(alpha - tmp), p, mod2pi(-beta + tmp), "RSR"


def _lsr(alpha, beta, d):
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    p2 = -2.0 + d * d + 2.0 * cab + 2.0 * d * (sa + sb)
    if p2 < -1e-10:
        return None
    p = math.sqrt(max(0.0, p2))
    tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
    return mod2pi(-alpha + tmp), p, mod2pi(-beta + tmp), "LSR"


def _rsl(alpha, beta, d):
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    p2 = d * d - 2.0 + 2.0 * cab - 2.0 * d * (sa + sb)
    if p2 < -1e-10:
        return None
    p = math.sqrt(max(0.0, p2))
    tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
    return mod2pi(alpha - tmp), p, mod2pi(beta - tmp), "RSL"


def _rlr(alpha, beta, d):
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    tmp = (6.0 - d * d + 2.0 * cab + 2.0 * d * (sa - sb)) / 8.0
    if abs(tmp) > 1.0 + 1e-10:
        return None
    tmp = max(-1.0, min(1.0, tmp))
    p = mod2pi(2.0 * math.pi - math.acos(tmp))
    t = mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
    q = mod2pi(alpha - beta - t + p)
    return t, p, q, "RLR"


def _lrl(alpha, beta, d):
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    tmp = (6.0 - d * d + 2.0 * cab + 2.0 * d * (-sa + sb)) / 8.0
    if abs(tmp) > 1.0 + 1e-10:
        return None
    tmp = max(-1.0, min(1.0, tmp))
    p = mod2pi(2.0 * math.pi - math.acos(tmp))
    t = mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
    q = mod2pi(beta - alpha - t + p)
    return t, p, q, "LRL"


PLANNERS = (_lsl, _rsr, _lsr, _rsl, _rlr, _lrl)


def dubins_candidates(start, goal, radius):
    x0, y0, yaw0 = start
    x1, y1, yaw1 = goal
    dx, dy = x1 - x0, y1 - y0
    distance = math.hypot(dx, dy)
    theta = mod2pi(math.atan2(dy, dx))
    alpha = mod2pi(yaw0 - theta)
    beta = mod2pi(yaw1 - theta)
    d = distance / radius
    result = []
    for planner in PLANNERS:
        item = planner(alpha, beta, d)
        if item is None:
            continue
        t, p, q, mode = item
        result.append((radius * (t + p + q), (t, p, q), mode))
    return sorted(result, key=lambda item: item[0])


def _advance_pose(x, y, yaw, mode, distance, radius):
    if mode == "S":
        return (
            x + distance * math.cos(yaw),
            y + distance * math.sin(yaw),
            yaw,
        )

    direction = 1.0 if mode == "L" else -1.0
    angle = direction * distance / radius
    cx = x - direction * radius * math.sin(yaw)
    cy = y + direction * radius * math.cos(yaw)
    new_yaw = yaw + angle
    new_x = cx + direction * radius * math.sin(new_yaw)
    new_y = cy - direction * radius * math.cos(new_yaw)
    return new_x, new_y, new_yaw


def sample_dubins(start, lengths, mode, radius, step=5.0):
    x, y, yaw = start
    points = [(x, y, mod2pi(yaw))]
    for normalized_length, segment_mode in zip(lengths, mode):
        segment_distance = normalized_length * radius
        travelled = 0.0
        while travelled + step < segment_distance:
            x, y, yaw = _advance_pose(x, y, yaw, segment_mode, step, radius)
            travelled += step
            points.append((x, y, mod2pi(yaw)))
        remaining = segment_distance - travelled
        if remaining > 1e-9:
            x, y, yaw = _advance_pose(x, y, yaw, segment_mode, remaining, radius)
        points.append((x, y, mod2pi(yaw)))
    return points

def dubins_internal_junctions(
    start,
    lengths,
    mode,
    radius,
):
    """
    Dubins dönüşünün üç geometrik parçası arasındaki
    iki birleşme noktasını üretir.

    Böylece dönüş boyunca çok sayıda waypoint koymak
    yerine yalnızca gerekli iki yönlendirme noktası kullanılır.
    """
    x, y, yaw = start
    result = []

    for index, (
        normalized_length,
        segment_mode,
    ) in enumerate(zip(lengths, mode)):

        segment_distance = (
            normalized_length * radius
        )

        x, y, yaw = _advance_pose(
            x,
            y,
            yaw,
            segment_mode,
            segment_distance,
            radius,
        )

        if index < len(mode) - 1:
            result.append(
                (x, y, mod2pi(yaw))
            )

    return result


def _extract_line_segments(geometry):
    if geometry.is_empty:
        return []
    if isinstance(geometry, LineString):
        return [geometry]
    if isinstance(geometry, MultiLineString):
        return list(geometry.geoms)
    if isinstance(geometry, GeometryCollection):
        result = []
        for item in geometry.geoms:
            result.extend(_extract_line_segments(item))
        return result
    return []

def _centered_lane_positions(
    min_y,
    max_y,
    lane_spacing_m,
    footprint_m,
):
    """
    Poligonun tarama yönüne dik genişliği üzerinde tam aralıklı,
    ortalanmış uçuş hatlarını oluşturur.

    Örnek:
        genişlik = 250 m
        hat aralığı = 20 m

        Sonuç:
        5, 25, 45, ..., 245
    """
    min_y = float(min_y)
    max_y = float(max_y)
    lane_spacing_m = float(lane_spacing_m)
    footprint_m = float(footprint_m)

    width_m = max_y - min_y

    if width_m <= 0.0:
        raise ValueError(
            "Tarama alanı genişliği pozitif olmalı."
        )

    if lane_spacing_m <= 0.0:
        raise ValueError(
            "Tarama hattı aralığı pozitif olmalı."
        )

    if footprint_m <= 0.0:
        raise ValueError(
            "Kamera yer izi pozitif olmalı."
        )

    # Hat aralığı kamera görüntüsünden büyük olursa
    # komşu görüntüler arasında taranmamış boşluk kalır.
    if lane_spacing_m > footprint_m:
        raise ValueError(
            "Hat aralığı kamera yer izinden büyük; "
            "alan tamamen taranamaz."
        )

    # Kamera tek geçişte bütün genişliği kapatıyorsa
    # uçuş hattını tam ortaya koy.
    if width_m <= footprint_m:
        return [
            (min_y + max_y) / 2.0
        ]

    lane_count = max(
        2,
        math.ceil(width_m / lane_spacing_m),
    )

    used_span_m = (
        (lane_count - 1) * lane_spacing_m
    )

    side_margin_m = (
        width_m - used_span_m
    ) / 2.0

    first_y = min_y + side_margin_m

    y_values = [
        first_y + index * lane_spacing_m
        for index in range(lane_count)
    ]

    # İlk ve son kamera görüntüsünün poligon kenarlarını
    # gerçekten kapsadığını son kez doğrula.
    covered_min_y = (
        y_values[0] - footprint_m / 2.0
    )

    covered_max_y = (
        y_values[-1] + footprint_m / 2.0
    )

    tolerance_m = 1e-6

    if (
        covered_min_y > min_y + tolerance_m
        or covered_max_y < max_y - tolerance_m
    ):
        raise ValueError(
            "Oluşturulan tarama hatları alan kenarlarını "
            "tamamen kapsamıyor."
        )

    return y_values


def _make_scan_rows(
    poly_rot,
    cross_width,
    along_width,
    overlap,
    lane_spacing_m=None,
):
    min_x, min_y, max_x, max_y = poly_rot.bounds
    width = max_y - min_y

    if lane_spacing_m is None:
        # Eski VTOL/ROI planlayıcılarının davranışı korunuyor.
        max_spacing = cross_width * (1.0 - overlap)

        if width <= cross_width:
            y_values = [
                (min_y + max_y) / 2.0
            ]
        else:
            count = 1 + math.ceil(
                (width - cross_width) / max_spacing
            )

            spacing = (
                width - cross_width
            ) / (count - 1)

            y_values = [
                min_y
                + cross_width / 2.0
                + index * spacing
                for index in range(count)
            ]

    else:
        # Yeni fixed-wing poligon sistemi:
        # tam kamera görüntüsüyle, kesin 20 m merkez aralığı.
        y_values = _centered_lane_positions(
            min_y=min_y,
            max_y=max_y,
            lane_spacing_m=lane_spacing_m,
            footprint_m=cross_width,
        )

    rows = []
    scan_margin = max(max_x - min_x, max_y - min_y) + 100.0
    for y in y_values:
        cutter = LineString([(min_x - scan_margin, y), (max_x + scan_margin, y)])
        clipped = poly_rot.intersection(cutter)
        segments = []
        for line in _extract_line_segments(clipped):
            coords = list(line.coords)
            if len(coords) < 2:
                continue
            x1 = min(point[0] for point in coords)
            x2 = max(point[0] for point in coords)
            if x2 - x1 > 0.05:
                segments.append((x1, x2))
        if segments:
            rows.append({"y": y, "segments": sorted(segments)})

    def coverage_for(current_rows):
        rectangles = []
        for current_row in current_rows:
            row_y = current_row["y"]
            for x1, x2 in current_row["segments"]:
                rectangles.append(
                    box(
                        x1 - along_width / 2.0,
                        row_y - cross_width / 2.0,
                        x2 + along_width / 2.0,
                        row_y + cross_width / 2.0,
                    )
                )
        current_coverage = (
            unary_union(rectangles) if rectangles else GeometryCollection()
        )
        return current_coverage, poly_rot.difference(current_coverage)

    coverage, uncovered = coverage_for(rows)

    # Slanted/concave boundaries can leave small corner pieces even though the
    # nominal row spacing is sufficient. Add a row through each largest
    # uncovered component until the polygon is covered geometrically.
    patch_iterations = 0
    while uncovered.area > 0.05 and patch_iterations < 50:
        components = (
            list(uncovered.geoms)
            if hasattr(uncovered, "geoms")
            else [uncovered]
        )
        largest = max(components, key=lambda item: item.area)
        point = largest.representative_point()
        patch_y = point.y
        if any(abs(row["y"] - patch_y) < 0.05 for row in rows):
            break
        cutter = LineString(
            [(min_x - scan_margin, patch_y), (max_x + scan_margin, patch_y)]
        )
        clipped = poly_rot.intersection(cutter)
        segments = []
        for line in _extract_line_segments(clipped):
            coords = list(line.coords)
            if len(coords) < 2:
                continue
            x1 = min(item[0] for item in coords)
            x2 = max(item[0] for item in coords)
            if x2 - x1 > 0.05:
                segments.append((x1, x2))
        if not segments:
            break
        rows.append({"y": patch_y, "segments": sorted(segments)})
        rows.sort(key=lambda row: row["y"])
        coverage, uncovered = coverage_for(rows)
        patch_iterations += 1
    return rows, coverage, uncovered


def scan_plan_candidates(
    polygon_xy,
    cross_width,
    along_width,
    overlap,
    lane_spacing_m=None,
):
    if shapely_rotate is None:
        raise RuntimeError(
            "Shapely bulunamadı. Bir kez 'python -m pip install shapely' çalıştırın."
        ) from SHAPELY_IMPORT_ERROR
    polygon = Polygon(polygon_xy)
    if not polygon.is_valid:
        raise ValueError("invalid polygon")
    angles = {math.radians(value) for value in range(0, 180, 5)}
    for i, (x1, y1) in enumerate(polygon_xy):
        x2, y2 = polygon_xy[(i + 1) % len(polygon_xy)]
        angles.add(mod2pi(math.atan2(y2 - y1, x2 - x1)) % math.pi)
    candidates = []
    for angle in sorted(angles):
        rotated = shapely_rotate(polygon, -angle, origin=(0.0, 0.0), use_radians=True)
        rows, coverage, uncovered = _make_scan_rows(
            rotated,
            cross_width,
            along_width,
            overlap,
            lane_spacing_m=lane_spacing_m,
        )
        pass_count = sum(len(row["segments"]) for row in rows)
        scan_length = sum(
            x2 - x1 for row in rows for x1, x2 in row["segments"]
        )
        candidates.append(
            {
                "angle": angle,
                "rotated_polygon": rotated,
                "rows": rows,
                "coverage": coverage,
                "uncovered": uncovered,
                "pass_count": pass_count,
                "scan_length": scan_length,
            }
        )
    return sorted(
        candidates,
        key=lambda c: (c["pass_count"], c["uncovered"].area, c["scan_length"]),
    )


def rotate_xy(point, angle):
    x, y = point
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return x * cosine - y * sine, x * sine + y * cosine


def _vtol_route_variant(candidate, reverse_rows=False, start_forward=True):
    """Build one alternating VTOL route variant from clipped scan segments."""
    rows = list(candidate["rows"])
    if reverse_rows:
        rows.reverse()

    angle = candidate["angle"]
    forward = start_forward
    route_points = []
    scan_pass_id = 0

    for row in rows:
        segments = list(row["segments"])
        if not forward:
            segments.reverse()

        for x1, x2 in segments:
            if forward:
                start_rotated = (x1, row["y"])
                end_rotated = (x2, row["y"])
            else:
                start_rotated = (x2, row["y"])
                end_rotated = (x1, row["y"])

            start = rotate_xy(start_rotated, angle)
            end = rotate_xy(end_rotated, angle)
            route_points.append(
                {
                    "x": start[0],
                    "y": start[1],
                    "kind": "SCAN_ENTRY",
                    "scan_pass": scan_pass_id,
                }
            )
            route_points.append(
                {
                    "x": end[0],
                    "y": end[1],
                    "kind": "SCAN_EXIT",
                    "scan_pass": scan_pass_id,
                }
            )
            scan_pass_id += 1

        forward = not forward

    return route_points


def _sample_route_xy(route_points, step_m=3.0):
    """Sample straight waypoint legs for boundary-distance validation."""
    if not route_points:
        return []
    result = [(route_points[0]["x"], route_points[0]["y"])]
    for previous, current in zip(route_points, route_points[1:]):
        dx = current["x"] - previous["x"]
        dy = current["y"] - previous["y"]
        distance = math.hypot(dx, dy)
        divisions = max(1, math.ceil(distance / step_m))
        for index in range(1, divisions + 1):
            ratio = index / divisions
            result.append(
                (previous["x"] + dx * ratio, previous["y"] + dy * ratio)
            )
    return result


def build_vtol_survey_plan(
    polygon_xy,
    altitude_m=20.0,
    horizontal_fov_deg=60.0,
    frame_width_px=1280,
    roi_size_px=380,
    overlap_ratio=0.15,
    vtol_speed_mps=4.5,
    vtol_accel_mps2=1.0,
    allowed_polygon_xy=None,
    max_waypoints=500,
):
    """Create a coverage-complete route to be flown entirely in VTOL mode.

    Unlike the fixed-wing planner, this route has no bank-angle turn radius.
    The vehicle brakes, translates to the next scan leg, and accelerates again.
    If ``allowed_polygon_xy`` is supplied, every straight route leg must remain
    inside that blue/safe boundary or the function refuses to produce a plan.
    """
    if shapely_rotate is None:
        raise RuntimeError(
            "Shapely bulunamadı. Bir kez 'python -m pip install shapely' çalıştırın."
        ) from SHAPELY_IMPORT_ERROR
    if len(polygon_xy) < 3:
        raise ValueError("Survey polygon needs at least three corners")
    if altitude_m <= 0.0 or vtol_speed_mps <= 0.0 or vtol_accel_mps2 <= 0.0:
        raise ValueError("Altitude, VTOL speed and VTOL acceleration must be positive")
    if frame_width_px <= 0 or roi_size_px <= 0 or roi_size_px > frame_width_px:
        raise ValueError("ROI size must be positive and no wider than the source frame")
    if not 0.0 < horizontal_fov_deg < 180.0:
        raise ValueError("Invalid FOV")
    if not 0.0 <= overlap_ratio < 1.0:
        raise ValueError("Invalid overlap")

    polygon = Polygon(polygon_xy)
    if not polygon.is_valid or polygon.area <= 0.25:
        raise ValueError("Invalid survey polygon")

    allowed_polygon = None
    if allowed_polygon_xy is not None:
        if len(allowed_polygon_xy) < 3:
            raise ValueError("Allowed-flight polygon needs at least three corners")
        allowed_polygon = Polygon(allowed_polygon_xy)
        if not allowed_polygon.is_valid or allowed_polygon.area <= polygon.area:
            raise ValueError("Invalid allowed-flight polygon")
        if not allowed_polygon.buffer(0.05).covers(polygon):
            raise ValueError("Blue allowed-flight polygon must contain the red scan polygon")

    focal_px = (frame_width_px / 2.0) / math.tan(
        math.radians(horizontal_fov_deg) / 2.0
    )
    footprint = altitude_m * roi_size_px / focal_px
    max_spacing = footprint * (1.0 - overlap_ratio)

    candidates = [
        item
        for item in scan_plan_candidates(
            polygon_xy, footprint, footprint, overlap_ratio
        )
        if item["uncovered"].area <= 0.05
    ]
    if not candidates:
        raise ValueError("No complete coverage angle was found")

    minimum_passes = candidates[0]["pass_count"]
    finalists = [item for item in candidates if item["pass_count"] == minimum_passes]
    variants = []
    for candidate in finalists:
        for reverse_rows in (False, True):
            for start_forward in (False, True):
                route_points = _vtol_route_variant(
                    candidate,
                    reverse_rows=reverse_rows,
                    start_forward=start_forward,
                )
                if len(route_points) > max_waypoints:
                    continue
                route_line = LineString(
                    [(item["x"], item["y"]) for item in route_points]
                )
                if allowed_polygon is not None and not allowed_polygon.buffer(0.05).covers(route_line):
                    continue
                variants.append(
                    {
                        "candidate": candidate,
                        "route_points": route_points,
                        "route_length_m": _route_length(route_points),
                    }
                )

    if not variants:
        if allowed_polygon is not None:
            raise ValueError(
                "VTOL route does not fit inside the blue allowed-flight polygon"
            )
        raise ValueError("No VTOL route variant was found")

    selected = min(variants, key=lambda item: item["route_length_m"])
    candidate = selected["candidate"]
    route_points = selected["route_points"]

    angle = candidate["angle"]
    scan_lines = []
    for row in candidate["rows"]:
        for x1, x2 in row["segments"]:
            scan_lines.append(
                [
                    rotate_xy((x1, row["y"]), angle),
                    rotate_xy((x2, row["y"]), angle),
                ]
            )

    reposition_lines = []
    for index in range(1, len(route_points) - 1, 2):
        if index + 1 < len(route_points):
            reposition_lines.append(
                [
                    (route_points[index]["x"], route_points[index]["y"]),
                    (route_points[index + 1]["x"], route_points[index + 1]["y"]),
                ]
            )

    coverage_original = shapely_rotate(
        candidate["coverage"], angle, origin=(0.0, 0.0), use_radians=True
    )
    sampled_route = _sample_route_xy(route_points)
    max_outside_distance = max(
        polygon.distance(Point(x, y)) for x, y in sampled_route
    )
    stopping_distance = vtol_speed_mps**2 / (2.0 * vtol_accel_mps2)
    recommended_margin = stopping_distance + max(3.0, footprint / 2.0)
    recommended_blue_polygon = polygon.buffer(recommended_margin)

    return {
        "route_points": route_points,
        "scan_lines": scan_lines,
        "reposition_lines": reposition_lines,
        "coverage_rings": _geometry_polygon_rings(coverage_original),
        # This is a calculated preview envelope, not proof that the real field
        # is obstacle-free or legally available.  The operator must accept or
        # replace it with an explicitly drawn blue allowed-flight polygon.
        "recommended_blue_polygon_rings": _geometry_polygon_rings(
            recommended_blue_polygon
        ),
        "allowed_polygon_rings": (
            _geometry_polygon_rings(allowed_polygon)
            if allowed_polygon is not None
            else []
        ),
        "summary": {
            "flight_mode": "VTOL",
            "angle_deg": math.degrees(angle) % 180.0,
            "lane_count": len(candidate["rows"]),
            "scan_pass_count": candidate["pass_count"],
            "reposition_count": max(0, candidate["pass_count"] - 1),
            "waypoint_count": len(route_points),
            "footprint_m": footprint,
            "max_lane_spacing_m": max_spacing,
            "uncovered_area_m2": candidate["uncovered"].area,
            "route_length_m": selected["route_length_m"],
            "estimated_time_s": selected["route_length_m"] / vtol_speed_mps,
            "vtol_speed_mps": vtol_speed_mps,
            "vtol_accel_mps2": vtol_accel_mps2,
            "stopping_distance_m": stopping_distance,
            "recommended_blue_margin_m": recommended_margin,
            "max_outside_red_polygon_m": max_outside_distance,
            "allowed_boundary_checked": allowed_polygon is not None,
        },
    }

def make_oriented_passes(
    candidate,
    lead_distance,
    exit_lead_distance=0.0,
):
    """
    lead_left/right:
        Sonraki hatta düzgün giriş yapabilmek için dışarıdaki giriş noktası.

    exit_left/right:
        Varsayılan olarak kırmızı sınırın tam üzerindedir.
        Böylece SCAN_END sonrasında ayrıca LEAD_OUT oluşmaz.
    """
    angle = candidate["angle"]
    result = []

    for row_index, row in enumerate(candidate["rows"]):
        for segment_index, (x1, x2) in enumerate(
            row["segments"]
        ):
            result.append(
                {
                    "row_index": row_index,
                    "segment_index": segment_index,

                    "scan_left": rotate_xy(
                        (x1, row["y"]),
                        angle,
                    ),
                    "scan_right": rotate_xy(
                        (x2, row["y"]),
                        angle,
                    ),

                    "lead_left": rotate_xy(
                        (
                            x1 - lead_distance,
                            row["y"],
                        ),
                        angle,
                    ),
                    "lead_right": rotate_xy(
                        (
                            x2 + lead_distance,
                            row["y"],
                        ),
                        angle,
                    ),

                    "exit_left": rotate_xy(
                        (
                            x1 - exit_lead_distance,
                            row["y"],
                        ),
                        angle,
                    ),
                    "exit_right": rotate_xy(
                        (
                            x2 + exit_lead_distance,
                            row["y"],
                        ),
                        angle,
                    ),

                    "angle": angle,
                }
            )

    return result

def pass_state(scan_pass, orientation):
    if orientation == 0:
        yaw = mod2pi(scan_pass["angle"])

        exit_point = scan_pass.get(
            "exit_right",
            scan_pass["scan_right"],
        )

        return {
            "entry": (
                scan_pass["lead_left"][0],
                scan_pass["lead_left"][1],
                yaw,
            ),
            "scan_start": scan_pass["scan_left"],
            "scan_end": scan_pass["scan_right"],
            "exit": (
                exit_point[0],
                exit_point[1],
                yaw,
            ),
        }

    yaw = mod2pi(scan_pass["angle"] + math.pi)

    exit_point = scan_pass.get(
        "exit_left",
        scan_pass["scan_left"],
    )

    return {
        "entry": (
            scan_pass["lead_right"][0],
            scan_pass["lead_right"][1],
            yaw,
        ),
        "scan_start": scan_pass["scan_right"],
        "scan_end": scan_pass["scan_left"],
        "exit": (
            exit_point[0],
            exit_point[1],
            yaw,
        ),
    }

def _polyline_outside_metrics(points, survey_polygon):
    """
    Dönüş yolunun kırmızı poligon dışında kalan kısmını ölçer.

    Dönüş değeri:
    - outside_length: dışarıda uçulan toplam metre
    - max_outside: kırmızı sınıra olan en büyük uzaklık
    """
    line = LineString(
        [(x, y) for x, y, _ in points]
    )

    outside_length = line.difference(
        survey_polygon
    ).length

    max_outside = max(
        survey_polygon.distance(Point(x, y))
        for x, y, _ in points
    )

    return (
        line,
        float(outside_length),
        float(max_outside),
    )

def choose_dubins_connector(
    start,
    goal,
    radius,
    survey_polygon,
    allowed_polygon,
    sample_step_m=2.0,
):
    """
    İki tarama hattı arasındaki en uygun sabit-kanat dönüşünü seçer.

    Yalnızca en kısa yolu seçmez. Her Dubins seçeneğini:
    - toplam mesafe,
    - kırmızı alan dışında kalma mesafesi,
    - kırmızıdan en fazla uzaklaşma,
    - izin verilen mavi sınır içinde kalma

    kriterlerine göre değerlendirir.
    """
    candidates = dubins_candidates(
        start,
        goal,
        radius
    )

    if not candidates:
        return None

    shortest_length = candidates[0][0]
    evaluated = []

    for total, lengths, mode in candidates:
        # Aşırı büyük ve gereksiz tam tur atan seçenekleri ele.
        maximum_reasonable_length = (
            shortest_length
            + 2.10 * math.pi * radius
        )

        if total > maximum_reasonable_length:
            continue

        sampled = sample_dubins(
            start,
            lengths,
            mode,
            radius,
            step=max(
                0.75,
                min(sample_step_m, radius / 12.0)
            ),
        )

        line, outside_length, max_outside = (
            _polyline_outside_metrics(
                sampled,
                survey_polygon
            )
        )

        # Dönüşün tamamı izin verilen mavi sınırın içinde olmalı.
        if not allowed_polygon.buffer(0.20).covers(line):
            continue

        # Düşük puan daha iyidir.
        #
        # 1.00 × toplam yol
        # 1.35 × kırmızı alan dışında uçulan yol
        # 3.00 × kırmızı sınırdan maksimum uzaklaşma
        score = (
            total
            + 1.35 * outside_length
            + 3.0 * max_outside
        )

        evaluated.append(
            {
                "score": float(score),
                "length": float(total),
                "lengths": lengths,
                "mode": mode,
                "sampled": sampled,
                "outside_length": float(outside_length),
                "max_outside": float(max_outside),
                "radius": float(radius)
            }
        )

    if not evaluated:
        return None

    return min(
        evaluated,
        key=lambda item: (
            item["score"],
            item["max_outside"],
            item["outside_length"],
            item["length"],
        ),
    )


def _optimise_pass_order(
    scan_passes,
    radius,
    survey_polygon,
    allowed_polygon,
    home_xy=None,
    start_pose=None,
    beam_width=350,
):
    """
    Tarama hatlarının sırasını ve uçuş yönünü birlikte optimize eder.

    Normal ilk taramada:
        home_xy kullanılır ve HOME'a en yakın giriş seçilir.

    Recovery taramasında:
        start_pose kullanılır ve uçağın gerçek konum/yönünden
        erişilebilen en uygun giriş seçilir.
    """
    pass_count = len(scan_passes)

    if pass_count == 0:
        raise ValueError("Tarama hattı bulunamadı.")

    # Aynı iki hat arasındaki dönüşü tekrar tekrar hesaplamamak için
    # sonuçları bellekte tutar.
    connector_cache = {}

    def connector(
        from_index,
        from_orientation,
        to_index,
        to_orientation,
    ):
        key = (
            from_index,
            from_orientation,
            to_index,
            to_orientation,
        )

        if key not in connector_cache:
            from_state = pass_state(
                scan_passes[from_index],
                from_orientation,
            )

            to_state = pass_state(
                scan_passes[to_index],
                to_orientation,
            )

            connector_cache[key] = choose_dubins_connector(
                from_state["exit"],
                to_state["entry"],
                radius,
                survey_polygon,
                allowed_polygon,
            )

        return connector_cache[key]

    initial_options = []

    # Recovery durumunda uçak zaten havadadır.
    # İlk recovery hattına, mevcut konum ve heading'den ulaşmalıdır.
    if start_pose is not None:
        for pass_index in range(pass_count):
            for orientation in (0, 1):
                entry = pass_state(
                    scan_passes[pass_index],
                    orientation,
                )["entry"]

                info = choose_dubins_connector(
                    start_pose,
                    entry,
                    radius,
                    survey_polygon,
                    allowed_polygon,
                )

                if info is not None:
                    initial_options.append(
                        (
                            pass_index,
                            orientation,
                            info["score"],
                            info,
                        )
                    )

    # Ana taramanın başlangıcında HOME'a en yakın hat girişini seç.
    else:
        raw_options = []

        for pass_index in range(pass_count):
            for orientation in (0, 1):
                entry = pass_state(
                    scan_passes[pass_index],
                    orientation,
                )["entry"]

                home_distance = 0.0

                if home_xy is not None:
                    home_distance = math.hypot(
                        entry[0] - home_xy[0],
                        entry[1] - home_xy[1],
                    )

                raw_options.append(
                    (
                        pass_index,
                        orientation,
                        home_distance,
                        None,
                    )
                )

        if home_xy is not None:
            nearest_distance = min(
                item[2] for item in raw_options
            )

            # HOME'a en yakın giriş şartını zayıf bir tavsiye olarak
            # bırakmıyoruz. Yalnızca en yakına 0.5 m içinde olan
            # başlangıç adayları kabul edilir.
            raw_options = [
                item
                for item in raw_options
                if item[2] <= nearest_distance + 0.50
            ]

        initial_options = raw_options

    if not initial_options:
        raise ValueError(
            "Uçağın mevcut konumundan izin verilen mavi sınır "
            "içinde ulaşılabilecek tarama girişi bulunamadı."
        )

    # Anahtar:
    # (ziyaret edilen hatlar, son hat, son uçuş yönü)
    states = {}

    for (
        pass_index,
        orientation,
        initial_cost,
        initial_connector,
    ) in initial_options:

        visited_mask = 1 << pass_index

        key = (
            visited_mask,
            pass_index,
            orientation,
        )

        state = {
            "cost": float(initial_cost),
            "order": [
                (pass_index, orientation)
            ],
            "outside_length": (
                initial_connector["outside_length"]
                if initial_connector is not None
                else 0.0
            ),
            "max_outside": (
                initial_connector["max_outside"]
                if initial_connector is not None
                else 0.0
            ),
            "connector_length": (
                initial_connector["length"]
                if initial_connector is not None
                else 0.0
            ),
            "initial_connector": initial_connector,
            "home_start_distance": (
                float(initial_cost)
                if start_pose is None and home_xy is not None
                else 0.0
            ),
        }

        previous = states.get(key)

        if (
            previous is None
            or state["cost"] < previous["cost"]
        ):
            states[key] = state

    # Örneğin 6 tarama hattında:
    # 0b111111 bütün altı hattın ziyaret edildiğini gösterir.
    full_mask = (1 << pass_count) - 1

    # 11 hatta kadar bütün kombinasyonları eksiksiz hesapla.
    # Daha büyük alanlarda hesap süresini kontrol altında tutmak için
    # en iyi 350 adayla beam search kullan.
    exact_search = pass_count <= 11

    for _ in range(1, pass_count):
        expanded = {}

        for (
            mask,
            last_index,
            last_orientation,
        ), state in states.items():

            for next_index in range(pass_count):
                # Bu hat daha önce uçulduysa atla.
                if mask & (1 << next_index):
                    continue

                for next_orientation in (0, 1):
                    info = connector(
                        last_index,
                        last_orientation,
                        next_index,
                        next_orientation,
                    )

                    # İzin verilen mavi sınır içinde dönüş bulunamadı.
                    if info is None:
                        continue

                    new_mask = mask | (1 << next_index)

                    new_key = (
                        new_mask,
                        next_index,
                        next_orientation,
                    )

                    new_state = {
                        "cost": (
                            state["cost"]
                            + info["score"]
                        ),
                        "order": (
                            state["order"]
                            + [(next_index, next_orientation)]
                        ),
                        "outside_length": (
                            state["outside_length"]
                            + info["outside_length"]
                        ),
                        "max_outside": max(
                            state["max_outside"],
                            info["max_outside"],
                        ),
                        "connector_length": (
                            state["connector_length"]
                            + info["length"]
                        ),
                        "initial_connector": (
                            state["initial_connector"]
                        ),
                        "home_start_distance": (
                            state["home_start_distance"]
                        ),
                    }

                    old_state = expanded.get(new_key)

                    if old_state is None:
                        expanded[new_key] = new_state
                        continue

                    new_comparison = (
                        new_state["cost"],
                        new_state["max_outside"],
                        new_state["connector_length"],
                    )

                    old_comparison = (
                        old_state["cost"],
                        old_state["max_outside"],
                        old_state["connector_length"],
                    )

                    if new_comparison < old_comparison:
                        expanded[new_key] = new_state

        if not expanded:
            raise ValueError(
                "Bütün tarama hatlarını birbirine bağlayan, "
                "dönüş yarıçapına ve mavi sınıra uygun rota bulunamadı."
            )

        if exact_search:
            states = expanded

        else:
            best_items = sorted(
                expanded.items(),
                key=lambda item: (
                    item[1]["cost"],
                    item[1]["max_outside"],
                    item[1]["connector_length"],
                ),
            )[:beam_width]

            states = dict(best_items)

    finalists = [
        state
        for (mask, _, _), state in states.items()
        if mask == full_mask
    ]

    if not finalists:
        raise ValueError(
            "Bütün tarama hatlarını tamamlayan sabit-kanat "
            "rotası bulunamadı."
        )

    def final_key(state):
        """
        Ana maliyet dönüşlerin güvenliğidir.

        İki rota birbirine yakınsa, son noktası HOME'a daha yakın
        olan rotaya küçük bir avantaj verilir.
        """
        end_home_distance = 0.0

        if home_xy is not None:
            last_index, last_orientation = state["order"][-1]

            exit_pose = pass_state(
                scan_passes[last_index],
                last_orientation,
            )["exit"]

            end_home_distance = math.hypot(
                exit_pose[0] - home_xy[0],
                exit_pose[1] - home_xy[1],
            )

        return (
            state["cost"] + 0.03 * end_home_distance,
            state["max_outside"],
            state["connector_length"],
            end_home_distance,
        )

    best = min(
        finalists,
        key=final_key,
    )

    best["connector_cache"] = connector_cache

    if home_xy is not None:
        last_index, last_orientation = best["order"][-1]

        exit_pose = pass_state(
            scan_passes[last_index],
            last_orientation,
        )["exit"]

        best["home_end_distance"] = math.hypot(
            exit_pose[0] - home_xy[0],
            exit_pose[1] - home_xy[1],
        )

    else:
        best["home_end_distance"] = 0.0

    return best

def _sequential_order_variants(scan_passes):
    """
    Tarama hatlarını fiziksel sırasıyla düzenler.

    Yalnızca dört seçenek üretir:
    - aşağıdan yukarı, ilk hat soldan sağa
    - aşağıdan yukarı, ilk hat sağdan sola
    - yukarıdan aşağı, ilk hat soldan sağa
    - yukarıdan aşağı, ilk hat sağdan sola

    HOME'a en yakın olan seçenek daha sonra seçilir.
    """
    if not scan_passes:
        return []

    rows = {}

    for pass_index, scan_pass in enumerate(scan_passes):
        row_index = int(
            scan_pass.get("row_index", pass_index)
        )

        rows.setdefault(
            row_index,
            []
        ).append(pass_index)

    sorted_row_indexes = sorted(rows)
    variants = []

    for reverse_rows in (False, True):
        row_order = list(sorted_row_indexes)

        if reverse_rows:
            row_order.reverse()

        for first_orientation in (0, 1):
            order = []

            for row_position, row_index in enumerate(row_order):
                # Komşu hatlar ters yönde uçulur:
                # 1 sağa, 2 sola, 3 sağa...
                orientation = (
                    first_orientation
                    if row_position % 2 == 0
                    else 1 - first_orientation
                )

                row_passes = sorted(
                    rows[row_index],
                    key=lambda index: (
                        scan_passes[index].get(
                            "segment_index",
                            0,
                        )
                    ),
                )

                # Sağdan sola uçulacaksa aynı satırdaki
                # parçaların sırası da ters çevrilir.
                if orientation == 1:
                    row_passes.reverse()

                for pass_index in row_passes:
                    order.append(
                        (
                            pass_index,
                            orientation,
                        )
                    )

            variants.append(order)

    return variants

def _select_sequential_pass_order(
    scan_passes,
    radius,
    survey_polygon,
    allowed_polygon,
    home_xy=None,
    start_pose=None,
):
    """
    Hatları kesin fiziksel sırayla uçurur.

    Serbest rota optimizasyonu yapmaz.
    Yalnızca HOME'a hangi köşeden başlamanın daha uygun
    olduğunu ve ilk hattın yönünü seçer.
    """
    if not scan_passes:
        raise ValueError(
            "Sıralanacak tarama hattı bulunamadı."
        )

    # Recovery sistemini şimdilik değiştirmiyoruz.
    # İleride mevcut konumdan yeniden tarama gerekirse
    # eski gelişmiş optimizasyon kullanılabilir.
    if start_pose is not None:
        return _optimise_pass_order(
            scan_passes=scan_passes,
            radius=radius,
            survey_polygon=survey_polygon,
            allowed_polygon=allowed_polygon,
            home_xy=home_xy,
            start_pose=start_pose,
        )

    connector_cache = {}

    def get_connector(
        from_index,
        from_orientation,
        to_index,
        to_orientation,
    ):
        key = (
            from_index,
            from_orientation,
            to_index,
            to_orientation,
        )

        if key not in connector_cache:
            from_state = pass_state(
                scan_passes[from_index],
                from_orientation,
            )

            to_state = pass_state(
                scan_passes[to_index],
                to_orientation,
            )

            connector_cache[key] = (
                choose_dubins_connector(
                    start=from_state["exit"],
                    goal=to_state["entry"],
                    radius=radius,
                    survey_polygon=survey_polygon,
                    allowed_polygon=allowed_polygon,
                )
            )

        return connector_cache[key]

    evaluated = []

    for order in _sequential_order_variants(scan_passes):
        if not order:
            continue

        first_index, first_orientation = order[0]

        first_entry = pass_state(
            scan_passes[first_index],
            first_orientation,
        )["entry"]

        home_start_distance = 0.0

        if home_xy is not None:
            home_start_distance = math.hypot(
                first_entry[0] - home_xy[0],
                first_entry[1] - home_xy[1],
            )

        total_cost = 0.0
        outside_length = 0.0
        max_outside = 0.0
        connector_length = 0.0
        valid = True

        for current, following in zip(
            order,
            order[1:],
        ):
            from_index, from_orientation = current
            to_index, to_orientation = following

            info = get_connector(
                from_index,
                from_orientation,
                to_index,
                to_orientation,
            )

            if info is None:
                valid = False
                break

            total_cost += info["score"]
            outside_length += info["outside_length"]
            connector_length += info["length"]

            max_outside = max(
                max_outside,
                info["max_outside"],
            )

        if not valid:
            continue

        last_index, last_orientation = order[-1]

        last_exit = pass_state(
            scan_passes[last_index],
            last_orientation,
        )["exit"]

        home_end_distance = 0.0

        if home_xy is not None:
            home_end_distance = math.hypot(
                last_exit[0] - home_xy[0],
                last_exit[1] - home_xy[1],
            )

        evaluated.append(
            {
                "cost": float(total_cost),
                "order": order,
                "outside_length": float(outside_length),
                "max_outside": float(max_outside),
                "connector_length": float(connector_length),
                "initial_connector": None,
                "home_start_distance": float(
                    home_start_distance
                ),
                "home_end_distance": float(
                    home_end_distance
                ),
            }
        )

    if not evaluated:
        raise ValueError(
            "Sıralı tarama hatlarını birbirine bağlayan "
            "uygun sabit-kanat dönüşü bulunamadı."
        )

    if home_xy is not None:
        nearest_home_distance = min(
            item["home_start_distance"]
            for item in evaluated
        )

        # HOME'a en yakın köşeden başlamayı kesin şart yapıyoruz.
        evaluated = [
            item
            for item in evaluated
            if item["home_start_distance"]
            <= nearest_home_distance + 0.50
        ]

    best = min(
        evaluated,
        key=lambda item: (
            item["cost"],
            item["max_outside"],
            item["outside_length"],
            item["home_end_distance"],
        ),
    )

    best["connector_cache"] = connector_cache

    return best



def _append_route_point(
    route_points,
    x,
    y,
    kind,
    scan_pass=-1,
    **extra
):
    """
    Aynı konuma çok yakın mükerrer waypoint oluşmasını engeller.
    Her waypointin görevini 'kind' alanında saklar.
    """
    point = {
        "x": float(x),
        "y": float(y),
        "kind": kind,
        "scan_pass": int(scan_pass),
    }

    point.update(extra)

    if route_points:
        previous = route_points[-1]

        distance = math.hypot(
            previous["x"] - point["x"],
            previous["y"] - point["y"],
        )

        if distance < 0.05:
            # Sınır noktası, sıradan TURN noktasından daha anlamlıdır.
            if kind in (
                "SCAN_START",
                "SCAN_END",
                "LEAD_IN",
                "LEAD_OUT",
            ):
                route_points[-1] = point

            return

    route_points.append(point)

TURN_ACCEPTANCE_RADIUS_M = 12.0
LEAD_ACCEPTANCE_RADIUS_M = 6.0
SCAN_ACCEPTANCE_RADIUS_M = 3.0

def _append_dubins_control_points(
    route_points,
    connector_info,
    turn_index,
):
    sampled = connector_info["sampled"]

    controls = dubins_internal_junctions(
        start=sampled[0],
        lengths=connector_info["lengths"],
        mode=connector_info["mode"],
        radius=connector_info["radius"],
    )

    if len(controls) != 2:
        raise ValueError(
            "Dönüş için iki Dubins kontrol noktası üretilemedi."
        )

    for control_index, (x, y, _) in enumerate(
        controls,
        start=1,
    ):
        _append_route_point(
            route_points,
            x,
            y,
            "TURN",
            -1,
            turn_index=turn_index,
            dubins_type=connector_info["mode"],
            turn_role=f"JUNCTION_{control_index}",
            acceptance_radius_m=(
                TURN_ACCEPTANCE_RADIUS_M
            ),
        )

def build_route_points(
    scan_passes,
    route_info,
    mission_step=10.0,
):
    """
    Her hat:
        LEAD_IN -> SCAN_START -> SCAN_END

    Her iki hat arası:
        TURN-1 -> TURN-2 -> sonraki LEAD_IN
    """
    # Eski çağrılar bozulmasın; artık seyrekleştirmede kullanılmıyor.
    _ = mission_step

    route_points = []
    oriented_scan_passes = []
    lead_lines = []
    turn_lines = []

    order = route_info["order"]
    cache = route_info["connector_cache"]

    # Recovery sırasında mevcut konumdan ilk hatta bağlantı.
    initial = route_info.get("initial_connector")

    if initial is not None:
        sampled = initial["sampled"]

        turn_lines.append(
            [
                (float(x), float(y))
                for x, y, _ in sampled
            ]
        )

        _append_dubins_control_points(
            route_points,
            initial,
            turn_index=-1,
        )

    for order_index, (
        pass_index,
        orientation,
    ) in enumerate(order):

        state = pass_state(
            scan_passes[pass_index],
            orientation,
        )

        entry = state["entry"]
        scan_start = state["scan_start"]
        scan_end = state["scan_end"]
        exit_pose = state["exit"]

        _append_route_point(
            route_points,
            entry[0],
            entry[1],
            "LEAD_IN",
            order_index,
            source_pass=pass_index,
            acceptance_radius_m=(
                LEAD_ACCEPTANCE_RADIUS_M
            ),
        )

        _append_route_point(
            route_points,
            scan_start[0],
            scan_start[1],
            "SCAN_START",
            order_index,
            source_pass=pass_index,
            acceptance_radius_m=(
                SCAN_ACCEPTANCE_RADIUS_M
            ),
        )

        _append_route_point(
            route_points,
            scan_end[0],
            scan_end[1],
            "SCAN_END",
            order_index,
            source_pass=pass_index,
            acceptance_radius_m=(
                SCAN_ACCEPTANCE_RADIUS_M
            ),
        )

        lead_lines.append(
            [
                (entry[0], entry[1]),
                scan_start,
            ]
        )

        # Gelecekte exit_lead_distance sıfırdan farklı seçilirse
        # LEAD_OUT tekrar otomatik oluşabilir.
        exit_distance = math.hypot(
            exit_pose[0] - scan_end[0],
            exit_pose[1] - scan_end[1],
        )

        if exit_distance > 0.05:
            _append_route_point(
                route_points,
                exit_pose[0],
                exit_pose[1],
                "LEAD_OUT",
                order_index,
                source_pass=pass_index,
                acceptance_radius_m=(
                    LEAD_ACCEPTANCE_RADIUS_M
                ),
            )

            lead_lines.append(
                [
                    scan_end,
                    (
                        exit_pose[0],
                        exit_pose[1],
                    ),
                ]
            )

        oriented_scan_passes.append(
            {
                "scan_pass": order_index,
                "source_pass": pass_index,
                "start": (
                    float(scan_start[0]),
                    float(scan_start[1]),
                ),
                "end": (
                    float(scan_end[0]),
                    float(scan_end[1]),
                ),
                "heading_rad": float(entry[2]),
            }
        )

        if order_index + 1 >= len(order):
            continue

        next_index, next_orientation = (
            order[order_index + 1]
        )

        connector_info = cache[
            (
                pass_index,
                orientation,
                next_index,
                next_orientation,
            )
        ]

        sampled = connector_info["sampled"]

        # Haritada matematiksel dönüş eğrisi gösterilir.
        turn_lines.append(
            [
                (float(x), float(y))
                for x, y, _ in sampled
            ]
        )

        # Pixhawk'a yalnızca iki anlamlı dönüş noktası gider.
        _append_dubins_control_points(
            route_points,
            connector_info,
            turn_index=order_index,
        )

    return (
        route_points,
        oriented_scan_passes,
        lead_lines,
        turn_lines,
    )


def _geometry_polygon_rings(geometry):
    if geometry.is_empty:
        return []
    if isinstance(geometry, Polygon):
        return [[(float(x), float(y)) for x, y in geometry.exterior.coords]]
    if hasattr(geometry, "geoms"):
        rings = []
        for item in geometry.geoms:
            rings.extend(_geometry_polygon_rings(item))
        return rings
    return []


def _route_length(points):
    return sum(
        math.hypot(
            points[index]["x"] - points[index - 1]["x"],
            points[index]["y"] - points[index - 1]["y"],
        )
        for index in range(1, len(points))
    )

def _build_plan_from_passes(
    scan_passes,
    survey_polygon,
    turn_radius,
    allowed_turn_margin_m,
    mission_turn_step_m,
    turn_corridor_half_width_m,
    home_xy=None,
    start_pose=None,
):
    """
    Tarama hatlarının sırasını seçer, dönüşleri oluşturur ve tüm rotanın
    izin verilen sınır içinde kaldığını doğrular.
    """
    allowed_polygon = survey_polygon.buffer(
        allowed_turn_margin_m,
        join_style=2,
    )

    order_info = _select_sequential_pass_order(
        scan_passes=scan_passes,
        radius=turn_radius,
        survey_polygon=survey_polygon,
        allowed_polygon=allowed_polygon,
        home_xy=home_xy,
        start_pose=start_pose,
    )

    (
        route_points,
        oriented_passes,
        lead_lines,
        turn_lines,
    ) = build_route_points(
        scan_passes,
        order_info,
        mission_step=mission_turn_step_m,
    )

    mission_route_line = LineString(
        [
            (point["x"], point["y"])
            for point in route_points
        ]
    )

    if not allowed_polygon.buffer(0.25).covers(
        mission_route_line
    ):
        raise ValueError(
            "Oluşturulan waypoint rotası izin verilen "
            "dönüş sınırını aşıyor."
        )

    scan_lines = [
        [
            item["start"],
            item["end"],
        ]
        for item in oriented_passes
    ]

    exact_parts = [
        LineString(line)
        for line in (
            lead_lines
            + scan_lines
            + turn_lines
        )
        if len(line) >= 2
    ]

    if not exact_parts:
        raise ValueError(
            "Uçulabilir rota geometrisi oluşturulamadı."
        )

    # Yalnızca waypointleri değil, waypointlerin temsil ettiği gerçek
    # Dubins eğrilerini de mavi sınır içinde kontrol et.
    for part in exact_parts:
        if not allowed_polygon.buffer(0.25).covers(part):
            raise ValueError(
                "Gerçek dönüş geometrisi izin verilen "
                "sınırı aşıyor."
            )

    route_length = sum(
        part.length
        for part in exact_parts
    )

    outside_length = sum(
        part.difference(survey_polygon).length
        for part in exact_parts
    )

    max_outside = max(
        survey_polygon.distance(Point(x, y))
        for line in (
            lead_lines
            + scan_lines
            + turn_lines
        )
        for x, y in line
    )

    exact_route_geometry = unary_union(exact_parts)

    # Haritadaki mavi kesikli alan: seçilen gerçek rotanın koridoru.
    route_corridor = survey_polygon.union(
        exact_route_geometry.buffer(
            turn_corridor_half_width_m,
            cap_style=2,
            join_style=2,
        )
    )

    return {
        "route_points": route_points,
        "scan_passes": oriented_passes,
        "scan_lines": scan_lines,
        "lead_lines": lead_lines,
        "turn_lines": turn_lines,
        "route_corridor_rings": (
            _geometry_polygon_rings(route_corridor)
        ),
        "allowed_polygon_rings": (
            _geometry_polygon_rings(allowed_polygon)
        ),
        "route_length_m": float(route_length),
        "outside_length_m": float(outside_length),
        "max_outside_m": float(max_outside),
        "order_info": order_info,
    }


def build_survey_plan(
    polygon_xy,
    altitude_m=20.0,
    horizontal_fov_deg=60.0,
    frame_width_px=1280,
    roi_size_px=380,
    full_frame_swath_m=23.8,
    along_track_swath_m=16.0,
    lane_spacing_m=20.0,
    overlap_ratio=0.15,
    planning_speed_mps=15.0,
    design_bank_deg=40.0,
    turn_radius_safety=1.10,
    wp_radius_m=8.0,
    lead_margin_m=0,
    mission_turn_step_m=22.5,
    max_waypoints=500,
    safe_swath_m=5.0,
    home_xy=None,
    allowed_turn_margin_m=None,
    turn_corridor_half_width_m=7.5,
):
    """
    Verilen herhangi bir geçerli poligon için sabit-kanat tarama
    rotası oluşturur.
    """
    if shapely_rotate is None:
        raise RuntimeError(
            "Shapely bulunamadı. "
            "'python -m pip install shapely' çalıştırın."
        ) from SHAPELY_IMPORT_ERROR

    if altitude_m <= 0.0:
        raise ValueError("İrtifa pozitif olmalı.")

    if planning_speed_mps <= 0.0:
        raise ValueError("Planlama hızı pozitif olmalı.")

    if (
        frame_width_px <= 0
        or roi_size_px <= 0
        or roi_size_px > frame_width_px
    ):
        raise ValueError("ROI veya görüntü genişliği geçersiz.")

    if full_frame_swath_m <= 0.0:
        raise ValueError(
            "Tam kamera yer izi pozitif olmalı."
        )

    if along_track_swath_m <= 0.0:
        raise ValueError(
            "Kameranın uçuş yönündeki yer izi pozitif olmalı."
        )

    if lane_spacing_m <= 0.0:
        raise ValueError(
            "Tarama hattı aralığı pozitif olmalı."
        )

    if lane_spacing_m > full_frame_swath_m:
        raise ValueError(
            "Hat aralığı kamera yer izinden büyük. "
            "Bu ayarla alan tamamen taranamaz."
        )

    if not 0.0 < horizontal_fov_deg < 180.0:
        raise ValueError("FOV değeri geçersiz.")

    if not 0.0 <= overlap_ratio < 1.0:
        raise ValueError("Bindirme oranı geçersiz.")

    if not 0.0 < design_bank_deg < 60.0:
        raise ValueError("Tasarım yatış açısı geçersiz.")

    if turn_radius_safety < 1.0:
        raise ValueError(
            "Dönüş yarıçapı güvenlik katsayısı en az 1 olmalı."
        )

    if wp_radius_m < 0.0 or lead_margin_m < 0.0:
        raise ValueError(
            "WP_RADIUS ve giriş payı negatif olamaz."
        )

    if mission_turn_step_m <= 0.0:
        raise ValueError(
            "Dönüş waypoint aralığı pozitif olmalı."
        )

    if max_waypoints < 2:
        raise ValueError("Waypoint sınırı geçersiz.")

    if safe_swath_m <= 0.0:
        raise ValueError(
            "Güvenli kamera tarama genişliği pozitif olmalı."
        )

    if turn_corridor_half_width_m <= 0.0:
        raise ValueError(
            "Rota koridor genişliği pozitif olmalı."
        )

    if len(polygon_xy) < 3:
        raise ValueError(
            "Tarama poligonu en az üç köşe içermeli."
        )

    polygon = Polygon(polygon_xy)

    if not polygon.is_valid or polygon.area <= 0.25:
        raise ValueError("Tarama poligonu geçersiz.")

    # Tam görüntünün odak uzaklığını piksel cinsinden hesapla.
    # ROI'nin yerdeki genişliğini yalnızca bilgi amaçlı hesaplıyoruz.
    # Fixed-wing tarama hatlarını artık ROI değil, tam görüntü oluşturacak.
    focal_px = (
        frame_width_px / 2.0
    ) / math.tan(
        math.radians(horizontal_fov_deg) / 2.0
    )

    roi_footprint_m = (
        altitude_m
        * roi_size_px
        / focal_px
    )

    # Poligonun taranmasında kullanılan esas kamera genişliği.
    footprint = float(
        full_frame_swath_m
    )

    max_spacing = float(
        lane_spacing_m
    )

    actual_overlap_ratio = (
        1.0
        - max_spacing / footprint
    )

    # R = V² / (g × tan(bank))
    theoretical_radius = (
        planning_speed_mps ** 2
    ) / (
        9.80665
        * math.tan(math.radians(design_bank_deg))
    )

    turn_radius = (
        theoretical_radius
        * turn_radius_safety
    )

        # Kırmızı sınırdan sonra kısa bir düz çıkış bırakılır.
    # Dönüşün asıl genişliğini Dubins geometrisi oluşturacaktır.
    lead_distance = float(
        lead_margin_m
    )

    if allowed_turn_margin_m is None:
        allowed_turn_margin_m = max(
            3.50 * turn_radius,
            lead_distance + turn_radius + 5.0,
        )

    allowed_turn_margin_m = float(
        allowed_turn_margin_m
    )

    if allowed_turn_margin_m <= lead_distance:
        raise ValueError(
            "İzin verilen dönüş alanı, düz giriş mesafesinden küçük."
        )

    # safe_swath_m zaten bindirme payı düşürülmüş güvenli genişliktir.
    # Bu nedenle satır üretiminde tekrar overlap uygulanmaz.
    candidates = [
        item
        for item in scan_plan_candidates(
            polygon_xy=polygon_xy,
            cross_width=full_frame_swath_m,
            along_width=along_track_swath_m,
            overlap=0.0,
            lane_spacing_m=lane_spacing_m,
        )
        if item["uncovered"].area <= 0.05
    ]

    if not candidates:
        raise ValueError(
            "Poligonu tamamen kaplayan tarama açısı bulunamadı."
        )

    minimum_passes = candidates[0]["pass_count"]

    # Sadece en az hatlı açı değil, bir fazla hat kullanan fakat çok daha
    # düzgün dönen açılar da değerlendirilsin.
    finalists = [
        item
        for item in candidates
        if item["pass_count"] <= minimum_passes + 1
    ][:18]

    finalist_results = []

    for candidate in finalists:
        passes = make_oriented_passes(
            candidate,
            lead_distance,
            exit_lead_distance=lead_distance,
        )

        try:
            fixed_wing = _build_plan_from_passes(
                passes,
                polygon,
                turn_radius,
                allowed_turn_margin_m,
                mission_turn_step_m,
                turn_corridor_half_width_m,
                home_xy=home_xy,
            )

        except ValueError:
            continue

        score = (
            fixed_wing["route_length_m"]
            + 1.35 * fixed_wing["outside_length_m"]
            + 3.0 * fixed_wing["max_outside_m"]
            + 25.0 * (
                candidate["pass_count"]
                - minimum_passes
            )
            + fixed_wing["order_info"][
                "home_start_distance"
            ]
        )

        finalist_results.append(
            {
                "score": score,
                "candidate": candidate,
                "passes": passes,
                "fixed_wing": fixed_wing,
            }
        )

    if not finalist_results:
        raise ValueError(
            "Hesaplanan izinli dönüş sınırına sığan "
            "sabit-kanat rotası bulunamadı."
        )

    selected = min(
        finalist_results,
        key=lambda item: item["score"],
    )

    candidate = selected["candidate"]
    angle = candidate["angle"]
    fixed_wing = selected["fixed_wing"]
    route_points = fixed_wing["route_points"]

    if len(route_points) > max_waypoints:
        raise ValueError(
            f"Rota {len(route_points)} waypoint gerektiriyor; "
            f"sınır {max_waypoints}."
        )

    coverage_original = shapely_rotate(
        candidate["coverage"],
        angle,
        origin=(0.0, 0.0),
        use_radians=True,
    )

    coverage_rings = _geometry_polygon_rings(
        coverage_original
    )

    total_length = fixed_wing["route_length_m"]

    return {
        "route_points": route_points,
        "scan_lines": fixed_wing["scan_lines"],
        "scan_passes": fixed_wing["scan_passes"],
        "lead_lines": fixed_wing["lead_lines"],
        "turn_lines": fixed_wing["turn_lines"],
        "route_corridor_rings": (
            fixed_wing["route_corridor_rings"]
        ),
        "allowed_polygon_rings": (
            fixed_wing["allowed_polygon_rings"]
        ),
        "coverage_rings": coverage_rings,

        "summary": {
            "flight_mode": "FIXED_WING",
            "angle_deg": (
                math.degrees(angle) % 180.0
            ),
            "lane_count": len(candidate["rows"]),
            "scan_pass_count": candidate["pass_count"],
            "turn_count": max(
                0,
                candidate["pass_count"] - 1,
            ),
            "waypoint_count": len(route_points),

            # Kamera sonuçları
            "footprint_m": footprint,
            "roi_footprint_m": roi_footprint_m,
            "max_lane_spacing_m": max_spacing,

            # Eski YKİ logları bozulmasın diye anahtar şimdilik korunuyor.
            "safe_swath_m": max_spacing,
            "actual_overlap_ratio": actual_overlap_ratio,
            "actual_overlap_percent": (
                actual_overlap_ratio * 100.0
            ),

            # Sabit-kanat sonuçları
            "turn_radius_m": turn_radius,
            "theoretical_turn_radius_m": (
                theoretical_radius
            ),
            "lead_distance_m": lead_distance,
            "turn_control_points_per_turn": 2,
            "max_outside_distance_m": (
                fixed_wing["max_outside_m"]
            ),
            "outside_red_route_length_m": (
                fixed_wing["outside_length_m"]
            ),
            "allowed_turn_margin_m": (
                allowed_turn_margin_m
            ),

            # HOME başlangıç/bitiş bilgileri
            "home_entry_distance_m": (
                fixed_wing["order_info"][
                    "home_start_distance"
                ]
            ),
            "home_exit_distance_m": (
                fixed_wing["order_info"][
                    "home_end_distance"
                ]
            ),

            "uncovered_area_m2": (
                candidate["uncovered"].area
            ),
            "route_length_m": total_length,
            "estimated_time_s": (
                total_length / planning_speed_mps
            ),
            "design_bank_deg": design_bank_deg,
            "planning_speed_mps": planning_speed_mps,
            "plan_verified": True,
        },
    }

def build_recovery_plan(
    polygon_xy,
    scan_lines_xy,
    current_pose_xy,
    altitude_m=20.0,
    planning_speed_mps=15.0,
    design_bank_deg=40.0,
    turn_radius_safety=1.20,
    wp_radius_m=9.0,
    lead_margin_m=5.0,
    safe_swath_m=5.0,
    mission_turn_step_m=22.5,
    allowed_turn_margin_m=None,
    turn_corridor_half_width_m=7.5,
):
    """
    Yalnızca eksik kalan tarama hatları için yeni sabit-kanat
    recovery rotası oluşturur.

    current_pose_xy:
        (x, y, yaw_radyan)
    """
    if shapely_rotate is None:
        raise RuntimeError(
            "Shapely bulunamadı. "
            "'python -m pip install shapely' çalıştırın."
        ) from SHAPELY_IMPORT_ERROR

    polygon = Polygon(polygon_xy)

    if not polygon.is_valid or polygon.area <= 0.25:
        raise ValueError("Recovery poligonu geçersiz.")

    if not scan_lines_xy:
        raise ValueError(
            "Recovery için en az bir eksik tarama hattı gerekli."
        )

    theoretical_radius = (
        planning_speed_mps ** 2
    ) / (
        9.80665
        * math.tan(math.radians(design_bank_deg))
    )

    turn_radius = (
        theoretical_radius
        * turn_radius_safety
    )

    lead_distance = max(
        safe_swath_m / 2.0
        + wp_radius_m
        + lead_margin_m,

        turn_radius * 0.60,
    )

    if allowed_turn_margin_m is None:
        allowed_turn_margin_m = max(
            3.50 * turn_radius,
            lead_distance + turn_radius + 5.0,
        )

    scan_passes = []

    for line_index, line in enumerate(scan_lines_xy):
        if len(line) != 2:
            continue

        (x1, y1), (x2, y2) = line

        length = math.hypot(
            x2 - x1,
            y2 - y1,
        )

        if length < 0.50:
            continue

        ux = (x2 - x1) / length
        uy = (y2 - y1) / length
        angle = math.atan2(uy, ux)

        scan_passes.append(
            {
                "row_index": line_index,
                "segment_index": 0,

                "scan_left": (
                    float(x1),
                    float(y1),
                ),
                "scan_right": (
                    float(x2),
                    float(y2),
                ),

                "lead_left": (
                    float(x1 - ux * lead_distance),
                    float(y1 - uy * lead_distance),
                ),
                "lead_right": (
                    float(x2 + ux * lead_distance),
                    float(y2 + uy * lead_distance),
                ),

                "angle": angle,
            }
        )

    if not scan_passes:
        raise ValueError(
            "Geçerli recovery tarama hattı oluşturulamadı."
        )

    # home_xy yerine uçağın gerçek konumu ve yönü kullanılır.
    fixed_wing = _build_plan_from_passes(
        scan_passes,
        polygon,
        turn_radius,
        float(allowed_turn_margin_m),
        mission_turn_step_m,
        turn_corridor_half_width_m,
        start_pose=current_pose_xy,
    )

    fixed_wing["summary"] = {
        "flight_mode": "FIXED_WING_RECOVERY",
        "scan_pass_count": len(
            fixed_wing["scan_passes"]
        ),
        "waypoint_count": len(
            fixed_wing["route_points"]
        ),
        "safe_swath_m": safe_swath_m,
        "turn_radius_m": turn_radius,
        "lead_distance_m": lead_distance,
        "max_outside_distance_m": (
            fixed_wing["max_outside_m"]
        ),
        "outside_red_route_length_m": (
            fixed_wing["outside_length_m"]
        ),
        "route_length_m": (
            fixed_wing["route_length_m"]
        ),
        "estimated_time_s": (
            fixed_wing["route_length_m"]
            / planning_speed_mps
        ),
        "allowed_turn_margin_m": float(
            allowed_turn_margin_m
        ),
        "plan_verified": True,
    }

    return fixed_wing

def _angle_difference_deg(first, second):
    """
    İki pusula açısı arasındaki en küçük farkı döndürür.

    Örnek:
        359° ile 1° arasındaki fark 358° değil, 2° olmalıdır.
    """
    return abs(
        (
            float(first)
            - float(second)
            + 180.0
        ) % 360.0
        - 180.0
    )

class SurveyCoverageTracker:
    """
    Gerçek uçuş telemetrisini kullanarak kırmızı poligonun hangi
    kısımlarının güvenli biçimde tarandığını hesaplar.
    """

    def __init__(
        self,
        polygon_xy,
        scan_passes,
        safe_swath_m=5.0,
        target_altitude_m=20.0,
        cross_track_limit_m=1.0,
        heading_limit_deg=8.0,
        roll_limit_deg=10.0,
        altitude_limit_m=3.0,
        minimum_gps_fix=3,
        maximum_sample_gap_m=18.0,
        minimum_alignment_samples=2,
        maximum_lead_observation_m=45.0,
    ):
        self.polygon = Polygon(polygon_xy)

        if (
            not self.polygon.is_valid
            or self.polygon.area <= 0.25
        ):
            raise ValueError(
                "Kapsama takibi için poligon geçersiz."
            )

        self.scan_passes = list(scan_passes)
        self.safe_swath_m = float(safe_swath_m)
        self.half_swath_m = self.safe_swath_m / 2.0

        self.target_altitude_m = float(
            target_altitude_m
        )
        self.cross_track_limit_m = float(
            cross_track_limit_m
        )
        self.heading_limit_deg = float(
            heading_limit_deg
        )
        self.roll_limit_deg = float(
            roll_limit_deg
        )
        self.altitude_limit_m = float(
            altitude_limit_m
        )
        self.minimum_gps_fix = int(
            minimum_gps_fix
        )
        self.maximum_sample_gap_m = float(
            maximum_sample_gap_m
        )
        self.minimum_alignment_samples = int(
            minimum_alignment_samples
        )
        self.maximum_lead_observation_m = float(
            maximum_lead_observation_m
        )

        self.lines = [
            LineString(
                [
                    item["start"],
                    item["end"],
                ]
            )
            for item in self.scan_passes
        ]

        # Her sarı hattın güvenli kamera şeridi.
        # 5 m için hattın iki tarafına 2.5 m açılır.
        self.pass_strips = [
            line.buffer(
                self.half_swath_m,
                cap_style=2,
            ).intersection(self.polygon)
            for line in self.lines
        ]

        self.covered = GeometryCollection()
        self.actual_track = []

        self.last_valid_by_pass = {}
        self.last_alignment_by_pass = {}
        self.alignment_streak_by_pass = {}

        self.accepted_samples = 0
        self.rejected_samples = 0
        self.last_result = None

    def reset(self):
        self.covered = GeometryCollection()
        self.actual_track = []

        self.last_valid_by_pass = {}
        self.last_alignment_by_pass = {}
        self.alignment_streak_by_pass = {}

        self.accepted_samples = 0
        self.rejected_samples = 0
        self.last_result = None

    def update(
        self,
        x,
        y,
        heading_deg,
        roll_deg,
        altitude_m,
        gps_fix,
    ):
        """
        Bir telemetri örneğini değerlendirir.

        Örneğin kapsama kabul edilmesi için:
        - GPS fix >= 3
        - roll <= 10°
        - irtifa hatası <= 3 m
        - sarı hattan sapma <= 1 m
        - heading hatası <= 8°
        - art arda en az 2 uygun örnek
        - uçağın kırmızı poligon içinde olması

        gerekir.
        """
        x = float(x)
        y = float(y)

        point = Point(x, y)
        line_metrics = []

        for index, pass_info in enumerate(
            self.scan_passes
        ):
            x1, y1 = pass_info["start"]
            x2, y2 = pass_info["end"]

            vx = x2 - x1
            vy = y2 - y1
            length = math.hypot(vx, vy)

            along_ratio = (
                (
                    (x - x1) * vx
                    + (y - y1) * vy
                )
                / (length * length)
            )

            cross_track = abs(
                vx * (y - y1)
                - vy * (x - x1)
            ) / length

            # Nokta hattın başlangıç veya bitişinin ne kadar dışında?
            outside_along = max(
                0.0,
                -along_ratio * length,
                (along_ratio - 1.0) * length,
            )

            line_metrics.append(
                (
                    cross_track
                    + 0.05 * outside_along,
                    index,
                    cross_track,
                    outside_along,
                )
            )

        (
            _,
            nearest_index,
            cross_track,
            outside_along,
        ) = min(line_metrics)

        pass_info = self.scan_passes[nearest_index]

        dx = (
            pass_info["end"][0]
            - pass_info["start"][0]
        )
        dy = (
            pass_info["end"][1]
            - pass_info["start"][1]
        )

        # x=doğu, y=kuzey koordinatlarını pusula headingine çevirir.
        expected_heading = (
            math.degrees(math.atan2(dx, dy))
            % 360.0
        )

        # Aynı hattı ters yönde uçmak da kamera kapsaması açısından geçerli.
        heading_error = min(
            _angle_difference_deg(
                heading_deg,
                expected_heading,
            ),
            _angle_difference_deg(
                heading_deg,
                (
                    expected_heading + 180.0
                ) % 360.0,
            ),
        )

        altitude_error = abs(
            float(altitude_m)
            - self.target_altitude_m
        )

        alignment_valid = (
            int(gps_fix) >= self.minimum_gps_fix
            and abs(float(roll_deg))
            <= self.roll_limit_deg
            and altitude_error
            <= self.altitude_limit_m
            and cross_track
            <= self.cross_track_limit_m
            and heading_error
            <= self.heading_limit_deg
            and outside_along
            <= self.maximum_lead_observation_m
        )

        if alignment_valid:
            self.alignment_streak_by_pass[
                nearest_index
            ] = (
                self.alignment_streak_by_pass.get(
                    nearest_index,
                    0,
                )
                + 1
            )
        else:
            self.alignment_streak_by_pass[
                nearest_index
            ] = 0

        alignment_streak = (
            self.alignment_streak_by_pass.get(
                nearest_index,
                0,
            )
        )

        aligned_long_enough = (
            alignment_streak
            >= self.minimum_alignment_samples
        )

        inside_red = self.polygon.buffer(
            0.10
        ).covers(point)

        valid = (
            alignment_valid
            and aligned_long_enough
            and inside_red
        )

        result = {
            "valid": bool(valid),
            "alignment_valid": bool(
                alignment_valid
            ),
            "alignment_streak": (
                alignment_streak
            ),
            "scan_pass": nearest_index,
            "cross_track_m": float(
                cross_track
            ),
            "heading_error_deg": float(
                heading_error
            ),
            "altitude_error_m": float(
                altitude_error
            ),
        }

        self.actual_track.append(
            {
                "x": x,
                "y": y,
                "valid": bool(valid),
                "aligned": bool(
                    alignment_valid
                    and aligned_long_enough
                ),
                "scan_pass": nearest_index,
            }
        )

        previous_alignment = (
            self.last_alignment_by_pass.get(
                nearest_index
            )
        )

        if alignment_valid:
            self.last_alignment_by_pass[
                nearest_index
            ] = (x, y)

        if not valid:
            self.rejected_samples += 1
            self.last_result = result
            return result

        self.accepted_samples += 1

        previous = self.last_valid_by_pass.get(
            nearest_index
        )

        # İlk geçerli örnekte bir önceki hizalı örneği kullanarak
        # hattın başlangıcında küçük bir yapay boşluk bırakmayı önler.
        if (
            previous_alignment is not None
            and (
                previous is None
                or math.hypot(
                    x - previous[0],
                    y - previous[1],
                ) > self.maximum_sample_gap_m
            )
        ):
            previous = previous_alignment

        if (
            previous is not None
            and math.hypot(
                x - previous[0],
                y - previous[1],
            ) <= self.maximum_sample_gap_m
        ):
            sample_geometry = LineString(
                [
                    previous,
                    (x, y),
                ]
            ).buffer(
                self.half_swath_m,
                cap_style=2,
            )

        else:
            sample_geometry = point.buffer(
                self.half_swath_m
            )

        # Uçağın izi yalnızca ait olduğu sarı tarama şeridi içinde ve
        # kırmızı poligon sınırları içerisinde kapsama sayılır.
        sample_geometry = (
            sample_geometry
            .intersection(
                self.pass_strips[nearest_index]
            )
            .intersection(self.polygon)
        )

        self.covered = self.covered.union(
            sample_geometry
        )

        self.last_valid_by_pass[
            nearest_index
        ] = (x, y)

        self.last_result = result
        return result

    def missing_geometry(self):
        return self.polygon.difference(
            self.covered
        )

    def missing_area_m2(self):
        return float(
            self.missing_geometry().area
        )

    def completion_percent(self):
        if self.polygon.area <= 0.0:
            return 0.0

        covered_area = self.covered.intersection(
            self.polygon
        ).area

        ratio = covered_area / self.polygon.area

        return 100.0 * max(
            0.0,
            min(1.0, ratio),
        )

    def recovery_scan_line_indexes(
        self,
        minimum_missing_area_m2=0.35
    ):
        """
        Eksik alanla kesişen tarama şeritlerinin indekslerini döndürür.
        """
        missing = self.missing_geometry()
        result = []

        for index, strip in enumerate(
            self.pass_strips
        ):
            missing_area = missing.intersection(
                strip
            ).area

            if (
                missing_area
                >= minimum_missing_area_m2
            ):
                result.append(index)

        return result

    def snapshot(self):
        """
        YKİ ve harita katmanının kullanacağı anlık kapsama özetini üretir.
        """
        missing = self.missing_geometry()

        return {
            "covered_area_m2": float(
                self.covered
                .intersection(self.polygon)
                .area
            ),
            "missing_area_m2": float(
                missing.area
            ),
            "completion_percent": (
                self.completion_percent()
            ),
            "accepted_samples": (
                self.accepted_samples
            ),
            "rejected_samples": (
                self.rejected_samples
            ),
            "covered_rings": (
                _geometry_polygon_rings(
                    self.covered
                )
            ),
            "missing_rings": (
                _geometry_polygon_rings(
                    missing
                )
            ),
            "recovery_scan_line_indexes": (
                self.recovery_scan_line_indexes()
            ),
            "actual_track": list(
                self.actual_track
            ),
        }

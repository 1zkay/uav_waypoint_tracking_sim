#!/usr/bin/env python3

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml


GENERATED_MODEL_NAMES = {
    "mission_reference_yard",
    "waypoint_markers",
    "planned_route_altitude_line",
    "wind_direction_indicator",
}


Waypoint = tuple[float, float, float]
Point3 = tuple[float, float, float]
Color = tuple[float, float, float, float]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Render Gazebo waypoint visuals from a PX4 local NED waypoint YAML file."
    )
    parser.add_argument("--base-world", required=True, type=Path)
    parser.add_argument("--waypoints", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--wind-config", type=Path)
    args = parser.parse_args()

    config = load_waypoint_config(args.waypoints)
    waypoints = parse_waypoints(config)
    acceptance_radius = float(config.get("acceptance_radius_m", 1.0))
    wind_config = load_wind_config(args.wind_config)

    tree = ET.parse(args.base_world)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError(f"World element not found in {args.base_world}")

    remove_generated_models(world)
    configure_wind(world, wind_config)
    insert_index = find_insert_index(world)
    for model in make_generated_models(waypoints, acceptance_radius, wind_config):
        world.insert(insert_index, model)
        insert_index += 1

    ET.indent(tree, space="  ")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding="utf-8", xml_declaration=False)
    args.output.write_text(args.output.read_text(encoding="utf-8") + "\n", encoding="utf-8")


def load_waypoint_config(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"Waypoint config does not exist: {path}")

    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}

    if str(config.get("frame", "NED")).upper() != "NED":
        raise ValueError("Only PX4 local NED waypoints are supported.")

    return config


def load_wind_config(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {"enabled": False}
    if not path.exists():
        raise FileNotFoundError(f"Wind config does not exist: {path}")

    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}

    frame = str(config.get("frame", "ENU")).upper()
    if frame != "ENU":
        raise ValueError("Only Gazebo ENU wind vectors are supported.")

    return config


def parse_waypoints(config: dict[str, Any]) -> list[Waypoint]:
    raw_waypoints = config.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("Config must contain a non-empty 'waypoints' list.")

    waypoints: list[Waypoint] = []
    for index, waypoint in enumerate(raw_waypoints):
        if not isinstance(waypoint, list | tuple) or len(waypoint) != 3:
            raise ValueError(f"Waypoint {index} must be [x, y, z].")
        waypoints.append(tuple(float(value) for value in waypoint))

    return waypoints


def remove_generated_models(world: ET.Element) -> None:
    for model in list(world.findall("model")):
        if model.get("name") in GENERATED_MODEL_NAMES:
            world.remove(model)


def configure_wind(world: ET.Element, config: dict[str, Any]) -> None:
    for wind in list(world.findall("wind")):
        world.remove(wind)

    for plugin in list(world.findall("plugin")):
        if plugin.get("name") == "gz::sim::systems::WindEffects":
            world.remove(plugin)

    if not bool(config.get("enabled", False)):
        return

    wind = ET.Element("wind")
    ET.SubElement(wind, "linear_velocity").text = fmt_triplet(parse_triplet(config, "linear_velocity_mps"))

    plugin = make_wind_effects_plugin(config.get("wind_effects", {}))

    insert_index = find_wind_insert_index(world)
    world.insert(insert_index, wind)
    world.insert(insert_index + 1, plugin)


def find_wind_insert_index(world: ET.Element) -> int:
    for index, child in enumerate(list(world)):
        if child.tag == "spherical_coordinates":
            return index
    return len(world)


def find_insert_index(world: ET.Element) -> int:
    for index, child in enumerate(list(world)):
        if child.tag in {"light", "spherical_coordinates"}:
            return index
    return len(world)


def make_generated_models(
    waypoints: list[Waypoint], acceptance_radius: float, wind_config: dict[str, Any]
) -> list[ET.Element]:
    points = [ned_to_gazebo(waypoint) for waypoint in waypoints]
    models = [
        make_reference_yard(points),
        make_waypoint_markers(points, acceptance_radius),
        make_route_line(points),
    ]
    if bool(wind_config.get("enabled", False)):
        models.append(make_wind_indicator(points, wind_config))
    return models


def make_reference_yard(points: list[Point3]) -> ET.Element:
    min_x, max_x, min_y, max_y = xy_bounds(points, margin=2.0)
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    width = max_x - min_x
    depth = max_y - min_y
    first = points[0]

    model = ET.Element("model", {"name": "mission_reference_yard"})
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", {"name": "link"})

    add_box_visual(link, "north_boundary", (center_x, max_y, 0.04), (width, 0.08, 0.08), dark_gray())
    add_box_visual(link, "south_boundary", (center_x, min_y, 0.04), (width, 0.08, 0.08), dark_gray())
    add_box_visual(link, "west_boundary", (min_x, center_y, 0.04), (0.08, depth, 0.08), dark_gray())
    add_box_visual(link, "east_boundary", (max_x, center_y, 0.04), (0.08, depth, 0.08), dark_gray())
    add_cylinder_visual(link, "takeoff_pad", (first[0], first[1], 0.015), 1.25, 0.03, (0.08, 0.45, 0.18, 1.0))

    add_box_visual(
        link,
        "reference_block_east",
        (max_x + 2.0, center_y - 2.0, 0.65),
        (1.4, 1.0, 1.3),
        (0.55, 0.58, 0.60, 1.0),
    )
    add_box_visual(
        link,
        "reference_block_north",
        (center_x, max_y + 2.0, 0.45),
        (1.2, 1.2, 0.9),
        (0.38, 0.48, 0.58, 1.0),
    )
    add_box_visual(
        link,
        "reference_block_west",
        (min_x - 2.0, center_y + 2.0, 0.55),
        (1.0, 1.8, 1.1),
        (0.58, 0.50, 0.38, 1.0),
    )

    return model


def make_waypoint_markers(points: list[Point3], acceptance_radius: float) -> ET.Element:
    model = ET.Element("model", {"name": "waypoint_markers"})
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", {"name": "link"})

    for index, point in enumerate(points):
        color = waypoint_color(index, len(points))
        marker_name = f"wp{index + 1:03d}"
        mast_height = max(point[2], 0.2)
        add_cylinder_visual(
            link,
            f"{marker_name}_mast",
            (point[0], point[1], mast_height / 2.0),
            0.045,
            mast_height,
            color,
        )
        add_sphere_visual(link, f"{marker_name}_target", point, 0.36, color)
        add_cylinder_visual(
            link,
            f"{marker_name}_acceptance",
            point,
            acceptance_radius,
            0.035,
            (color[0], color[1], color[2], 0.22),
        )

    return model


def make_route_line(points: list[Point3]) -> ET.Element:
    model = ET.Element("model", {"name": "planned_route_altitude_line"})
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", {"name": "link"})

    for index, (start, end) in enumerate(zip(points, points[1:]), start=1):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dz = end[2] - start[2]
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length < 1e-6:
            continue

        midpoint = (
            (start[0] + end[0]) / 2.0,
            (start[1] + end[1]) / 2.0,
            (start[2] + end[2]) / 2.0,
        )
        yaw = math.atan2(dy, dx)
        horizontal = math.sqrt(dx * dx + dy * dy)
        pitch = -math.atan2(dz, horizontal)
        add_box_visual(
            link,
            f"segment_{index:03d}",
            midpoint,
            (length, 0.08, 0.08),
            (0.0, 0.45, 1.0, 0.8),
            rpy=(0.0, pitch, yaw),
        )

    return model


def make_wind_indicator(points: list[Point3], config: dict[str, Any]) -> ET.Element:
    min_x, max_x, min_y, _ = xy_bounds(points, margin=2.0)
    wind = parse_triplet(config, "linear_velocity_mps")
    speed = math.sqrt(wind[0] * wind[0] + wind[1] * wind[1] + wind[2] * wind[2])
    horizontal_speed = math.sqrt(wind[0] * wind[0] + wind[1] * wind[1])

    model = ET.Element("model", {"name": "wind_direction_indicator"})
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", {"name": "link"})

    origin = (min_x, min_y - 1.6, 0.25)
    if horizontal_speed < 1e-6:
        add_sphere_visual(link, "calm_marker", origin, 0.28, wind_color())
        return model

    length = max(1.6, min(4.0, speed))
    yaw = math.atan2(wind[1], wind[0])
    shaft_center = (
        origin[0] + math.cos(yaw) * length * 0.5,
        origin[1] + math.sin(yaw) * length * 0.5,
        origin[2],
    )
    head_center = (
        origin[0] + math.cos(yaw) * (length + 0.25),
        origin[1] + math.sin(yaw) * (length + 0.25),
        origin[2],
    )

    add_box_visual(link, "shaft", shaft_center, (length, 0.12, 0.12), wind_color(), rpy=(0.0, 0.0, yaw))
    add_box_visual(link, "head", head_center, (0.5, 0.5, 0.18), wind_color(), rpy=(0.0, 0.0, yaw))
    return model


def make_wind_effects_plugin(config: dict[str, Any]) -> ET.Element:
    plugin = ET.Element(
        "plugin",
        {
            "name": "gz::sim::systems::WindEffects",
            "filename": "gz-sim-wind-effects-system",
        },
    )
    ET.SubElement(plugin, "force_approximation_scaling_factor").text = fmt(
        get_wind_float(config, [("force_approximation_scaling_factor",)], 1.0)
    )

    horizontal = ET.SubElement(plugin, "horizontal")
    magnitude = ET.SubElement(horizontal, "magnitude")
    ET.SubElement(magnitude, "time_for_rise").text = fmt(
        get_wind_float(
            config,
            [("horizontal", "magnitude", "time_for_rise"), ("horizontal", "magnitude", "time_for_rise_s")],
            10.0,
            legacy_keys=("magnitude_rise_time_s",),
        )
    )
    magnitude_sin = ET.SubElement(magnitude, "sin")
    ET.SubElement(magnitude_sin, "amplitude_percent").text = fmt(
        get_wind_float(
            config,
            [("horizontal", "magnitude", "sin", "amplitude_percent")],
            0.05,
            legacy_keys=("magnitude_sin_amplitude_percent",),
        )
    )
    ET.SubElement(magnitude_sin, "period").text = fmt(
        get_wind_float(
            config,
            [("horizontal", "magnitude", "sin", "period"), ("horizontal", "magnitude", "sin", "period_s")],
            60.0,
            legacy_keys=("magnitude_sin_period_s",),
        )
    )
    add_noise_from_config(
        magnitude,
        config,
        ("horizontal", "magnitude", "noise"),
        default_stddev=0.0002,
        legacy_stddev_key="magnitude_noise_stddev",
    )

    direction = ET.SubElement(horizontal, "direction")
    ET.SubElement(direction, "time_for_rise").text = fmt(
        get_wind_float(
            config,
            [("horizontal", "direction", "time_for_rise"), ("horizontal", "direction", "time_for_rise_s")],
            30.0,
            legacy_keys=("direction_rise_time_s",),
        )
    )
    direction_sin = ET.SubElement(direction, "sin")
    ET.SubElement(direction_sin, "amplitude").text = fmt(
        get_wind_float(
            config,
            [
                ("horizontal", "direction", "sin", "amplitude"),
                ("horizontal", "direction", "sin", "amplitude_rad"),
            ],
            0.3,
            legacy_keys=("direction_sin_amplitude", "direction_sin_amplitude_rad"),
        )
    )
    ET.SubElement(direction_sin, "period").text = fmt(
        get_wind_float(
            config,
            [("horizontal", "direction", "sin", "period"), ("horizontal", "direction", "sin", "period_s")],
            20.0,
            legacy_keys=("direction_sin_period_s",),
        )
    )
    add_noise_from_config(
        direction,
        config,
        ("horizontal", "direction", "noise"),
        default_stddev=0.03,
        legacy_stddev_key="direction_noise_stddev",
    )

    vertical = ET.SubElement(plugin, "vertical")
    add_noise_from_config(
        vertical,
        config,
        ("vertical", "noise"),
        default_stddev=0.03,
        legacy_stddev_key="vertical_noise_stddev",
    )
    return plugin


def get_wind_float(
    config: dict[str, Any],
    paths: list[tuple[str, ...]],
    default: float,
    *,
    legacy_keys: tuple[str, ...] = (),
) -> float:
    for path in paths:
        value = nested_value(config, path)
        if value is not None:
            return float(value)

    for key in legacy_keys:
        if key in config:
            return float(config[key])

    return default


def nested_value(config: dict[str, Any], path: tuple[str, ...]) -> Any:
    value: Any = config
    for key in path:
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def add_noise_from_config(
    parent: ET.Element,
    config: dict[str, Any],
    noise_path: tuple[str, ...],
    *,
    default_stddev: float,
    legacy_stddev_key: str,
) -> None:
    stddev = get_wind_float(config, [noise_path + ("stddev",)], default_stddev, legacy_keys=(legacy_stddev_key,))
    mean = get_wind_float(config, [noise_path + ("mean",)], 0.0)
    noise_type = nested_value(config, noise_path + ("type",))
    add_noise(parent, stddev, mean=mean, noise_type=str(noise_type or "gaussian"))


def add_noise(parent: ET.Element, stddev: float, *, mean: float = 0.0, noise_type: str = "gaussian") -> None:
    noise = ET.SubElement(parent, "noise", {"type": noise_type})
    ET.SubElement(noise, "mean").text = fmt(mean)
    ET.SubElement(noise, "stddev").text = fmt(stddev)


def parse_triplet(config: dict[str, Any], key: str) -> Point3:
    values = config.get(key, (0.0, 0.0, 0.0))
    if not isinstance(values, list | tuple) or len(values) != 3:
        raise ValueError(f"Wind config '{key}' must be [x, y, z].")
    return tuple(float(value) for value in values)


def xy_bounds(points: list[Point3], margin: float) -> tuple[float, float, float, float]:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin


def waypoint_color(index: int, count: int) -> Color:
    if index == 0 or index == count - 1:
        return (0.08, 0.75, 0.28, 1.0)
    return (0.05, 0.35, 1.0, 1.0)


def dark_gray() -> Color:
    return (0.15, 0.15, 0.15, 1.0)


def wind_color() -> Color:
    return (0.1, 0.55, 0.9, 1.0)


def ned_to_gazebo(waypoint: Waypoint) -> Point3:
    x_north, y_east, z_down = waypoint
    return (y_east, x_north, -z_down)


def add_box_visual(
    link: ET.Element,
    name: str,
    pose_xyz: Point3,
    size_xyz: Point3,
    color: Color,
    *,
    rpy: Point3 = (0.0, 0.0, 0.0),
) -> None:
    visual = add_visual(link, name, pose_xyz, color, rpy=rpy)
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = fmt_triplet(size_xyz)


def add_cylinder_visual(
    link: ET.Element,
    name: str,
    pose_xyz: Point3,
    radius: float,
    length: float,
    color: Color,
) -> None:
    visual = add_visual(link, name, pose_xyz, color)
    geometry = ET.SubElement(visual, "geometry")
    cylinder = ET.SubElement(geometry, "cylinder")
    ET.SubElement(cylinder, "radius").text = fmt(radius)
    ET.SubElement(cylinder, "length").text = fmt(length)


def add_sphere_visual(link: ET.Element, name: str, pose_xyz: Point3, radius: float, color: Color) -> None:
    visual = add_visual(link, name, pose_xyz, color)
    geometry = ET.SubElement(visual, "geometry")
    sphere = ET.SubElement(geometry, "sphere")
    ET.SubElement(sphere, "radius").text = fmt(radius)


def add_visual(
    link: ET.Element,
    name: str,
    pose_xyz: Point3,
    color: Color,
    *,
    rpy: Point3 = (0.0, 0.0, 0.0),
) -> ET.Element:
    visual = ET.SubElement(link, "visual", {"name": name})
    ET.SubElement(visual, "pose").text = fmt_triplet(pose_xyz + rpy)
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = fmt_triplet(color)
    ET.SubElement(material, "diffuse").text = fmt_triplet(color)
    return visual


def fmt_triplet(values: tuple[float, ...]) -> str:
    return " ".join(fmt(value) for value in values)


def fmt(value: float) -> str:
    return f"{value:.6g}"


if __name__ == "__main__":
    main()

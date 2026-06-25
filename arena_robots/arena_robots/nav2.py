"""Nav2 launch helpers."""

import tempfile
import typing
from pathlib import Path

import launch
import yaml
from arena_bringup.substitutions import YAMLFileSubstitution

from arena_robots.caps import MobileSpec, stringify_float_matrix
from arena_robots.Robot import ModelParams
from arena_robots.Sensor import SensorSpec, SensorType

_TYPE_TO_NAV2: dict[str, str] = {
    SensorType.LASERSCAN.value: "LaserScan",
    SensorType.POINTCLOUD.value: "PointCloud2",
}

# Fallback raytrace/obstacle range when caps/mobile.yaml carries no `laser:` block.
_DEFAULT_LIDAR_RANGE = 10.0


def compile_sensors_to_nav2(
    sensors: list[SensorSpec],
    *,
    max_range: float,
    obstacle_range_margin: float = 1.0,
    max_obstacle_height: float = 2.0,
    pointcloud_min_obstacle_height: float = 0.1,
    clearing: bool = True,
    marking: bool = True,
    inf_is_valid: bool = True,
    extra_per_source: dict[str, typing.Any] | None = None,
) -> dict[str, dict[str, typing.Any]]:
    """Compile SensorSpec entries with a nav2 costmap data_type into nav2's observation_sources_dict shape.

    `max_range` drives `raytrace_max_range` (clearing) so the costmap tracks the full
    sensor range rather than nav2's 3.0 m default. `obstacle_max_range` sits a margin
    below it: Isaac's 3D lidar emits phantom max-range points for missed rays, and a
    margin keeps those from leaking into the costmap as concentric arcs that no later
    raytrace ever clears. `inf_is_valid` lets no-return beams clear out to `max_range`.
    `pointcloud_min_obstacle_height` is a height floor applied to 3D cloud sources only,
    so their ground returns are dropped instead of marked (a flat LaserScan needs none).
    `extra_per_source` is merged onto every emitted source last, letting callers layer
    layer-specific tunables (e.g. `observation_persistence` for the global costmap).
    """
    obstacle_max_range = max(0.0, max_range - obstacle_range_margin)
    out: dict[str, dict[str, typing.Any]] = {}
    for spec in sensors:
        type_str = spec.type.value if isinstance(spec.type, SensorType) else str(spec.type)
        data_type = _TYPE_TO_NAV2.get(type_str)
        if data_type is None:
            continue
        source: dict[str, typing.Any] = {
            "topic": spec.topic,
            "data_type": data_type,
            "max_obstacle_height": max_obstacle_height,
            "clearing": clearing,
            "marking": marking,
            "obstacle_max_range": obstacle_max_range,
            "raytrace_max_range": max_range,
            "inf_is_valid": inf_is_valid,
        }
        if data_type == "PointCloud2":
            source["min_obstacle_height"] = pointcloud_min_obstacle_height
        if extra_per_source:
            source.update(extra_per_source)
        out[spec.name] = source
    return out


def _load_mobile(path_str: str) -> MobileSpec:
    with open(path_str) as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path_str}: mobile.yaml must be a mapping at top level")
    return MobileSpec(path=Path(path_str), raw=data)


class SensorsDerivedYAML(YAMLFileSubstitution):
    """Emit `observation_sources{,_string,_dict,_dict_global}` from sensors+laser range.

    Local uses the full lidar range; global uses shorter capped ranges and pulls per-source
    overrides (`raytrace_max_range`, `obstacle_max_range`, `observation_persistence`) from
    the optional `nav2.global_observation` block in caps/mobile.yaml.
    """

    _GLOBAL_DEFAULT_RAYTRACE = 6.0
    _GLOBAL_DEFAULT_OBSTACLE = 5.0
    _GLOBAL_DEFAULT_PERSISTENCE = 0.0

    def __init__(
        self,
        model_params_path: launch.SomeSubstitutionsType,
        mobile_path: launch.SomeSubstitutionsType,
    ):
        super().__init__(path=[], default={}, substitute=False)
        self._path = launch.utilities.normalize_to_list_of_substitutions(model_params_path)
        self._mobile_path = launch.utilities.normalize_to_list_of_substitutions(mobile_path)

    def perform(self, context: launch.LaunchContext) -> str:
        path_str = launch.utilities.perform_substitutions(context, self._path)
        mobile_str = launch.utilities.perform_substitutions(context, self._mobile_path)
        sensors = ModelParams.from_yaml(path_str).sensors
        mobile = _load_mobile(mobile_str)
        max_range = mobile.laser.range if mobile.laser is not None else _DEFAULT_LIDAR_RANGE

        local_sources = compile_sensors_to_nav2(sensors, max_range=max_range)

        overrides = mobile.raw.get('nav2', {}).get('global_observation', {}) or {}
        g_raytrace = float(overrides.get('raytrace_max_range', min(max_range, self._GLOBAL_DEFAULT_RAYTRACE)))
        g_obstacle = float(overrides.get('obstacle_max_range', min(g_raytrace, self._GLOBAL_DEFAULT_OBSTACLE)))
        g_persistence = float(overrides.get('observation_persistence', self._GLOBAL_DEFAULT_PERSISTENCE))
        global_sources = compile_sensors_to_nav2(
            sensors,
            max_range=g_raytrace,
            obstacle_range_margin=max(0.0, g_raytrace - g_obstacle),
            extra_per_source={'observation_persistence': g_persistence},
        )

        derived = {
            'observation_sources_string': ' '.join(local_sources.keys()),
            'observation_sources': list(local_sources.keys()),
            'observation_sources_dict': local_sources,
            'observation_sources_dict_global': global_sources,
        }
        tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(derived, tmp)
        tmp.close()
        return tmp.name


class Nav2SubBlockYAML(YAMLFileSubstitution):
    """Extract the `nav2:` sub-block from caps/mobile.yaml and emit it at top level
    as a temp YAML file. Lets YAMLMergeSubstitution treat adapter-specific config
    (footprint, polygons*, planner_plugins*) as flat merge-time keys while they
    stay nested in the authored file."""

    def __init__(self, mobile_path: launch.SomeSubstitutionsType):
        super().__init__(path=[], default={}, substitute=False)
        self._path = launch.utilities.normalize_to_list_of_substitutions(mobile_path)

    def perform(self, context: launch.LaunchContext) -> str:
        path_str = launch.utilities.perform_substitutions(context, self._path)
        raw = _load_mobile(path_str).sub('nav2')
        tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(raw, tmp)
        tmp.close()
        return tmp.name


class Nav2KinematicsDerivedYAML(YAMLFileSubstitution):
    """Emit controller-agnostic velocity/acceleration keys from the top-level
    ``velocity_limits``/``acceleration_limits`` in caps/mobile.yaml.

    These are the planner envelope (what nav2 may sample), not the hardware
    envelope (motor firmware / ``diff_drive_controller`` clip downstream).

    Controller plugin configs reference these via ``${max_linear_vel}`` etc.,
    letting each controller map the generic envelope onto its plugin-specific
    field names. Controllers wanting a lower cap can drop the ``${...}`` ref
    and hardcode the literal instead.
    """

    def __init__(self, mobile_path: launch.SomeSubstitutionsType):
        super().__init__(path=[], default={}, substitute=False)
        self._path = launch.utilities.normalize_to_list_of_substitutions(mobile_path)

    def perform(self, context: launch.LaunchContext) -> str:
        path_str = launch.utilities.perform_substitutions(context, self._path)
        mobile = _load_mobile(path_str)

        out: dict[str, typing.Any] = {}

        # Drive kinematics flag, so controller configs can switch holonomic mode
        # per-robot via ${is_holonomic} (e.g. Dynamic Gap's `holonomic:` field).
        out['is_holonomic'] = mobile.is_holonomic

        vel = mobile.velocity_limits
        if vel is not None:
            out['min_linear_vel'] = vel.linear.min
            out['max_linear_vel'] = vel.linear.max
            out['min_angular_vel'] = vel.angular.min
            out['max_angular_vel'] = vel.angular.max
            if vel.lateral is not None:
                out['min_lateral_vel'] = vel.lateral.min
                out['max_lateral_vel'] = vel.lateral.max

        acc = mobile.acceleration_limits
        if acc is not None:
            out['linear_acc'] = acc.linear
            out['angular_acc'] = acc.angular
            out['linear_decel'] = -acc.linear
            out['angular_decel'] = -acc.angular
            if acc.lateral is not None:
                out['lateral_acc'] = acc.lateral
                out['lateral_decel'] = -acc.lateral

        tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(out if out else {}, tmp)
        tmp.close()
        return tmp.name


class Nav2CollisionDerivedYAML(YAMLFileSubstitution):
    """Compile top-level `footprint` and `polygons_dict` from caps/mobile.yaml
    into the stringified form nav2's collision_monitor expects, overriding any
    raw float lists emitted by the preceding YAMLFileSubstitution(mobile_path)."""

    def __init__(self, mobile_path: launch.SomeSubstitutionsType):
        super().__init__(path=[], default={}, substitute=False)
        self._path = launch.utilities.normalize_to_list_of_substitutions(mobile_path)

    def perform(self, context: launch.LaunchContext) -> str:
        path_str = launch.utilities.perform_substitutions(context, self._path)
        mobile = _load_mobile(path_str)
        raw = mobile.raw

        out: dict[str, typing.Any] = {}

        footprint_raw = raw.get('footprint')
        if isinstance(footprint_raw, list):
            out['footprint'] = stringify_float_matrix([[float(c) for c in pt] for pt in footprint_raw])

        polygons_raw = raw.get('polygons_dict')
        if isinstance(polygons_raw, dict) and polygons_raw:
            out['polygons'] = list(polygons_raw.keys())
            compiled: dict[str, typing.Any] = {}
            for name, entry in polygons_raw.items():
                ptype = entry.get('type')
                polygon_entry: dict[str, typing.Any] = {}
                for field in ('type', 'action_type', 'polygon_pub_topic', 'min_points', 'visualize', 'enabled', 'slowdown_ratio'):
                    if field in entry:
                        polygon_entry[field] = entry[field]
                if ptype == 'polygon':
                    pts = entry.get('points')
                    if isinstance(pts, list):
                        polygon_entry['points'] = stringify_float_matrix([[float(c) for c in pt] for pt in pts])
                    else:
                        polygon_entry['points'] = pts
                elif ptype == 'circle':
                    if 'radius' in entry:
                        polygon_entry['radius'] = entry['radius']
                compiled[name] = polygon_entry
            out['polygons_dict'] = compiled

        tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(out if out else {}, tmp)
        tmp.close()
        return tmp.name

#!/usr/bin/env python3

import ast
import math
from dataclasses import dataclass
from typing import Any


Point3 = tuple[float, float, float]


_MATH_NAMES: dict[str, Any] = {
    "pi": math.pi,
    "tau": math.tau,
    "e": math.e,
    "sin": math.sin,
    "cos": math.cos,
    "tan": math.tan,
    "asin": math.asin,
    "acos": math.acos,
    "atan": math.atan,
    "atan2": math.atan2,
    "sqrt": math.sqrt,
    "hypot": math.hypot,
    "abs": abs,
    "min": min,
    "max": max,
    "pow": pow,
}


_ALLOWED_AST_NODES = (
    ast.Expression,
    ast.BinOp,
    ast.UnaryOp,
    ast.Call,
    ast.Name,
    ast.Load,
    ast.Constant,
    ast.Add,
    ast.Sub,
    ast.Mult,
    ast.Div,
    ast.Pow,
    ast.Mod,
    ast.USub,
    ast.UAdd,
)


class ScalarExpression:
    """Small numeric expression evaluator for trajectory YAML files."""

    def __init__(self, value: Any, name: str, allowed_variables: set[str]) -> None:
        self.text = str(value)
        self.name = name
        self.allowed_names = set(allowed_variables) | set(_MATH_NAMES)
        tree = ast.parse(self.text, mode="eval")
        self._validate(tree)
        self._code = compile(tree, f"<{name}>", "eval")

    def evaluate(self, variables: dict[str, float]) -> float:
        env = dict(_MATH_NAMES)
        env.update(variables)
        return float(eval(self._code, {"__builtins__": {}}, env))

    def _validate(self, tree: ast.AST) -> None:
        for node in ast.walk(tree):
            if not isinstance(node, _ALLOWED_AST_NODES):
                raise ValueError(f"{self.name} contains unsupported syntax: {type(node).__name__}")

            if isinstance(node, ast.Name) and node.id not in self.allowed_names:
                raise ValueError(f"{self.name} uses unknown name '{node.id}'.")

            if isinstance(node, ast.Call):
                if not isinstance(node.func, ast.Name) or node.func.id not in _MATH_NAMES:
                    raise ValueError(f"{self.name} may only call allowed math functions.")


@dataclass(frozen=True)
class ParametricTrajectory:
    duration_s: float
    constants: dict[str, float]
    position: tuple[ScalarExpression, ScalarExpression, ScalarExpression]
    velocity: tuple[ScalarExpression, ScalarExpression, ScalarExpression] | None
    yaw: ScalarExpression | None
    return_point: Point3 | None
    return_acceptance_radius_m: float
    visualization_samples: int

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "ParametricTrajectory":
        trajectory_type = str(config.get("trajectory_type", "parametric")).strip().lower()
        if trajectory_type != "parametric":
            raise ValueError("trajectory_type must be 'parametric'.")

        raw_curve = config.get("curve")
        if not isinstance(raw_curve, dict):
            raise ValueError("Config must contain a 'curve' mapping.")

        duration_s = positive_float(raw_curve.get("duration_s", config.get("duration_s")), "curve.duration_s")
        constants = parse_constants(raw_curve.get("constants", {}))
        allowed_variables = set(constants) | {"t", "u"}

        raw_position = raw_curve.get("position")
        if not isinstance(raw_position, dict):
            raise ValueError("curve.position must be a mapping with x, y, z expressions.")
        position = (
            ScalarExpression(required(raw_position, "x", "curve.position"), "curve.position.x", allowed_variables),
            ScalarExpression(required(raw_position, "y", "curve.position"), "curve.position.y", allowed_variables),
            ScalarExpression(required(raw_position, "z", "curve.position"), "curve.position.z", allowed_variables),
        )

        raw_velocity = raw_curve.get("velocity")
        velocity = None
        if raw_velocity is not None:
            if not isinstance(raw_velocity, dict):
                raise ValueError("curve.velocity must be a mapping with x, y, z expressions.")
            velocity = (
                ScalarExpression(required(raw_velocity, "x", "curve.velocity"), "curve.velocity.x", allowed_variables),
                ScalarExpression(required(raw_velocity, "y", "curve.velocity"), "curve.velocity.y", allowed_variables),
                ScalarExpression(required(raw_velocity, "z", "curve.velocity"), "curve.velocity.z", allowed_variables),
            )

        yaw = None
        if "yaw" in raw_curve:
            yaw = ScalarExpression(raw_curve["yaw"], "curve.yaw", allowed_variables)

        return_config = config.get("return", {})
        if return_config is None:
            return_config = {}
        if not isinstance(return_config, dict):
            raise ValueError("return must be a mapping.")
        return_enabled = bool(return_config.get("enabled", False))
        return_point = None
        if return_enabled:
            return_point = parse_point(required(return_config, "point", "return"), "return.point")

        return_acceptance_radius_m = positive_float(
            return_config.get("acceptance_radius_m", config.get("acceptance_radius_m", 1.0)),
            "return.acceptance_radius_m",
        )
        visualization_samples = max(2, int(config.get("visualization_samples", 160)))

        return cls(
            duration_s=duration_s,
            constants=constants,
            position=position,
            velocity=velocity,
            yaw=yaw,
            return_point=return_point,
            return_acceptance_radius_m=return_acceptance_radius_m,
            visualization_samples=visualization_samples,
        )

    @property
    def has_velocity(self) -> bool:
        return self.velocity is not None

    def sample(self, t_s: float) -> tuple[Point3, Point3 | None, float | None]:
        t = min(max(0.0, t_s), self.duration_s)
        variables = self._variables(t)
        position = tuple(expression.evaluate(variables) for expression in self.position)
        velocity = None
        if self.velocity is not None:
            velocity = tuple(expression.evaluate(variables) for expression in self.velocity)
        yaw = self.yaw.evaluate(variables) if self.yaw is not None else None
        return position, velocity, yaw

    def sample_points(self, *, include_return: bool = True) -> list[Point3]:
        points = [self.sample(self.duration_s * index / self.visualization_samples)[0] for index in range(self.visualization_samples + 1)]
        if include_return and self.return_point is not None:
            points.append(self.return_point)
        return points

    def _variables(self, t_s: float) -> dict[str, float]:
        variables = dict(self.constants)
        variables["t"] = t_s
        variables["u"] = t_s / self.duration_s
        return variables


def parse_constants(raw_constants: Any) -> dict[str, float]:
    if raw_constants is None:
        return {}
    if not isinstance(raw_constants, dict):
        raise ValueError("curve.constants must be a mapping.")
    constants: dict[str, float] = {}
    for key, value in raw_constants.items():
        name = str(key)
        if not name.isidentifier():
            raise ValueError(f"Invalid curve constant name '{name}'.")
        if name in {"t", "u"} or name in _MATH_NAMES:
            raise ValueError(f"curve.constants cannot redefine reserved name '{name}'.")
        constants[name] = float(value)
    return constants


def parse_point(value: Any, name: str) -> Point3:
    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError(f"{name} must be [x, y, z].")
    return tuple(float(item) for item in value)


def positive_float(value: Any, name: str) -> float:
    if value is None:
        raise ValueError(f"{name} is required.")
    number = float(value)
    if number <= 0.0:
        raise ValueError(f"{name} must be positive.")
    return number


def required(mapping: dict[str, Any], key: str, context: str) -> Any:
    if key not in mapping:
        raise ValueError(f"{context}.{key} is required.")
    return mapping[key]

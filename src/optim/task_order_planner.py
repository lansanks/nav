"""Global optimizer for the fixed-field mission task order.

The public entry point is :func:`plan_task_order`.  It reads the fixed task
field geometry from ``config/mission/map_geom.yaml`` and solves the finite
budget pick-and-place ordering problem with subset dynamic programming.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import math
from pathlib import Path
from typing import Any

import yaml

Point2 = tuple[float, float]
_EPS = 1.0e-9
_COLUMN_X_TOLERANCE = 0.05


@dataclass(frozen=True)
class MapGeometry:
    """Fixed task-field geometry used by the optimizer."""

    storage_slots: dict[int, Point2]
    return_zones: dict[int, Point2]
    return_zone_names: dict[int, str]
    start_position: Point2
    start_yaw: float
    frame_id: str
    source_path: Path


@dataclass(frozen=True)
class PlanningStep:
    """One selected box delivery in the optimized sequence."""

    slot_id: int
    category: int
    category_name: str
    target_zone_id: int
    reward: int
    box_position: Point2
    return_zone_position: Point2
    step_cost: float
    cumulative_cost: float
    empty_distance: float
    loaded_distance: float
    turn_cost: float
    fixed_cost: float


@dataclass(frozen=True)
class PlanningResult:
    """Optimizer output.

    ``order`` contains storage slot ids, not dense DP indices.
    """

    order: list[int]
    best_score: int
    best_cost: float
    cost_budget: float
    steps: list[PlanningStep]
    considered_slots: list[int]
    high_score_category: int | None

    @property
    def score(self) -> int:
        return self.best_score

    @property
    def cost(self) -> float:
        return self.best_cost


@dataclass(frozen=True)
class _LegCost:
    total: float
    empty_distance: float
    loaded_distance: float
    turn_cost: float
    fixed_cost: float


def _default_geometry_path() -> Path:
    for parent in Path(__file__).resolve().parents:
        source_tree_path = parent / "config" / "mission" / "map_geom.yaml"
        if source_tree_path.exists():
            return source_tree_path

    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return Path(__file__).resolve().parents[1] / "config" / "mission" / "map_geom.yaml"

    return Path(get_package_share_directory("navigation")) / "config" / "mission" / "map_geom.yaml"


def load_map_geometry(path: str | Path | None = None) -> MapGeometry:
    """Load and cache the fixed mission map geometry YAML."""

    geometry_path = Path(path) if path is not None else _default_geometry_path()
    return _load_map_geometry_cached(str(geometry_path.expanduser().resolve()))


@lru_cache(maxsize=4)
def _load_map_geometry_cached(path_text: str) -> MapGeometry:
    path = Path(path_text)
    if not path.exists():
        raise FileNotFoundError(f"mission map geometry file not found: {path}")

    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or "map_geometry" not in data:
        raise ValueError(f"invalid mission map geometry YAML: {path}")

    geom = data["map_geometry"]
    frame = geom.get("frame", {})
    storage_slots: dict[int, Point2] = {}
    for slot in geom["storage_slots"]["slots"]:
        slot_id = int(slot["id"])
        storage_slots[slot_id] = _point_from_yaml(slot["center"])

    return_zones: dict[int, Point2] = {}
    return_zone_names: dict[int, str] = {}
    for zone in geom["return_zones"]["zones"]:
        zone_id = int(zone["id"])
        return_zones[zone_id] = _point_from_yaml(zone["center"])
        return_zone_names[zone_id] = str(zone.get("name", zone_id))

    start = geom["start_area"]["center"]
    return MapGeometry(
        storage_slots=storage_slots,
        return_zones=return_zones,
        return_zone_names=return_zone_names,
        start_position=_point_from_yaml(start),
        start_yaw=float(start.get("yaw", 0.0)),
        frame_id=str(frame.get("frame_id", "")),
        source_path=path,
    )


def plan_task_order(
    slot_category: list[int],
    high_score_category: int | None,
    cost_budget: float,
    current_position: tuple[float, float] = (3.7, -9.45),
    current_yaw: float = 1.57079632679,
    remaining_slots: list[int] | None = None,
    alpha: float = 1.0,
    beta: float = 0.3,
    eta: float = 0.4,
    g_pick_place: float = 0.0,
) -> PlanningResult:
    """Plan the globally optimal mission delivery order.

    Args:
        slot_category: Category for each storage slot id. Category ids map to
            return zone ids in ``map_geom.yaml``: 0 food, 1 tool,
            2 instrument, 3 medicine.
        high_score_category: Category whose boxes are worth 200 points. Use
            ``None`` when the high-score area is unknown.
        cost_budget: Maximum allowed total cost.
        current_position: Current robot position used as the first empty-leg
            start point. Defaults to the start area center.
        current_yaw: Current robot yaw used for the first turn cost.
        remaining_slots: Optional subset of slot ids to optimize over. Use this
            for receding-horizon replanning after completed deliveries.
        alpha: Base unit-distance cost.
        beta: Unit turn-angle cost.
        eta: Extra unit-distance cost while carrying a box.
        g_pick_place: Fixed grab/adjust/place cost for each selected box.

    Returns:
        PlanningResult with the selected slot order, score, cost, and per-step
        breakdown.  If no positive-score delivery is feasible, ``order`` is
        empty and ``best_cost`` is 0.
    """

    geometry = load_map_geometry()
    _validate_cost_parameters(cost_budget, alpha, beta, eta, g_pick_place)
    cost_budget = float(cost_budget)
    alpha = float(alpha)
    beta = float(beta)
    eta = float(eta)
    g_pick_place = float(g_pick_place)
    high_score_category = _normalize_high_score_category(high_score_category, geometry)
    active_slots = _normalize_remaining_slots(slot_category, remaining_slots, geometry)
    if not active_slots:
        return PlanningResult(
            order=[],
            best_score=0,
            best_cost=0.0,
            cost_budget=float(cost_budget),
            steps=[],
            considered_slots=[],
            high_score_category=high_score_category,
        )

    current_position = _point_from_pair(current_position, "current_position")
    current_yaw = float(current_yaw)

    categories = [int(slot_category[slot_id]) for slot_id in active_slots]
    rewards = [200 if category == high_score_category else 100 for category in categories]

    start_costs = [
        _start_leg_cost(
            current_position,
            current_yaw,
            geometry.storage_slots[slot_id],
            geometry.return_zones[category],
            alpha,
            beta,
            eta,
            g_pick_place,
        )
        for slot_id, category in zip(active_slots, categories)
    ]

    transition_costs = _build_transition_costs(
        active_slots,
        categories,
        geometry,
        alpha,
        beta,
        eta,
        g_pick_place,
    )
    prerequisite_indices = _build_active_prerequisites(active_slots, geometry)

    best_mask, best_last, best_score, best_cost, parent = _solve_subset_dp(
        start_costs,
        transition_costs,
        rewards,
        float(cost_budget),
        prerequisite_indices,
    )

    if best_last is None:
        return PlanningResult(
            order=[],
            best_score=0,
            best_cost=0.0,
            cost_budget=float(cost_budget),
            steps=[],
            considered_slots=active_slots,
            high_score_category=high_score_category,
        )

    order_indices = _reconstruct_order(best_mask, best_last, parent)
    order = [active_slots[index] for index in order_indices]
    steps = _build_steps(
        order_indices,
        active_slots,
        categories,
        rewards,
        geometry,
        start_costs,
        transition_costs,
    )

    return PlanningResult(
        order=order,
        best_score=best_score,
        best_cost=best_cost,
        cost_budget=float(cost_budget),
        steps=steps,
        considered_slots=active_slots,
        high_score_category=high_score_category,
    )


def _solve_subset_dp(
    start_costs: list[_LegCost],
    transition_costs: list[list[_LegCost | None]],
    rewards: list[int],
    cost_budget: float,
    prerequisite_indices: list[int | None],
) -> tuple[int, int | None, int, float, list[list[int | None]]]:
    n = len(start_costs)
    total_masks = 1 << n
    dp = [[math.inf for _ in range(n)] for _ in range(total_masks)]
    parent: list[list[int | None]] = [[None for _ in range(n)] for _ in range(total_masks)]

    for index, leg_cost in enumerate(start_costs):
        dp[1 << index][index] = leg_cost.total

    for mask in range(total_masks):
        for last in range(n):
            base_cost = dp[mask][last]
            if math.isinf(base_cost):
                continue
            if base_cost > cost_budget + _EPS:
                continue
            for nxt in range(n):
                if mask & (1 << nxt):
                    continue
                prerequisite = prerequisite_indices[nxt]
                if prerequisite is not None and not (mask & (1 << prerequisite)):
                    continue
                leg_cost = transition_costs[last][nxt]
                if leg_cost is None:
                    continue
                new_mask = mask | (1 << nxt)
                new_cost = base_cost + leg_cost.total
                if new_cost + _EPS < dp[new_mask][nxt]:
                    dp[new_mask][nxt] = new_cost
                    parent[new_mask][nxt] = last

    reward_by_mask = [0 for _ in range(total_masks)]
    for mask in range(1, total_masks):
        lsb = mask & -mask
        index = lsb.bit_length() - 1
        reward_by_mask[mask] = reward_by_mask[mask ^ lsb] + rewards[index]

    best_mask = 0
    best_last: int | None = None
    best_score = 0
    best_cost = 0.0
    for mask in range(1, total_masks):
        mask_score = reward_by_mask[mask]
        for last in range(n):
            cost = dp[mask][last]
            if cost > cost_budget + _EPS:
                continue
            if mask_score > best_score:
                best_mask = mask
                best_last = last
                best_score = mask_score
                best_cost = cost
            elif mask_score == best_score and best_last is not None and cost + _EPS < best_cost:
                best_mask = mask
                best_last = last
                best_cost = cost

    return best_mask, best_last, best_score, best_cost, parent


def _build_active_prerequisites(
    active_slots: list[int],
    geometry: MapGeometry,
) -> list[int | None]:
    """Return per-active-slot prerequisite indices.

    The first selected slot is intentionally unrestricted.  During later DP
    transitions, a farther slot in a two-slot column requires its nearer slot
    to have already been selected, unless that nearer slot is not active.
    """

    near_before_far = _derive_near_before_far_slots(geometry)
    active_index_by_slot = {slot_id: index for index, slot_id in enumerate(active_slots)}
    prerequisites: list[int | None] = []
    for slot_id in active_slots:
        near_slot = near_before_far.get(slot_id)
        if near_slot is None or near_slot not in active_index_by_slot:
            prerequisites.append(None)
        else:
            prerequisites.append(active_index_by_slot[near_slot])
    return prerequisites


def _derive_near_before_far_slots(geometry: MapGeometry) -> dict[int, int]:
    return_zone_center = _return_zone_centroid(geometry)
    columns: list[list[int]] = []
    slot_ids = sorted(
        geometry.storage_slots,
        key=lambda slot_id: (
            geometry.storage_slots[slot_id][0],
            geometry.storage_slots[slot_id][1],
            slot_id,
        ),
    )

    for slot_id in slot_ids:
        slot_x = geometry.storage_slots[slot_id][0]
        for column in columns:
            column_x = geometry.storage_slots[column[0]][0]
            if abs(slot_x - column_x) <= _COLUMN_X_TOLERANCE:
                column.append(slot_id)
                break
        else:
            columns.append([slot_id])

    prerequisites: dict[int, int] = {}
    for column in columns:
        if len(column) != 2:
            continue
        first, second = column
        first_distance = _dist(geometry.storage_slots[first], return_zone_center)
        second_distance = _dist(geometry.storage_slots[second], return_zone_center)
        if abs(first_distance - second_distance) <= _EPS:
            continue
        if first_distance < second_distance:
            prerequisites[second] = first
        else:
            prerequisites[first] = second
    return prerequisites


def _return_zone_centroid(geometry: MapGeometry) -> Point2:
    if not geometry.return_zones:
        raise ValueError("mission map geometry has no return zones")
    count = len(geometry.return_zones)
    return (
        sum(point[0] for point in geometry.return_zones.values()) / count,
        sum(point[1] for point in geometry.return_zones.values()) / count,
    )


def _reconstruct_order(
    best_mask: int,
    best_last: int,
    parent: list[list[int | None]],
) -> list[int]:
    order: list[int] = []
    mask = best_mask
    last: int | None = best_last
    while last is not None:
        order.append(last)
        previous = parent[mask][last]
        mask &= ~(1 << last)
        last = previous
    order.reverse()
    return order


def _build_transition_costs(
    active_slots: list[int],
    categories: list[int],
    geometry: MapGeometry,
    alpha: float,
    beta: float,
    eta: float,
    fixed_cost: float,
) -> list[list[_LegCost | None]]:
    costs: list[list[_LegCost | None]] = []
    for prev_slot, prev_category in zip(active_slots, categories):
        row: list[_LegCost | None] = []
        for next_slot, next_category in zip(active_slots, categories):
            if prev_slot == next_slot:
                row.append(None)
                continue
            row.append(
                _transition_leg_cost(
                    geometry.storage_slots[prev_slot],
                    geometry.return_zones[prev_category],
                    geometry.storage_slots[next_slot],
                    geometry.return_zones[next_category],
                    alpha,
                    beta,
                    eta,
                    fixed_cost,
                )
            )
        costs.append(row)
    return costs


def _build_steps(
    order_indices: list[int],
    active_slots: list[int],
    categories: list[int],
    rewards: list[int],
    geometry: MapGeometry,
    start_costs: list[_LegCost],
    transition_costs: list[list[_LegCost | None]],
) -> list[PlanningStep]:
    steps: list[PlanningStep] = []
    cumulative = 0.0
    previous_index: int | None = None
    for index in order_indices:
        leg_cost = start_costs[index] if previous_index is None else transition_costs[previous_index][index]
        if leg_cost is None:
            raise RuntimeError("missing transition cost while building planning steps")
        cumulative += leg_cost.total

        slot_id = active_slots[index]
        category = categories[index]
        steps.append(
            PlanningStep(
                slot_id=slot_id,
                category=category,
                category_name=geometry.return_zone_names.get(category, str(category)),
                target_zone_id=category,
                reward=rewards[index],
                box_position=geometry.storage_slots[slot_id],
                return_zone_position=geometry.return_zones[category],
                step_cost=leg_cost.total,
                cumulative_cost=cumulative,
                empty_distance=leg_cost.empty_distance,
                loaded_distance=leg_cost.loaded_distance,
                turn_cost=leg_cost.turn_cost,
                fixed_cost=leg_cost.fixed_cost,
            )
        )
        previous_index = index
    return steps


def _start_leg_cost(
    current_position: Point2,
    current_yaw: float,
    box_position: Point2,
    return_zone_position: Point2,
    alpha: float,
    beta: float,
    eta: float,
    fixed_cost: float,
) -> _LegCost:
    empty_distance = _dist(current_position, box_position)
    loaded_distance = _dist(box_position, return_zone_position)
    empty_heading = _heading_or_none(current_position, box_position)
    loaded_heading = _heading_or_none(box_position, return_zone_position)

    first_turn = 0.0 if empty_heading is None else _angle_diff(current_yaw, empty_heading)
    incoming_heading = current_yaw if empty_heading is None else empty_heading
    pickup_turn = 0.0 if loaded_heading is None else _angle_diff(incoming_heading, loaded_heading)
    turn_cost = beta * (first_turn + pickup_turn)
    total = alpha * empty_distance + (alpha + eta) * loaded_distance + turn_cost + fixed_cost
    return _LegCost(total, empty_distance, loaded_distance, turn_cost, fixed_cost)


def _transition_leg_cost(
    prev_box_position: Point2,
    prev_return_zone_position: Point2,
    next_box_position: Point2,
    next_return_zone_position: Point2,
    alpha: float,
    beta: float,
    eta: float,
    fixed_cost: float,
) -> _LegCost:
    empty_distance = _dist(prev_return_zone_position, next_box_position)
    loaded_distance = _dist(next_box_position, next_return_zone_position)
    previous_loaded_heading = _heading_or_none(prev_box_position, prev_return_zone_position)
    empty_heading = _heading_or_none(prev_return_zone_position, next_box_position)
    next_loaded_heading = _heading_or_none(next_box_position, next_return_zone_position)

    previous_heading = previous_loaded_heading if previous_loaded_heading is not None else 0.0
    zone_turn = 0.0 if empty_heading is None else _angle_diff(previous_heading, empty_heading)
    incoming_heading = previous_heading if empty_heading is None else empty_heading
    pickup_turn = 0.0 if next_loaded_heading is None else _angle_diff(incoming_heading, next_loaded_heading)

    turn_cost = beta * (zone_turn + pickup_turn)
    total = alpha * empty_distance + (alpha + eta) * loaded_distance + turn_cost + fixed_cost
    return _LegCost(total, empty_distance, loaded_distance, turn_cost, fixed_cost)


def _normalize_remaining_slots(
    slot_category: list[int],
    remaining_slots: list[int] | None,
    geometry: MapGeometry,
) -> list[int]:
    slot_count = len(geometry.storage_slots)
    if len(slot_category) != slot_count:
        raise ValueError(f"slot_category must contain {slot_count} categories, got {len(slot_category)}")

    if remaining_slots is None:
        active_slots = sorted(geometry.storage_slots)
    else:
        active_slots = [int(slot_id) for slot_id in remaining_slots]

    seen: set[int] = set()
    normalized: list[int] = []
    for slot_id in active_slots:
        if slot_id in seen:
            raise ValueError(f"remaining_slots contains duplicate slot id: {slot_id}")
        if slot_id not in geometry.storage_slots:
            raise ValueError(f"unknown storage slot id: {slot_id}")
        category = int(slot_category[slot_id])
        if category not in geometry.return_zones:
            raise ValueError(f"slot {slot_id} has unknown category {category}")
        seen.add(slot_id)
        normalized.append(slot_id)
    return normalized


def _normalize_high_score_category(
    high_score_category: int | None,
    geometry: MapGeometry,
) -> int | None:
    if high_score_category is None:
        return None
    category = int(high_score_category)
    if category < 0:
        return None
    if category not in geometry.return_zones:
        raise ValueError(f"unknown high_score_category: {category}")
    return category


def _validate_cost_parameters(
    cost_budget: float,
    alpha: float,
    beta: float,
    eta: float,
    fixed_cost: float,
) -> None:
    values = {
        "cost_budget": cost_budget,
        "alpha": alpha,
        "beta": beta,
        "eta": eta,
        "g_pick_place": fixed_cost,
    }
    for name, value in values.items():
        value = float(value)
        if math.isnan(value):
            raise ValueError(f"{name} must not be NaN")
        if value < 0.0:
            raise ValueError(f"{name} must be non-negative")


def _dist(a: Point2, b: Point2) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _heading_or_none(a: Point2, b: Point2) -> float | None:
    if _dist(a, b) <= _EPS:
        return None
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _angle_diff(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(b - a), math.cos(b - a)))


def _point_from_yaml(value: dict[str, Any]) -> Point2:
    return float(value["x"]), float(value["y"])


def _point_from_pair(value: tuple[float, float], name: str) -> Point2:
    if len(value) != 2:
        raise ValueError(f"{name} must be a pair of (x, y)")
    return float(value[0]), float(value[1])

#!/usr/bin/env python3
"""Independently validate CLRP and Farham CLRP-TW JSON solutions.

The validator deliberately does not import solver code.  It reads the original
instance, reconstructs every route, and checks the objective and constraints
from first principles.  CLRP-TW validation is the common CLRP validation plus
hard customer time windows and the depot closing time.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence


PROBLEM_MODES = frozenset(("clrp", "clrp-tw"))
DEFAULT_ABSOLUTE_TOLERANCE = 1e-9
DEFAULT_RELATIVE_TOLERANCE = 1e-9


class ValidationInputError(ValueError):
    """Raised when an instance or solution document is malformed."""


@dataclass(frozen=True)
class Customer:
    identifier: int
    x: float
    y: float
    demand: int
    ready: float = 0.0
    due: float = math.inf
    service: float = 0.0


@dataclass(frozen=True)
class Depot:
    identifier: int
    x: float
    y: float
    capacity: int
    opening_cost: float
    opening_time: float = 0.0
    closing_time: float = math.inf


@dataclass(frozen=True)
class ValidationInstance:
    problem_mode: str
    name: str
    customers: Mapping[int, Customer]
    depots: Mapping[int, Depot]
    vehicle_capacity: int
    vehicle_fixed_cost: float
    distance_policy: str

    def distance(self, first: Customer | Depot, second: Customer | Depot) -> float:
        raw = math.hypot(first.x - second.x, first.y - second.y)
        if self.distance_policy == "euclidean-ceil-100":
            return float(math.ceil(raw * 100.0))
        if self.distance_policy == "euclidean-floor":
            return float(math.floor(raw + 1e-12))
        if self.distance_policy == "euclidean":
            return raw
        raise ValidationInputError(
            f"unsupported distance policy {self.distance_policy!r}"
        )


@dataclass(frozen=True)
class ValidationResult:
    valid: bool
    errors: tuple[str, ...]
    warnings: tuple[str, ...]
    recomputed: Mapping[str, float | int]

    def as_dict(self) -> dict[str, Any]:
        return {
            "status": "passed" if self.valid else "failed",
            "valid": self.valid,
            "errors": list(self.errors),
            "warnings": list(self.warnings),
            "recomputed": dict(self.recomputed),
        }


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValidationInputError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValidationInputError(f"{field} must be finite")
    return result


def _integer(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValidationInputError(f"{field} must be an integer")
    return value


def _load_json_object(path: Path) -> dict[str, Any]:
    def reject_nonfinite(value: str) -> None:
        raise ValidationInputError(f"non-finite JSON number {value}")

    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream, parse_constant=reject_nonfinite)
    except OSError as error:
        raise ValidationInputError(f"cannot read {path}: {error}") from error
    except json.JSONDecodeError as error:
        raise ValidationInputError(f"invalid JSON in {path}: {error}") from error
    if not isinstance(value, dict):
        raise ValidationInputError(f"{path} must contain a JSON object")
    return value


def parse_set_s(path: Path) -> ValidationInstance:
    """Parse the JSON Set S representation used by this project."""

    value = _load_json_object(path)
    raw_customers = value.get("customers")
    raw_depots = value.get("depots")
    if not isinstance(raw_customers, list) or not raw_customers:
        raise ValidationInputError("Set S customers must be a non-empty array")
    if not isinstance(raw_depots, list) or not raw_depots:
        raise ValidationInputError("Set S depots must be a non-empty array")

    customers: dict[int, Customer] = {}
    for index, raw in enumerate(raw_customers):
        if not isinstance(raw, Mapping):
            raise ValidationInputError(f"customers[{index}] must be an object")
        identifier = _integer(raw.get("index"), f"customers[{index}].index")
        customer = Customer(
            identifier,
            _finite_number(raw.get("x"), f"customers[{index}].x"),
            _finite_number(raw.get("y"), f"customers[{index}].y"),
            _integer(raw.get("demand"), f"customers[{index}].demand"),
        )
        if customer.demand < 0:
            raise ValidationInputError(
                f"customers[{index}].demand must be nonnegative"
            )
        if identifier in customers:
            raise ValidationInputError(f"duplicate customer ID {identifier}")
        customers[identifier] = customer

    depots: dict[int, Depot] = {}
    for index, raw in enumerate(raw_depots):
        if not isinstance(raw, Mapping):
            raise ValidationInputError(f"depots[{index}] must be an object")
        identifier = _integer(raw.get("index"), f"depots[{index}].index")
        depot = Depot(
            identifier,
            _finite_number(raw.get("x"), f"depots[{index}].x"),
            _finite_number(raw.get("y"), f"depots[{index}].y"),
            _integer(raw.get("capacity"), f"depots[{index}].capacity"),
            _finite_number(raw.get("costs"), f"depots[{index}].costs"),
        )
        if depot.capacity < 0:
            raise ValidationInputError(
                f"depots[{index}].capacity must be nonnegative"
            )
        if identifier in depots:
            raise ValidationInputError(f"duplicate depot ID {identifier}")
        depots[identifier] = depot

    vehicle_capacity = _integer(
        value.get("vehicle_capacity"), "vehicle_capacity"
    )
    if vehicle_capacity <= 0:
        raise ValidationInputError("vehicle_capacity must be positive")

    return ValidationInstance(
        problem_mode="clrp",
        # The solver writes the input filename stem, independently of the
        # optional descriptive name embedded in the JSON document.
        name=path.stem,
        customers=customers,
        depots=depots,
        vehicle_capacity=vehicle_capacity,
        vehicle_fixed_cost=_finite_number(
            value.get("vehicle_costs"), "vehicle_costs"
        ),
        distance_policy="euclidean-ceil-100",
    )


def parse_set_s_dat(path: Path) -> ValidationInstance:
    """Parse the project's original whitespace-delimited Set S format.

    The field order and objective-distance policy intentionally mirror
    ``code/io/SetSDatParser.cpp``.  In particular, the final
    ``realOrInteger`` flag selects ceil(Euclidean * 100) when zero and raw
    Euclidean cost otherwise.
    """

    try:
        tokens = path.read_text(encoding="utf-8").split()
    except OSError as error:
        raise ValidationInputError(f"cannot read {path}: {error}") from error
    cursor = 0

    def take(field: str) -> str:
        nonlocal cursor
        if cursor >= len(tokens):
            raise ValidationInputError(
                f"legacy Set S DAT is missing or has invalid {field}"
            )
        token = tokens[cursor]
        cursor += 1
        return token

    def take_integer(field: str) -> int:
        token = take(field)
        try:
            return int(token)
        except ValueError as error:
            raise ValidationInputError(
                f"legacy Set S DAT is missing or has invalid {field}"
            ) from error

    def take_number(field: str) -> float:
        token = take(field)
        try:
            value = float(token)
        except ValueError as error:
            raise ValidationInputError(
                f"legacy Set S DAT is missing or has invalid {field}"
            ) from error
        if not math.isfinite(value):
            raise ValidationInputError(
                f"legacy Set S DAT {field} must be finite"
            )
        return value

    customer_count = take_integer("customer count")
    depot_count = take_integer("depot count")
    if customer_count <= 0 or depot_count <= 0:
        raise ValidationInputError(
            "legacy Set S DAT customer and depot counts must be positive"
        )

    depot_coordinates = [
        (take_number("depot x-coordinate"), take_number("depot y-coordinate"))
        for _ in range(depot_count)
    ]
    customer_coordinates = [
        (
            take_number("customer x-coordinate"),
            take_number("customer y-coordinate"),
        )
        for _ in range(customer_count)
    ]
    vehicle_capacity = take_integer("vehicle capacity")
    depot_capacities = [
        take_integer("depot capacity") for _ in range(depot_count)
    ]
    customer_demands = [
        take_integer("customer demand") for _ in range(customer_count)
    ]
    depot_opening_costs = [
        take_number("depot opening cost") for _ in range(depot_count)
    ]
    vehicle_fixed_cost = take_number("vehicle fixed cost")
    real_or_integer = take_integer("legacy real-or-integer flag")

    if vehicle_capacity <= 0:
        raise ValidationInputError(
            "legacy Set S DAT vehicle capacity must be positive"
        )
    if any(capacity < 0 for capacity in depot_capacities):
        raise ValidationInputError(
            "legacy Set S DAT depot capacity must be non-negative"
        )
    if any(demand < 0 for demand in customer_demands):
        raise ValidationInputError(
            "legacy Set S DAT customer demand must be non-negative"
        )

    customers = {
        identifier: Customer(
            identifier,
            customer_coordinates[identifier][0],
            customer_coordinates[identifier][1],
            customer_demands[identifier],
        )
        for identifier in range(customer_count)
    }
    depots = {
        identifier: Depot(
            identifier,
            depot_coordinates[identifier][0],
            depot_coordinates[identifier][1],
            depot_capacities[identifier],
            depot_opening_costs[identifier],
        )
        for identifier in range(depot_count)
    }
    return ValidationInstance(
        problem_mode="clrp",
        name=path.stem,
        customers=customers,
        depots=depots,
        vehicle_capacity=vehicle_capacity,
        vehicle_fixed_cost=vehicle_fixed_cost,
        distance_policy=(
            "euclidean-ceil-100" if real_or_integer == 0 else "euclidean"
        ),
    )


def _numeric_rows(path: Path) -> dict[str, list[list[str]]]:
    sections: dict[str, list[list[str]]] = {
        "SIZE": [],
        "VEHICLE": [],
        "SATELLITE": [],
        "CUSTOMER": [],
    }
    current: str | None = None
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise ValidationInputError(f"cannot read {path}: {error}") from error
    for line_number, line in enumerate(lines, 1):
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("##"):
            continue
        if stripped.startswith("#"):
            label = stripped.lstrip("#").strip().upper()
            current = label if label in sections else current
            continue
        if current is None:
            raise ValidationInputError(
                f"{path}:{line_number}: data before a recognized section"
            )
        sections[current].append(stripped.split())
    return sections


def parse_farham(path: Path) -> ValidationInstance:
    """Parse the single-echelon Farham LRPTW text format."""

    sections = _numeric_rows(path)
    if len(sections["SIZE"]) != 1 or len(sections["SIZE"][0]) != 3:
        raise ValidationInputError("SIZE must contain NUM_CDC, depots, customers")
    try:
        num_cdc, num_depots, num_customers = map(int, sections["SIZE"][0])
    except ValueError as error:
        raise ValidationInputError("SIZE values must be integers") from error
    if num_cdc != 0:
        raise ValidationInputError(
            "only single-echelon Farham instances with NUM_CDC=0 are supported"
        )
    if num_depots <= 0 or num_customers <= 0:
        raise ValidationInputError("depot and customer counts must be positive")

    if len(sections["VEHICLE"]) != 1 or len(sections["VEHICLE"][0]) != 3:
        raise ValidationInputError(
            "VEHICLE must contain type ID, fixed cost, and capacity"
        )
    try:
        vehicle_type, vehicle_fixed_cost, vehicle_capacity = map(
            float, sections["VEHICLE"][0]
        )
    except ValueError as error:
        raise ValidationInputError("VEHICLE values must be numeric") from error
    if not all(
        math.isfinite(value)
        for value in (vehicle_type, vehicle_fixed_cost, vehicle_capacity)
    ):
        raise ValidationInputError("VEHICLE values must be finite")
    if vehicle_fixed_cost < 0:
        raise ValidationInputError("vehicle fixed cost must be nonnegative")
    if not vehicle_capacity.is_integer() or vehicle_capacity <= 0:
        raise ValidationInputError("vehicle capacity must be a positive integer")

    if len(sections["SATELLITE"]) < num_depots:
        raise ValidationInputError(
            f"expected {num_depots} SATELLITE rows, found "
            f"{len(sections['SATELLITE'])}"
        )
    depots: dict[int, Depot] = {}
    for index, row in enumerate(sections["SATELLITE"][:num_depots]):
        if len(row) != 6:
            raise ValidationInputError(
                f"SATELLITE row {index + 1} must contain 6 fields"
            )
        try:
            identifier = int(row[0])
            x, y, close_time, opening_cost = map(float, row[1:5])
            capacity = int(row[5])
        except ValueError as error:
            raise ValidationInputError(
                f"SATELLITE row {index + 1} has non-numeric fields"
            ) from error
        if not all(
            math.isfinite(value)
            for value in (x, y, close_time, opening_cost)
        ):
            raise ValidationInputError(
                f"SATELLITE row {index + 1} values must be finite"
            )
        if close_time < 0 or opening_cost < 0 or capacity <= 0:
            raise ValidationInputError(
                f"SATELLITE row {index + 1} has invalid time/cost/capacity"
            )
        if identifier in depots:
            raise ValidationInputError(f"duplicate depot ID {identifier}")
        depots[identifier] = Depot(
            identifier,
            x,
            y,
            capacity,
            opening_cost,
            0.0,
            close_time,
        )

    if len(sections["CUSTOMER"]) < num_customers:
        raise ValidationInputError(
            f"expected {num_customers} CUSTOMER rows, found "
            f"{len(sections['CUSTOMER'])}"
        )
    customers: dict[int, Customer] = {}
    # Farham files physically retain up to 100 Solomon customers.  The SIZE
    # declaration defines the active prefix for this derived instance.
    for index, row in enumerate(sections["CUSTOMER"][:num_customers]):
        if len(row) != 7:
            raise ValidationInputError(
                f"CUSTOMER row {index + 1} must contain 7 fields"
            )
        try:
            identifier = int(row[0])
            x, y = map(float, row[1:3])
            demand = int(row[3])
            ready, due, service = map(float, row[4:7])
        except ValueError as error:
            raise ValidationInputError(
                f"CUSTOMER row {index + 1} has non-numeric fields"
            ) from error
        if not all(math.isfinite(value) for value in (x, y, ready, due, service)):
            raise ValidationInputError(
                f"CUSTOMER row {index + 1} values must be finite"
            )
        if identifier in customers:
            raise ValidationInputError(f"duplicate customer ID {identifier}")
        if (
            demand < 0
            or demand > vehicle_capacity
            or ready < 0
            or ready > due
            or service < 0
        ):
            raise ValidationInputError(
                f"customer {identifier} has inconsistent demand/time data"
            )
        customers[identifier] = Customer(
            identifier, x, y, demand, ready, due, service
        )

    return ValidationInstance(
        problem_mode="clrp-tw",
        name=path.stem,
        customers=customers,
        depots=depots,
        vehicle_capacity=int(vehicle_capacity),
        vehicle_fixed_cost=vehicle_fixed_cost,
        distance_policy="euclidean-floor",
    )


def load_instance(path: Path, problem_mode: str) -> ValidationInstance:
    if problem_mode not in PROBLEM_MODES:
        raise ValidationInputError(f"unsupported problem mode {problem_mode!r}")
    if path.suffix.casefold() == ".json":
        instance = parse_set_s(path)
    elif path.suffix.casefold() == ".dat":
        instance = parse_set_s_dat(path)
    elif path.suffix.casefold() == ".txt":
        instance = parse_farham(path)
    else:
        raise ValidationInputError(
            f"unsupported validation instance format: {path.suffix}"
        )
    if instance.problem_mode != problem_mode:
        raise ValidationInputError(
            f"instance is {instance.problem_mode}, requested mode is {problem_mode}"
        )
    return instance


def _close(
    reported: Any,
    expected: float,
    field: str,
    errors: list[str],
    absolute_tolerance: float,
    relative_tolerance: float,
) -> None:
    try:
        value = _finite_number(reported, field)
    except ValidationInputError as error:
        errors.append(str(error))
        return
    if not math.isclose(
        value,
        expected,
        rel_tol=relative_tolerance,
        abs_tol=absolute_tolerance,
    ):
        errors.append(f"{field} is {value}, recomputed value is {expected}")


def validate_document(
    instance: ValidationInstance,
    solution: Mapping[str, Any],
    *,
    absolute_tolerance: float = DEFAULT_ABSOLUTE_TOLERANCE,
    relative_tolerance: float = DEFAULT_RELATIVE_TOLERANCE,
) -> ValidationResult:
    """Validate one already-parsed solution document."""

    if (
        not math.isfinite(absolute_tolerance)
        or not math.isfinite(relative_tolerance)
        or absolute_tolerance < 0
        or relative_tolerance < 0
    ):
        raise ValidationInputError(
            "validation tolerances must be finite and nonnegative"
        )

    errors: list[str] = []
    warnings: list[str] = []
    try:
        schema_version = _integer(
            solution.get("schema_version"), "solution.schema_version"
        )
        if schema_version != 1:
            errors.append(
                f"solution.schema_version is {schema_version}, expected 1"
            )
    except ValidationInputError as error:
        errors.append(str(error))
    instance_id = solution.get("instance_id")
    if not isinstance(instance_id, str):
        errors.append("solution.instance_id must be a string")
    elif instance_id != instance.name:
        errors.append(
            f"solution.instance_id is {instance_id!r}, expected "
            f"{instance.name!r}"
        )
    mode = solution.get("problem_mode")
    if mode != instance.problem_mode:
        errors.append(
            f"solution problem_mode is {mode!r}, expected {instance.problem_mode!r}"
        )
    if solution.get("feasible") is not True:
        errors.append("solution must explicitly declare feasible=true")
    reported_policy = solution.get("distance_policy")
    if reported_policy != instance.distance_policy:
        errors.append(
            f"solution distance_policy is {reported_policy!r}, expected "
            f"{instance.distance_policy!r}"
        )

    raw_depots = solution.get("depots")
    if not isinstance(raw_depots, list):
        raise ValidationInputError("solution.depots must be an array")

    visits: list[int] = []
    opening_cost = 0.0
    vehicle_cost = 0.0
    travel_cost = 0.0
    route_count = 0
    seen_depots: set[int] = set()

    for depot_index, raw_depot in enumerate(raw_depots):
        field = f"depots[{depot_index}]"
        if not isinstance(raw_depot, Mapping):
            errors.append(f"{field} must be an object")
            continue
        try:
            depot_id = _integer(raw_depot.get("depot_id"), f"{field}.depot_id")
        except ValidationInputError as error:
            errors.append(str(error))
            continue
        depot = instance.depots.get(depot_id)
        if depot is None:
            errors.append(f"{field}.depot_id {depot_id} is unknown")
            continue
        if depot_id in seen_depots:
            errors.append(f"depot {depot_id} appears more than once")
            continue
        seen_depots.add(depot_id)
        opening_cost += depot.opening_cost

        raw_routes = raw_depot.get("routes")
        if not isinstance(raw_routes, list):
            errors.append(f"{field}.routes must be an array")
            continue
        depot_load = 0
        for route_index, raw_route in enumerate(raw_routes):
            route_field = f"{field}.routes[{route_index}]"
            if not isinstance(raw_route, Mapping):
                errors.append(f"{route_field} must be an object")
                continue
            raw_customers = raw_route.get("customers")
            if not isinstance(raw_customers, list) or not raw_customers:
                errors.append(f"{route_field}.customers must be non-empty")
                continue

            route_customers: list[Customer] = []
            malformed = False
            for position, raw_id in enumerate(raw_customers):
                try:
                    customer_id = _integer(
                        raw_id, f"{route_field}.customers[{position}]"
                    )
                except ValidationInputError as error:
                    errors.append(str(error))
                    malformed = True
                    continue
                customer = instance.customers.get(customer_id)
                if customer is None:
                    errors.append(
                        f"{route_field}.customers[{position}] ID "
                        f"{customer_id} is unknown"
                    )
                    malformed = True
                    continue
                visits.append(customer_id)
                route_customers.append(customer)
            if malformed or not route_customers:
                continue

            route_count += 1
            vehicle_cost += instance.vehicle_fixed_cost
            load = sum(customer.demand for customer in route_customers)
            depot_load += load
            if load > instance.vehicle_capacity:
                errors.append(
                    f"{route_field} load {load} exceeds vehicle capacity "
                    f"{instance.vehicle_capacity}"
                )

            time = depot.opening_time
            previous: Customer | Depot = depot
            route_travel = 0.0
            recomputed_schedule: list[tuple[Customer, float, float, float, float]] = []
            for customer in route_customers:
                leg = instance.distance(previous, customer)
                route_travel += leg
                time += leg
                if instance.problem_mode == "clrp-tw":
                    arrival_time = time
                    service_start = max(time, customer.ready)
                    waiting_time = service_start - arrival_time
                    if service_start > customer.due + absolute_tolerance:
                        errors.append(
                            f"{route_field}: customer {customer.identifier} "
                            f"service starts at {service_start}, after due "
                            f"{customer.due}"
                        )
                    departure_time = service_start + customer.service
                    recomputed_schedule.append(
                        (
                            customer,
                            arrival_time,
                            waiting_time,
                            service_start,
                            departure_time,
                        )
                    )
                    time = departure_time
                previous = customer
            final_leg = instance.distance(previous, depot)
            route_travel += final_leg
            time += final_leg
            travel_cost += route_travel
            if (
                instance.problem_mode == "clrp-tw"
                and time > depot.closing_time + absolute_tolerance
            ):
                errors.append(
                    f"{route_field} returns at {time}, after depot "
                    f"{depot.identifier} closes at {depot.closing_time}"
                )

            if "load" in raw_route:
                _close(
                    raw_route["load"],
                    float(load),
                    f"{route_field}.load",
                    errors,
                    absolute_tolerance,
                    relative_tolerance,
                )
            if "travel_cost" in raw_route:
                _close(
                    raw_route["travel_cost"],
                    route_travel,
                    f"{route_field}.travel_cost",
                    errors,
                    absolute_tolerance,
                    relative_tolerance,
                )
            if instance.problem_mode == "clrp-tw" and "return_time" in raw_route:
                _close(
                    raw_route["return_time"],
                    time,
                    f"{route_field}.return_time",
                    errors,
                    absolute_tolerance,
                    relative_tolerance,
                )
            if instance.problem_mode == "clrp-tw" and "schedule" in raw_route:
                raw_schedule = raw_route["schedule"]
                if not isinstance(raw_schedule, list):
                    errors.append(f"{route_field}.schedule must be an array")
                elif len(raw_schedule) != len(recomputed_schedule):
                    errors.append(
                        f"{route_field}.schedule has {len(raw_schedule)} "
                        f"entries, expected {len(recomputed_schedule)}"
                    )
                else:
                    for position, (raw_visit, expected_visit) in enumerate(
                        zip(raw_schedule, recomputed_schedule)
                    ):
                        visit_field = (
                            f"{route_field}.schedule[{position}]"
                        )
                        if not isinstance(raw_visit, Mapping):
                            errors.append(f"{visit_field} must be an object")
                            continue
                        (
                            expected_customer,
                            expected_arrival,
                            expected_waiting,
                            expected_start,
                            expected_departure,
                        ) = expected_visit
                        try:
                            reported_customer = _integer(
                                raw_visit.get("customer_id"),
                                f"{visit_field}.customer_id",
                            )
                            if reported_customer != expected_customer.identifier:
                                errors.append(
                                    f"{visit_field}.customer_id is "
                                    f"{reported_customer}, expected "
                                    f"{expected_customer.identifier}"
                                )
                        except ValidationInputError as error:
                            errors.append(str(error))
                        for schedule_name, expected_value in (
                            ("arrival_time", expected_arrival),
                            ("waiting_time", expected_waiting),
                            ("service_start_time", expected_start),
                            ("departure_time", expected_departure),
                        ):
                            if schedule_name not in raw_visit:
                                errors.append(
                                    f"{visit_field}.{schedule_name} is missing"
                                )
                            else:
                                _close(
                                    raw_visit[schedule_name],
                                    expected_value,
                                    f"{visit_field}.{schedule_name}",
                                    errors,
                                    absolute_tolerance,
                                    relative_tolerance,
                                )

        if depot_load > depot.capacity:
            errors.append(
                f"depot {depot_id} load {depot_load} exceeds capacity "
                f"{depot.capacity}"
            )
        if "load" in raw_depot:
            _close(
                raw_depot["load"],
                float(depot_load),
                f"{field}.load",
                errors,
                absolute_tolerance,
                relative_tolerance,
            )

    expected = set(instance.customers)
    observed = set(visits)
    missing = sorted(expected - observed)
    duplicates = sorted(
        customer_id for customer_id in observed if visits.count(customer_id) > 1
    )
    if missing:
        errors.append(f"customers missing from all routes: {missing}")
    if duplicates:
        errors.append(f"customers visited more than once: {duplicates}")

    total = opening_cost + vehicle_cost + travel_cost
    raw_objective = solution.get("objective")
    if not isinstance(raw_objective, Mapping):
        errors.append("solution.objective must be an object")
    else:
        for name, expected_value in (
            ("total", total),
            ("depot_opening", opening_cost),
            ("vehicle_fixed", vehicle_cost),
            ("travel", travel_cost),
        ):
            if name not in raw_objective:
                errors.append(f"solution.objective.{name} is missing")
            else:
                _close(
                    raw_objective[name],
                    expected_value,
                    f"solution.objective.{name}",
                    errors,
                    absolute_tolerance,
                    relative_tolerance,
                )

    recomputed: dict[str, float | int] = {
        "objective": total,
        "depot_opening_cost": opening_cost,
        "vehicle_fixed_cost": vehicle_cost,
        "travel_cost": travel_cost,
        "opened_depots": len(seen_depots),
        "routes": route_count,
        "visited_customers": len(visits),
    }
    return ValidationResult(not errors, tuple(errors), tuple(warnings), recomputed)


def validate_solution(
    instance_path: Path,
    solution_path: Path,
    problem_mode: str,
    *,
    absolute_tolerance: float = DEFAULT_ABSOLUTE_TOLERANCE,
    relative_tolerance: float = DEFAULT_RELATIVE_TOLERANCE,
) -> ValidationResult:
    instance = load_instance(instance_path, problem_mode)
    solution = _load_json_object(solution_path)
    return validate_document(
        instance,
        solution,
        absolute_tolerance=absolute_tolerance,
        relative_tolerance=relative_tolerance,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Independently validate a CLRP/CLRP-TW solution.json"
    )
    parser.add_argument("--instance", required=True, type=Path)
    parser.add_argument("--solution", required=True, type=Path)
    parser.add_argument(
        "--problem-mode", required=True, choices=sorted(PROBLEM_MODES)
    )
    parser.add_argument(
        "--absolute-tolerance",
        type=float,
        default=DEFAULT_ABSOLUTE_TOLERANCE,
    )
    parser.add_argument(
        "--relative-tolerance",
        type=float,
        default=DEFAULT_RELATIVE_TOLERANCE,
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = build_parser().parse_args(argv)
    try:
        result = validate_solution(
            arguments.instance,
            arguments.solution,
            arguments.problem_mode,
            absolute_tolerance=arguments.absolute_tolerance,
            relative_tolerance=arguments.relative_tolerance,
        )
    except ValidationInputError as error:
        print(json.dumps({"status": "error", "error": str(error)}, indent=2))
        return 2
    print(json.dumps(result.as_dict(), indent=2, sort_keys=True))
    return 0 if result.valid else 1


if __name__ == "__main__":
    sys.exit(main())

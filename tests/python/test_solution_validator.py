from __future__ import annotations

import copy
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
VALIDATION_DIR = REPO_ROOT / "scripts" / "validation"
sys.path.insert(0, str(VALIDATION_DIR))

import solution_validator as validator  # noqa: E402


INSTANCE_PATH = REPO_ROOT / "instances" / "set_F" / "R101_s1_3_10.txt"


def singleton_solution(
    instance: validator.ValidationInstance,
) -> dict[str, object]:
    routes_by_depot: dict[int, list[dict[str, object]]] = {}
    for customer in instance.customers.values():
        selected = None
        for depot in instance.depots.values():
            distance = instance.distance(depot, customer)
            service_start = max(distance, customer.ready)
            return_time = service_start + customer.service + distance
            if (
                service_start <= customer.due + 1e-9
                and return_time <= depot.closing_time + 1e-9
            ):
                selected = (depot, distance, return_time)
                break
        if selected is None:
            raise AssertionError(f"no singleton route for {customer.identifier}")
        depot, distance, return_time = selected
        arrival_time = distance
        service_start_time = max(arrival_time, customer.ready)
        waiting_time = service_start_time - arrival_time
        departure_time = service_start_time + customer.service
        routes_by_depot.setdefault(depot.identifier, []).append(
            {
                "load": customer.demand,
                "travel_cost": 2.0 * distance,
                "return_time": return_time,
                "customers": [customer.identifier],
                "schedule": [
                    {
                        "customer_id": customer.identifier,
                        "arrival_time": arrival_time,
                        "waiting_time": waiting_time,
                        "service_start_time": service_start_time,
                        "departure_time": departure_time,
                    }
                ],
            }
        )

    depots = []
    opening = 0.0
    travel = 0.0
    routes = 0
    for depot_id, depot_routes in routes_by_depot.items():
        depot = instance.depots[depot_id]
        load = sum(int(route["load"]) for route in depot_routes)
        depots.append(
            {
                "depot_id": depot_id,
                "load": load,
                "routes": depot_routes,
            }
        )
        opening += depot.opening_cost
        travel += sum(float(route["travel_cost"]) for route in depot_routes)
        routes += len(depot_routes)
    vehicle = routes * instance.vehicle_fixed_cost
    return {
        "schema_version": 1,
        "problem_mode": "clrp-tw",
        "instance_id": instance.name,
        "feasible": True,
        "distance_policy": "euclidean-floor",
        "objective": {
            "total": opening + vehicle + travel,
            "depot_opening": opening,
            "vehicle_fixed": vehicle,
            "travel": travel,
        },
        "depots": depots,
    }


class FarhamParserTests(unittest.TestCase):
    def test_declared_customer_prefix_is_active(self) -> None:
        instance = validator.parse_farham(INSTANCE_PATH)
        self.assertEqual(len(instance.depots), 3)
        self.assertEqual(len(instance.customers), 10)
        self.assertEqual(set(instance.customers), set(range(1, 11)))
        self.assertEqual(instance.vehicle_capacity, 200)
        self.assertEqual(instance.vehicle_fixed_cost, 225.0)

    def test_legacy_dat_rounding_flag_matches_cpp_semantics(self) -> None:
        template = (
            "1 1\n"
            "0 0\n"
            "1 1\n"
            "10\n"
            "10\n"
            "1\n"
            "0\n"
            "0\n"
            "{flag}\n"
        )
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scaled_path = root / "scaled.dat"
            scaled_path.write_text(template.format(flag=0), encoding="utf-8")
            scaled = validator.load_instance(scaled_path, "clrp")
            self.assertEqual(scaled.distance_policy, "euclidean-ceil-100")
            self.assertEqual(
                scaled.distance(scaled.depots[0], scaled.customers[0]),
                142.0,
            )
            solution_path = root / "solution.json"
            solution_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "problem_mode": "clrp",
                        "instance_id": "scaled",
                        "feasible": True,
                        "distance_policy": "euclidean-ceil-100",
                        "objective": {
                            "total": 284.0,
                            "depot_opening": 0.0,
                            "vehicle_fixed": 0.0,
                            "travel": 284.0,
                        },
                        "depots": [
                            {
                                "depot_id": 0,
                                "load": 1,
                                "routes": [
                                    {
                                        "load": 1,
                                        "travel_cost": 284.0,
                                        "customers": [0],
                                    }
                                ],
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            result = validator.validate_solution(
                scaled_path, solution_path, "clrp"
            )
            self.assertTrue(result.valid, result.errors)

            exact_path = root / "exact.dat"
            exact_path.write_text(template.format(flag=1), encoding="utf-8")
            exact = validator.load_instance(exact_path, "clrp")
            self.assertEqual(exact.distance_policy, "euclidean")
            self.assertAlmostEqual(
                exact.distance(exact.depots[0], exact.customers[0]),
                math.sqrt(2.0),
            )


class ValidationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.instance = validator.parse_farham(INSTANCE_PATH)
        self.solution = singleton_solution(self.instance)

    def test_valid_singleton_solution_passes_all_constraints(self) -> None:
        result = validator.validate_document(self.instance, self.solution)
        self.assertTrue(result.valid, result.errors)
        self.assertEqual(result.recomputed["visited_customers"], 10)
        self.assertEqual(result.recomputed["routes"], 10)

    def test_duplicate_and_missing_customer_are_rejected(self) -> None:
        changed = copy.deepcopy(self.solution)
        depots = changed["depots"]
        first_route = depots[0]["routes"][0]
        second_route = depots[0]["routes"][1]
        second_route["customers"] = list(first_route["customers"])
        result = validator.validate_document(self.instance, changed)
        self.assertFalse(result.valid)
        self.assertTrue(
            any("missing" in error for error in result.errors), result.errors
        )
        self.assertTrue(
            any("more than once" in error for error in result.errors),
            result.errors,
        )

    def test_hard_time_window_is_an_extension_of_common_checks(self) -> None:
        changed = copy.deepcopy(self.solution)
        # Find an ordered pair that is capacity-feasible but whose second
        # service starts after its due time.  The two singleton routes are then
        # replaced by this one route, preserving common CLRP coverage.
        bad_pair = None
        for depot in self.instance.depots.values():
            for first in self.instance.customers.values():
                first_start = max(
                    self.instance.distance(depot, first), first.ready
                )
                departure = first_start + first.service
                for second in self.instance.customers.values():
                    if first.identifier == second.identifier:
                        continue
                    if (
                        first.demand + second.demand
                        > self.instance.vehicle_capacity
                    ):
                        continue
                    second_arrival = departure + self.instance.distance(
                        first, second
                    )
                    if max(second_arrival, second.ready) > second.due:
                        bad_pair = (
                            depot.identifier,
                            first.identifier,
                            second.identifier,
                        )
                        break
                if bad_pair:
                    break
            if bad_pair:
                break
        self.assertIsNotNone(bad_pair)
        target_depot_id, first_id, second_id = bad_pair

        removed = []
        for customer_id in (first_id, second_id):
            for depot_document in changed["depots"]:
                for route in list(depot_document["routes"]):
                    if route["customers"] == [customer_id]:
                        removed.append(route)
                        depot_document["routes"].remove(route)
                        depot_document["load"] -= route["load"]
                        break
                if len(removed) == (1 if customer_id == first_id else 2):
                    break
        target = next(
            depot
            for depot in changed["depots"]
            if depot["depot_id"] == target_depot_id
        )
        combined_load = sum(route["load"] for route in removed)
        target["routes"].append(
            {
                "load": combined_load,
                "customers": [first_id, second_id],
            }
        )
        target["load"] += combined_load

        result = validator.validate_document(self.instance, changed)
        self.assertFalse(result.valid)
        self.assertTrue(
            any("service starts" in error for error in result.errors),
            result.errors,
        )

    def test_objective_mismatch_is_rejected(self) -> None:
        changed = copy.deepcopy(self.solution)
        changed["objective"]["total"] = math.nextafter(
            changed["objective"]["total"] + 1.0, math.inf
        )
        result = validator.validate_document(self.instance, changed)
        self.assertFalse(result.valid)
        self.assertTrue(
            any("objective.total" in error for error in result.errors),
            result.errors,
        )

    def test_solution_schema_and_instance_identity_are_enforced(self) -> None:
        changed = copy.deepcopy(self.solution)
        changed["schema_version"] = 2
        changed["instance_id"] = "another-instance"
        changed.pop("distance_policy")
        result = validator.validate_document(self.instance, changed)
        self.assertFalse(result.valid)
        self.assertTrue(
            any("schema_version" in error for error in result.errors),
            result.errors,
        )
        self.assertTrue(
            any("instance_id" in error for error in result.errors),
            result.errors,
        )
        self.assertTrue(
            any("distance_policy" in error for error in result.errors),
            result.errors,
        )

    def test_reported_schedule_is_recomputed_not_trusted(self) -> None:
        changed = copy.deepcopy(self.solution)
        visit = changed["depots"][0]["routes"][0]["schedule"][0]
        visit["arrival_time"] += 1.0
        result = validator.validate_document(self.instance, changed)
        self.assertFalse(result.valid)
        self.assertTrue(
            any("schedule[0].arrival_time" in error for error in result.errors),
            result.errors,
        )

    def test_default_temporal_tolerance_matches_cpp(self) -> None:
        self.assertEqual(validator.DEFAULT_ABSOLUTE_TOLERANCE, 1e-9)
        with self.assertRaisesRegex(
            validator.ValidationInputError, "tolerances"
        ):
            validator.validate_document(
                self.instance,
                self.solution,
                absolute_tolerance=-1.0,
            )


if __name__ == "__main__":
    unittest.main()

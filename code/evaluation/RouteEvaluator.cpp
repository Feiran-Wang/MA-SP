/******************************************************************************
 * File: RouteEvaluator.cpp
 * Description:
 *   Implementation of shared CLRP and hard-time-window route evaluation.
 ******************************************************************************/

#include "RouteEvaluator.h"

#include "read_data.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace {

double checkedArcValue(
	double** matrix,
	const int from,
	const int to,
	const char* matrixName) {
	if (matrix == nullptr || matrix[from] == nullptr) {
		throw std::logic_error(
			std::string("route evaluation requires initialized ") +
			matrixName + " matrix");
	}
	const double value = matrix[from][to];
	if (!std::isfinite(value) || value < 0.0) {
		throw std::domain_error(
			std::string("route evaluation found an invalid ") +
			matrixName + " arc");
	}
	return value;
}

void validateCommonInput(
	const read_data& data,
	const int depotIndex,
	const std::vector<int>& sequence) {
	if (data.numV < 0 || data.numL <= 0 ||
		data.numC < data.numV + data.numL) {
		throw std::logic_error(
			"route evaluation received inconsistent instance dimensions");
	}
	if (depotIndex < 0 || depotIndex >= data.numL) {
		throw std::out_of_range(
			"route evaluation depot index is out of range");
	}
	if (data.demCus == nullptr) {
		throw std::logic_error(
			"route evaluation requires initialized customer demands");
	}
	if (data.vehCap < 0) {
		throw std::logic_error(
			"route evaluation requires a nonnegative vehicle capacity");
	}
	for (const int customer : sequence) {
		if (customer < 0 || customer >= data.numV) {
			throw std::out_of_range(
				"route evaluation customer index is out of range");
		}
		if (data.demCus[customer] < 0) {
			throw std::domain_error(
				"route evaluation found a negative customer demand");
		}
	}
}

void requireTemporalData(const read_data& data) {
	const std::size_t customerCount =
		static_cast<std::size_t>(data.numV);
	const std::size_t depotCount =
		static_cast<std::size_t>(data.numL);

	if (data.readyTime.size() != customerCount ||
		data.dueTime.size() != customerCount ||
		data.serviceTime.size() != customerCount) {
		throw std::logic_error(
			"hard-time-window evaluation requires one time-window "
			"record per customer");
	}
	if (data.depotOpenTime.size() != depotCount ||
		data.depotCloseTime.size() != depotCount) {
		throw std::logic_error(
			"hard-time-window evaluation requires one time horizon "
			"per depot");
	}
	if (data.T == nullptr) {
		throw std::logic_error(
			"hard-time-window evaluation requires an initialized "
			"travel-time matrix");
	}

	for (int customer = 0; customer < data.numV; ++customer) {
		const double ready = data.readyTime[customer];
		const double due = data.dueTime[customer];
		const double service = data.serviceTime[customer];
		if (!std::isfinite(ready) || !std::isfinite(due) ||
			!std::isfinite(service) || ready > due || service < 0.0) {
			throw std::domain_error(
				"hard-time-window evaluation found invalid customer "
				"time data");
		}
	}
	for (int depot = 0; depot < data.numL; ++depot) {
		const double open = data.depotOpenTime[depot];
		const double close = data.depotCloseTime[depot];
		if (!std::isfinite(open) || !std::isfinite(close) ||
			open > close) {
			throw std::domain_error(
				"hard-time-window evaluation found an invalid depot "
				"time horizon");
		}
	}
}

}  // namespace

bool NoTemporalConstraint::isNoOp() const noexcept {
	return true;
}

void NoTemporalConstraint::evaluate(
	const read_data&,
	const int,
	const std::vector<int>&,
	const EvaluationDetail,
	RouteEvaluation&) const {
	// Intentionally empty. RouteEvaluator avoids this virtual call through
	// isNoOp(), but keeping the implementation total makes the policy safe to
	// use directly.
}

HardTimeWindowConstraint::HardTimeWindowConstraint(
	const double epsilon)
	: epsilon_(epsilon) {
	if (!std::isfinite(epsilon_) || epsilon_ < 0.0) {
		throw std::invalid_argument(
			"hard-time-window epsilon must be finite and nonnegative");
	}
}

bool HardTimeWindowConstraint::isNoOp() const noexcept {
	return false;
}

double HardTimeWindowConstraint::epsilon() const noexcept {
	return epsilon_;
}

void HardTimeWindowConstraint::evaluate(
	const read_data& data,
	const int depotIndex,
	const std::vector<int>& sequence,
	const EvaluationDetail detail,
	RouteEvaluation& result) const {
	requireTemporalData(data);

	const int depotNode = data.numV + depotIndex;
	double currentTime = data.depotOpenTime[depotIndex];
	result.routeStartTime = currentTime;
	result.returnTime = currentTime;

	if (detail == EvaluationDetail::FullSchedule) {
		result.schedule.clear();
		result.schedule.reserve(sequence.size());
	}

	int previousNode = depotNode;
	for (const int customer : sequence) {
		const double travelTime =
			checkedArcValue(data.T, previousNode, customer, "travel-time");
		const double arrivalTime = currentTime + travelTime;
		const double serviceStartTime =
			std::max(arrivalTime, data.readyTime[customer]);
		const double waitingTime = serviceStartTime - arrivalTime;
		const double departureTime =
			serviceStartTime + data.serviceTime[customer];

		result.totalWaitingTime += waitingTime;
		if (detail == EvaluationDetail::FullSchedule) {
			result.schedule.push_back(CustomerSchedule{
				customer,
				arrivalTime,
				waitingTime,
				serviceStartTime,
				departureTime});
		}

		if (result.temporalFeasible &&
			serviceStartTime >
				data.dueTime[customer] + epsilon_) {
			result.temporalFeasible = false;
			result.violatingCustomer = customer;
			result.temporalViolation =
				TemporalViolationKind::CustomerDueTime;
		}

		currentTime = departureTime;
		previousNode = customer;
	}

	result.returnTime =
		currentTime +
		checkedArcValue(data.T, previousNode, depotNode, "travel-time");
	// Preserve the first customer-due-time violation when a route also returns
	// late: it is the earliest temporal failure along the route.
	if (result.temporalFeasible &&
		result.returnTime >
		data.depotCloseTime[depotIndex] + epsilon_) {
		result.temporalFeasible = false;
		result.violatingCustomer = -1;
		result.temporalViolation =
			TemporalViolationKind::DepotCloseTime;
	}
}

RouteEvaluator::RouteEvaluator(
	const read_data& data,
	const TemporalConstraint& temporalConstraint)
	: data_(data),
	  temporalConstraint_(temporalConstraint) {
}

RouteEvaluation RouteEvaluator::evaluate(
	const int depotIndex,
	const std::vector<int>& sequence,
	const EvaluationDetail detail) const {
	validateCommonInput(data_, depotIndex, sequence);

	RouteEvaluation result;
	result.customerCount = sequence.size();

	const int depotNode = data_.numV + depotIndex;
	int previousNode = depotNode;
	for (const int customer : sequence) {
		result.load += data_.demCus[customer];
		result.travelCost +=
			checkedArcValue(data_.D, previousNode, customer, "travel-cost");
		previousNode = customer;
	}
	result.travelCost +=
		checkedArcValue(data_.D, previousNode, depotNode, "travel-cost");
	result.capacityFeasible = result.load <= data_.vehCap;

	if (!temporalConstraint_.isNoOp()) {
		temporalConstraint_.evaluate(
			data_, depotIndex, sequence, detail, result);
	}
	return result;
}

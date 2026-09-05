/******************************************************************************
 * File: RouteEvaluator.cpp
 * Description:
 *   Implementation of shared CLRP and hard-time-window route evaluation.
 ******************************************************************************/

#include "RouteEvaluator.h"

#include "ComponentProfiler.h"
#include "SolverContext.h"

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

double routeArcValue(
		double** matrix,
		const int from,
		const int to,
		const char* matrixName,
		const bool trustedProblemData) {
	return trustedProblemData
		? matrix[from][to]
		: checkedArcValue(matrix, from, to, matrixName);
}

void validateRouteIndices(
		const SolverContext& data,
		const int depotIndex,
		const std::vector<int>& sequence) {
	if (depotIndex < 0 || depotIndex >= data.numL) {
		throw std::out_of_range(
			"route evaluation depot index is out of range");
	}
	for (const int customer : sequence) {
		if (customer < 0 || customer >= data.numV) {
			throw std::out_of_range(
				"route evaluation customer index is out of range");
		}
	}
}

void validateCommonInput(
	const SolverContext& data,
	const int depotIndex,
	const std::vector<int>& sequence) {
	if (data.numV < 0 || data.numL <= 0 ||
		data.numC < data.numV + data.numL) {
		throw std::logic_error(
			"route evaluation received inconsistent instance dimensions");
	}
	validateRouteIndices(data, depotIndex, sequence);
	if (data.demCus == nullptr) {
		throw std::logic_error(
			"route evaluation requires initialized customer demands");
	}
	if (data.vehCap < 0) {
		throw std::logic_error(
			"route evaluation requires a nonnegative vehicle capacity");
	}
	for (const int customer : sequence) {
		if (data.demCus[customer] < 0) {
			throw std::domain_error(
				"route evaluation found a negative customer demand");
		}
	}
}

void requireTemporalData(const SolverContext& data) {
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
	const SolverContext&,
	const int,
	const std::vector<int>&,
	const EvaluationDetail,
	const bool,
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
	const SolverContext& data,
	const int depotIndex,
	const std::vector<int>& sequence,
	const EvaluationDetail detail,
	const bool trustedProblemData,
	RouteEvaluation& result) const {
	if (!trustedProblemData) {
		requireTemporalData(data);
	}

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
			routeArcValue(
				data.T,
				previousNode,
				customer,
				"travel-time",
				trustedProblemData);
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
		routeArcValue(
			data.T,
			previousNode,
			depotNode,
			"travel-time",
			trustedProblemData);
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
	const SolverContext& data,
	const TemporalConstraint& temporalConstraint)
	: data_(data),
	  temporalConstraint_(temporalConstraint),
	  trustedProblemData_(
		  data.problemDataFinalizedForSearch() &&
		  (temporalConstraint.isNoOp() ||
		   data.temporalProblemDataFinalizedForSearch())) {
}

void RouteEvaluator::validateProblemDataForTrustedEvaluation(
		const SolverContext& data,
		const bool requireTemporal) {
	if (data.numV < 0 || data.numL <= 0 ||
		data.numC < data.numV + data.numL) {
		throw std::logic_error(
			"route evaluation received inconsistent instance dimensions");
	}
	if (data.demCus == nullptr || data.invLoc == nullptr) {
		throw std::logic_error(
			"route evaluation requires initialized demand/capacity data");
	}
	if (data.vehCap < 0) {
		throw std::logic_error(
			"route evaluation requires a nonnegative vehicle capacity");
	}
	for (int customer = 0; customer < data.numV; ++customer) {
		if (data.demCus[customer] < 0) {
			throw std::domain_error(
				"route evaluation found a negative customer demand");
		}
	}
	for (int depot = 0; depot < data.numL; ++depot) {
		if (data.invLoc[data.numV + depot] < 0) {
			throw std::domain_error(
				"route evaluation found a negative depot capacity");
		}
	}
	if (data.D == nullptr) {
		throw std::logic_error(
			"route evaluation requires initialized travel-cost matrix");
	}
	for (int from = 0; from < data.numC; ++from) {
		if (data.D[from] == nullptr) {
			throw std::logic_error(
				"route evaluation requires initialized travel-cost matrix");
		}
		for (int to = 0; to < data.numC; ++to) {
			const double value = data.D[from][to];
			if (!std::isfinite(value) || value < 0.0) {
				throw std::domain_error(
					"route evaluation found an invalid travel-cost arc");
			}
		}
	}
	if (!requireTemporal) {
		return;
	}
	requireTemporalData(data);
	for (int from = 0; from < data.numC; ++from) {
		if (data.T[from] == nullptr) {
			throw std::logic_error(
				"hard-time-window evaluation requires an initialized "
				"travel-time matrix");
		}
		for (int to = 0; to < data.numC; ++to) {
			const double value = data.T[from][to];
			if (!std::isfinite(value) || value < 0.0) {
				throw std::domain_error(
					"hard-time-window evaluation found an invalid "
					"travel-time arc");
			}
		}
	}
}

RouteEvaluation RouteEvaluator::evaluate(
	const int depotIndex,
	const std::vector<int>& sequence,
	const EvaluationDetail detail) const {
	hgasp::recordHotPathCounter(
		trustedProblemData_
			? hgasp::HotPathCounter::EvaluatorTrustedCalls
			: hgasp::HotPathCounter::EvaluatorCheckedCalls);
	hgasp::recordHotPathCounter(
		hgasp::HotPathCounter::EvaluatorCustomersScanned,
		static_cast<std::uint64_t>(sequence.size()));
	if (detail == EvaluationDetail::FullSchedule) {
		hgasp::recordHotPathCounter(
			hgasp::HotPathCounter::EvaluatorFullScheduleCalls);
	}
	if (trustedProblemData_) {
		validateRouteIndices(data_, depotIndex, sequence);
	} else {
		validateCommonInput(data_, depotIndex, sequence);
	}

	RouteEvaluation result;
	result.customerCount = sequence.size();

	const int depotNode = data_.numV + depotIndex;
	int previousNode = depotNode;
	for (const int customer : sequence) {
		result.load += data_.demCus[customer];
		result.travelCost +=
			routeArcValue(
				data_.D,
				previousNode,
				customer,
				"travel-cost",
				trustedProblemData_);
		previousNode = customer;
	}
	result.travelCost +=
		routeArcValue(
			data_.D,
			previousNode,
			depotNode,
			"travel-cost",
			trustedProblemData_);
	result.capacityFeasible = result.load <= data_.vehCap;

	if (!temporalConstraint_.isNoOp()) {
		temporalConstraint_.evaluate(
			data_,
			depotIndex,
			sequence,
			detail,
			trustedProblemData_,
			result);
	}
	return result;
}

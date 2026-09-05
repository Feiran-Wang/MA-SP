#ifndef HGASP_OPERATORS_ESET_BUILDER_H_
#define HGASP_OPERATORS_ESET_BUILDER_H_

#include <algorithm>
#include <set>
#include <stdexcept>
#include <vector>

namespace hgasp {

inline std::vector<std::set<int>> buildStrategy3ESets(
		const std::vector<std::set<int>>& commconCycles,
		const int partitionCount) {
	if (partitionCount <= 0) {
		throw std::invalid_argument(
			"EAX Strategy 3 partition count must be positive");
	}

	std::vector<std::set<int>> nonEmptyComponents;
	nonEmptyComponents.reserve(commconCycles.size());
	for (const std::set<int>& component : commconCycles) {
		if (!component.empty()) {
			nonEmptyComponents.push_back(component);
		}
	}
	if (nonEmptyComponents.empty()) {
		return {};
	}

	const std::size_t outputCount = std::min(
		static_cast<std::size_t>(partitionCount),
		nonEmptyComponents.size());
	std::vector<std::set<int>> result(outputCount);
	for (std::size_t output = 0U; output < outputCount; ++output) {
		const std::size_t begin =
			output * nonEmptyComponents.size() / outputCount;
		const std::size_t end =
			(output + 1U) * nonEmptyComponents.size() / outputCount;
		for (std::size_t ordinal = begin; ordinal < end; ++ordinal) {
			result[output].insert(
				nonEmptyComponents[ordinal].begin(),
				nonEmptyComponents[ordinal].end());
		}
	}
	return result;
}

}  // namespace hgasp

#endif  // HGASP_OPERATORS_ESET_BUILDER_H_

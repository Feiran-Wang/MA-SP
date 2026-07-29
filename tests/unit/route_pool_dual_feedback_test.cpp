#include "RoutePool.h"
#include "read_data.h"

#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

std::mt19937 global_rng(2811U);

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

StandaloneRoute route(
        const int depot,
        const double cost,
        const int load,
        std::vector<int> customers) {
    StandaloneRoute value;
    value.locId = depot;
    value.routeCost = cost;
    value.totalLoad = load;
    value.sequence = std::move(customers);
    return value;
}

void testDualFeedbackUsesMasterIdentity() {
    read_data data;
    data.numV = 2;
    data.numL = 1;
    data.numC = 3;
    data.spMaxPoolSize = 20;
    data.define();

    RoutePool pool(&data);
    const std::vector<StandaloneRoute> snapshot = {
        route(0, 5.0, 2, {0}),
        route(0, 7.0, 3, {1}),
    };
    pool.setProtectedRoutes(
        RouteProtection::Backbone, snapshot);
    pool.updateDualFeedback(
        snapshot,
        {4.0, 6.0},
        {-0.25, 1.5});

    require(
        pool.getCustomerDuals() ==
            std::vector<double>({4.0, 6.0}),
        "customer dual feedback was not retained");
    require(
        std::fabs(
            pool.reducedCostFor(
                route(0, 100.0, 2, {0})) +
            0.25) < 1.0e-12,
        "route feedback was not keyed by depot and customer set");

    bool rejected = false;
    try {
        pool.updateDualFeedback(
            snapshot, {1.0}, {-0.25, 1.5});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(
        rejected,
        "malformed customer-dual vector was accepted");
}

}  // namespace

int main() {
    try {
        testDualFeedbackUsesMasterIdentity();
        std::cout << "route_pool_dual_feedback_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "route_pool_dual_feedback_test: "
                  << error.what() << '\n';
        return 1;
    }
}

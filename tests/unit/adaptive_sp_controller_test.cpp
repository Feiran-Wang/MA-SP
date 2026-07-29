#include "AdaptiveSpController.h"

#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

std::mt19937 global_rng(1803U);

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

AdaptiveSpPolicy policy() {
    AdaptiveSpPolicy value;
    value.minimumInterval = 100;
    value.localNewColumnThreshold = 10;
    value.localStagnationThreshold = 50;
    value.globalStagnationThreshold = 200;
    value.utilityEmaWeight = 0.5;
    value.highUtilityThreshold = 0.1;
    return value;
}

void testNoveltyIntervalAndUtilityFeedback() {
    AdaptiveSpController controller(policy());
    AdaptiveSpContext context;

    context.iteration = 100;
    context.masterVersion = 0;
    require(
        controller.decide(context).reason ==
            "unchanged_master",
        "unchanged master was not skipped");

    context.iteration = 99;
    context.masterVersion = 20;
    require(
        controller.decide(context).reason ==
            "local_minimum_interval",
        "minimum HGA interval was not enforced");

    context.iteration = 100;
    const AdaptiveSpDecision first =
        controller.decide(context);
    require(
        first.dispatch == SpDispatch::Local &&
            first.reason == "column_novelty" &&
            first.effectiveLocalThreshold == 10U,
        "initial column novelty did not trigger local SP");

    controller.report(
        SpDispatch::Local, 100, 20, 0.0, 10.0);
    context.iteration = 200;
    context.masterVersion = 35;
    const AdaptiveSpDecision lowUtilitySkip =
        controller.decide(context);
    require(
        lowUtilitySkip.dispatch == SpDispatch::None &&
            lowUtilitySkip.effectiveLocalThreshold == 20U,
        "zero-utility SP did not increase the novelty threshold");

    context.masterVersion = 40;
    require(
        controller.decide(context).dispatch ==
            SpDispatch::Local,
        "adapted novelty threshold did not trigger at its boundary");

    AdaptiveSpController productive(policy());
    productive.report(
        SpDispatch::Local, 100, 20, 10.0, 10.0);
    context.iteration = 200;
    context.masterVersion = 25;
    context.nonImprovingIterations = 0;
    const AdaptiveSpDecision highUtility =
        productive.decide(context);
    require(
        highUtility.dispatch == SpDispatch::Local &&
            highUtility.effectiveLocalThreshold == 5U,
        "productive SP did not lower the novelty threshold");
}

void testGlobalPriorityAndBudgetGuard() {
    AdaptiveSpController controller(policy());
    controller.report(
        SpDispatch::Local, 100, 20, 1.0, 10.0);

    AdaptiveSpContext context;
    context.iteration = 101;
    context.masterVersion = 21;
    context.scheduledGlobal = true;
    context.remainingCpuSeconds = 100.0;
    require(
        controller.decide(context).dispatch ==
            SpDispatch::Global,
        "scheduled global SP did not bypass the local interval");

    controller.report(
        SpDispatch::Global, 101, 21, 0.0, 10.0);
    context.masterVersion = 22;
    context.remainingCpuSeconds = 12.0;
    const AdaptiveSpDecision budgetSkip =
        controller.decide(context);
    require(
        budgetSkip.dispatch == SpDispatch::None &&
            budgetSkip.reason == "insufficient_cpu_budget",
        "remaining CPU guard did not suppress an oversized SP call");

    AdaptiveSpController stagnant(policy());
    context.scheduledGlobal = false;
    context.remainingCpuSeconds = 100.0;
    context.iteration = 200;
    context.masterVersion = 20;
    context.nonImprovingIterations = 200;
    require(
        stagnant.decide(context).reason ==
            "global_stagnation",
        "global stagnation did not trigger intensification");
}

void testGlobalBackoffAndOneShotFinalPhase() {
    AdaptiveSpController controller(policy());
    AdaptiveSpContext context;
    context.iteration = 200;
    context.nonImprovingIterations = 200;
    context.masterVersion = 20;
    require(
        controller.decide(context).dispatch ==
            SpDispatch::Global,
        "initial long-stagnation global SP was not dispatched");
    controller.report(
        SpDispatch::Global, 200, 20, 0.0, 10.0);

    context.iteration = 300;
    context.masterVersion = 30;
    const AdaptiveSpDecision backedOff =
        controller.decide(context);
    require(
        backedOff.dispatch == SpDispatch::None &&
            backedOff.reason == "global_backoff",
        "zero-utility global SP was immediately repeated");

    context.iteration = 600;
    require(
        controller.decide(context).dispatch ==
            SpDispatch::Global,
        "global exponential backoff never released");
    controller.report(
        SpDispatch::Global, 600, 30, 0.0, 10.0);
    context.iteration = 700;
    context.masterVersion = 40;
    context.scheduledGlobal = true;
    require(
        controller.decide(context).dispatch ==
            SpDispatch::Global,
        "scheduled cadence did not override global backoff");

    AdaptiveSpController finalController(policy());
    AdaptiveSpContext finalContext;
    finalContext.iteration = 100;
    finalContext.masterVersion = 10;
    finalContext.finalIntensificationPhase = true;
    require(
        finalController.decide(finalContext).dispatch ==
            SpDispatch::Global,
        "final intensification was not dispatched");
    ++finalContext.iteration;
    ++finalContext.masterVersion;
    const AdaptiveSpDecision repeated =
        finalController.decide(finalContext);
    require(
        repeated.dispatch == SpDispatch::None &&
            repeated.reason ==
                "final_intensification_already_issued",
        "final intensification was dispatched more than once");
}

void testLocalConsumptionDoesNotHideGlobalNovelty() {
    AdaptiveSpController controller(policy());
    controller.report(
        SpDispatch::Local, 100, 20, 1.0, 1.0);
    AdaptiveSpContext context;
    context.iteration = 101;
    context.masterVersion = 20;
    context.scheduledGlobal = true;
    const AdaptiveSpDecision decision =
        controller.decide(context);
    require(
        decision.dispatch == SpDispatch::Global &&
            decision.newColumnEvents == 20U,
        "local SP version consumption hid columns from global SP");
}

}  // namespace

int main() {
    try {
        testNoveltyIntervalAndUtilityFeedback();
        testGlobalPriorityAndBudgetGuard();
        testGlobalBackoffAndOneShotFinalPhase();
        testLocalConsumptionDoesNotHideGlobalNovelty();
        std::cout << "adaptive_sp_controller_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "adaptive_sp_controller_test: "
                  << error.what() << '\n';
        return 1;
    }
}

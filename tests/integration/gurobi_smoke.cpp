#include "gurobi_c++.h"

#include <cmath>
#include <iostream>

int main() {
  try {
    GRBEnv environment(true);
    environment.set(GRB_IntParam_OutputFlag, 0);
    environment.start();

    GRBModel model(environment);
    GRBVar x = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x");
    GRBLinExpr objective = x;
    model.setObjective(objective, GRB_MAXIMIZE);
    model.optimize();

    if (model.get(GRB_IntAttr_Status) != GRB_OPTIMAL ||
        std::abs(x.get(GRB_DoubleAttr_X) - 1.0) > 1.0e-9) {
      std::cerr << "Gurobi smoke test returned an unexpected solution.\n";
      return 1;
    }

    std::cout << "Gurobi C++ API smoke test passed.\n";
    return 0;
  } catch (const GRBException& error) {
    std::cerr << "Gurobi error " << error.getErrorCode() << ": "
              << error.getMessage() << '\n';
    return 2;
  }
}

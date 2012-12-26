#include <gtest/gtest.h>
#include "ipi/sco/solver_interface.hpp"
#include "ipi/logging.hpp"
#include "ipi/sco/expr_ops.hpp"
#include <cstdio>
#include <boost/foreach.hpp>
#include <iostream>
#undef IPI_LOG_THRESH
#define IPI_LOG_THRESH IPI_LEVEL_DEBUG
using namespace std;

using namespace ipi::sco;



TEST(solver_interface, setup_problem) {
  ModelPtr solver = createModel(SOLVER_GUROBI);
  vector<Var> vars;
  for (int i=0; i < 3; ++i) {
    char namebuf[5];
    sprintf(namebuf, "v%i", i);
    vars.push_back(solver->addVar(namebuf));
  }
  solver->update();

  AffExpr aff;
  cout << aff << endl;
  for (int i=0; i < 3; ++i) {
    exprInc(aff, vars[i]);
    solver->setVarBounds(vars[i], 0, 10);
  }
  aff.constant -= 3;
  cout << aff << endl;
  QuadExpr affsquared = exprSquare(aff);
  solver->setObjective(affsquared);
  solver->update();
  IPI_LOG_ALERT("objective: %s", affsquared);
  IPI_LOG_ALERT("please manually check that /tmp/solver-interface-test.lp matches this");
  solver->writeToFile("/tmp/solver-interface-test.lp");

  solver->optimize();

  vector<double> soln(3);
  for (int i=0; i < 3; ++i) {
    soln[i] = solver->getVarValue(vars[i]);
  }
  EXPECT_NEAR(aff.value(soln), 0, 1e-6);

  solver->removeVar(vars[2]);
  solver->update();
  EXPECT_EQ(solver->getVars().size(), 2);

}

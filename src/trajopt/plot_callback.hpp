#pragma once
#include "ipi/sco/optimizers.hpp"
#include "trajopt/common.hpp"
namespace trajopt {
class TrajOptProb;

Optimizer::Callback PlotCallback(TrajOptProb& prob);

}

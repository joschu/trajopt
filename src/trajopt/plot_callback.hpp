#pragma once
#include "sco/optimizers.hpp"
#include "trajopt/common.hpp"
namespace trajopt {
class TrajOptProb;

Optimizer::Callback TRAJOPT_API PlotCallback(TrajOptProb& prob);

}

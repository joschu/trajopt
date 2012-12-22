#include "utils.hpp"
#include "ipi/sco/solver_interface.hpp"
namespace trajopt {
TrajArray getTraj(const vector<double>& x, const VarArray& vars) {
  TrajArray out(vars.rows(), vars.cols());
  for (size_t i=0; i < (size_t)vars.rows(); ++i) {
    for (size_t j=0; j < (size_t)vars.cols(); ++j) {
      out(i,j) = vars(i,j).value(x.data());
    }
  }
  return out;
}

}

#pragma once
#include "typedefs.hpp"
#include <openrave/openrave.h>

namespace trajopt {

TrajArray getTraj(const vector<double>& x, const VarArray& vars);



}

#pragma once
#include <Eigen/Core>
#include "macros.h"

Eigen::MatrixX2d hull2d(const Eigen::MatrixX2d&);
void TRAJOPT_API PolygonToEquations(const Eigen::MatrixX2d& pts, Eigen::MatrixX2d& ab, Eigen::VectorXd& c);
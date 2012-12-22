#pragma once
#include <boost/function.hpp>
#include <Eigen/Dense>
/*
 * Numerical derivatives
 todo: implement robust (but slower) scheme from derivest.m
 */

namespace ipi {
namespace sco {
using boost::function;
using Eigen::VectorXd;
using Eigen::MatrixXd;

typedef function<double(VectorXd)> ScalarOfVector;
typedef function<VectorXd(VectorXd)> VectorOfVector;
typedef function<MatrixXd(VectorXd)> MatrixOfVector;



VectorXd calcForwardNumGrad(const ScalarOfVector& f, const VectorXd& x, double epsilon);
MatrixXd calcForwardNumJac(const VectorOfVector& f, const VectorXd& x, double epsilon);
void calcGradAndDiagHess(const ScalarOfVector& f, const VectorXd& x, double epsilon,
    double& y, VectorXd& grad, VectorXd& hess);
void calcGradHess(const ScalarOfVector& f, const VectorXd& x, double epsilon,
    double& y, VectorXd& grad, MatrixXd& hess);
VectorOfVector forwardNumGrad(const ScalarOfVector& f, double epsilon);
MatrixOfVector forwardNumJac(const VectorOfVector& f, double epsilon);



}}

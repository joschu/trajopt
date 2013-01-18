#include "trajopt/plot_callback.hpp"
#include "trajopt/common.hpp"
#include "osgviewer/osgviewer.hpp"
#include "utils/eigen_conversions.hpp"
#include <boost/foreach.hpp>
#include "trajopt/problem_description.hpp"
using namespace OpenRAVE;
using namespace util;
namespace trajopt {

void PlotTraj(OSGViewer& viewer, RobotAndDOF& rad, const TrajArray& x, vector<GraphHandlePtr>& handles) {
  RobotBase::RobotStateSaver saver = rad.Save();
  for (int i=0; i < x.rows(); ++i) {
    rad.SetDOFValues(toDblVec(x.row(i)));
    handles.push_back(viewer.PlotKinBody(rad.GetRobot()));
    SetTransparency(handles.back(), .35);
  }
}

void PlotCosts(OSGViewer& viewer, vector<CostPtr>& costs, vector<ConstraintPtr>& cnts, RobotAndDOF& rad, const VarArray& vars, const DblVec& x) {
  vector<GraphHandlePtr> handles;
  handles.clear();
  BOOST_FOREACH(CostPtr& cost, costs) {
    if (Plotter* plotter = dynamic_cast<Plotter*>(cost.get())) {
      plotter->Plot(x, *rad.GetRobot()->GetEnv(), handles);
    }
  }
  BOOST_FOREACH(ConstraintPtr& cnt, cnts) {
    if (Plotter* plotter = dynamic_cast<Plotter*>(cnt.get())) {
      plotter->Plot(x, *rad.GetRobot()->GetEnv(), handles);
    }
  }
  PlotTraj(viewer, rad, getTraj(x, vars), handles);
  viewer.Idle();
}



Optimizer::Callback PlotCallback(TrajOptProb& prob) {
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(prob.GetEnv());
  if (!viewer) {
    printf("creating a new viewer\n");
    viewer.reset(new OSGViewer(prob.GetEnv()));
    prob.GetEnv()->AddViewer(viewer);
  }
  vector<ConstraintPtr> cnts = prob.getConstraints();
  return boost::bind(&PlotCosts, boost::ref(*viewer),
                      boost::ref(prob.getCosts()),
                      cnts,
                      boost::ref(*prob.GetRAD()),
                      boost::ref(prob.GetVars()),
                      _1);
}

}

#include <boost/python.hpp>
#include "trajopt/collision_checker.hpp"

using namespace trajopt;
using namespace boost::python;


class PyCollisionPairIgnorer {
};
class PyCollisionChecker {
};


BOOST_PYTHON_MODULE(trajoptpy) {
  // class_<PyCollisionChecker, boost::noncopyable> cc_("CollisionChecker", no_init);
  // cc_.def("AllVsAll", &PyCollisionChecker::AllVsAll);
  // 
  // class_<Collision, boost::noncopyable> col_("Collision", no_init);
        
}
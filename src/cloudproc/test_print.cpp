#include "utils/stl_to_string.hpp"
#include <iostream>
#include <Eigen/Core>
using namespace std;
using namespace Eigen;
int main() {

  map<int, set<int> > x;
  cout << util::Str(x) << endl;

}

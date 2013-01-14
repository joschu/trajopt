#include <Eigen/Core>
#include <iostream>
using namespace Eigen;
using namespace std;
void PolygonToEquations(const MatrixX2d& pts, MatrixX2d& ab, VectorXd& c) {
  // ax + by + c <= 0
  // assume polygon is convex

  Vector2d p0 = pts.row(0);

  for (int i=0; i < pts.rows(); ++i) {
    int i1 = (i+1) % pts.rows();
    double x0 = pts(i,0),
        y0 = pts(i,1),
        x1 = pts(i1,0),
        y1 = pts(i1,1);
    ab(i,0) = -(y1 - y0);
    ab(i,1) = x1 - x0;
    ab.row(i).normalize();
    c(i) = -ab.row(i).dot(pts.row(i));
  }

}

int main() {
  MatrixX2d m(4,2), ab(4,2);
  VectorXd c(4);
  m << 0,0,
      0,1,
      1,1,
      1,0;
  PolygonToEquations(m, ab, c);
  cout << "ab: " << ab << endl;
  cout << "c: " << c.transpose() << endl;
}

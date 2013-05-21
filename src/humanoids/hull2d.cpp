#include "hull2d.hpp"
#include <algorithm>
#include <vector>
using namespace Eigen;
using namespace std;


// http://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain
namespace {

typedef float coord_t;         // coordinate type
typedef float coord2_t;  // must be big enough to hold 2*max(|coordinate|)^2


struct Point {
        coord_t x, y;
        Point() {}
        Point(coord_t x, coord_t y) : x(x), y(y) {}

        bool operator <(const Point &p) const {
                return x < p.x || (x == p.x && y < p.y);
        }
};

// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
coord2_t cross(const Point &O, const Point &A, const Point &B)
{
        return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

// Returns a list of points on the convex hull in counter-clockwise order.
// Note: the last point in the returned list is the same as the first one.
vector<Point> convex_hull(vector<Point> P)
{
        int n = P.size(), k = 0;
        vector<Point> H(2*n);

        // Sort points lexicographically
        sort(P.begin(), P.end());

        // Build lower hull
        for (int i = 0; i < n; i++) {
                while (k >= 2 && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
                H[k++] = P[i];
        }

        // Build upper hull
        for (int i = n-2, t = k+1; i >= 0; i--) {
                while (k >= t && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
                H[k++] = P[i];
        }

        H.resize(k);
        return H;
}

}

Eigen::MatrixX2d hull2d(const Eigen::MatrixX2d& in) {
  vector<Point> points(in.rows());
  for (int i=0; i < in.rows(); ++i) points[i] = Point(in(i,0), in(i,1));
  vector<Point> hull = convex_hull(points);
  Eigen::MatrixX2d out(hull.size()-1,2);
  for (int i=0; i < hull.size()-1; ++i) {
    out(i,0) = hull[i].x;
    out(i,1) = hull[i].y;
  }
  return out;
}

void PolygonToEquations(const MatrixX2d& pts, MatrixX2d& ab, VectorXd& c) {
  // ax + by + c <= 0
  // assume polygon is convex
  ab.resize(pts.rows(),2);
  c.resize(pts.rows());
  // Vector2d p0 = pts.row(0);

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

  Vector2d centroid = pts.colwise().mean();
  if (ab.row(0) * centroid + c(0) >= 0) {
    ab *= -1;
    c *= -1;
  }

}


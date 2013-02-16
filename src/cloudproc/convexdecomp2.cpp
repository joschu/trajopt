#include <Eigen/Core>
#include <set>
#include <map>
#include <queue>
#include <boost/foreach.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include "cloudproc.hpp"
#include <iostream>

using namespace Eigen;
using namespace std;
using namespace pcl;

//#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#define DEBUG_PRINT(...)


typedef vector<int> IntVec;
typedef vector<float> FloatVec;
namespace {

template <typename T>
struct Frontier {
  set<T> m_excluded;
  queue<T> m_frontier;
  T pop() {
    T out = m_frontier.front();
    m_frontier.pop();
    return out;
  }
  void push(const T& x) {
    if (m_excluded.find(x) == m_excluded.end()) m_frontier.push(x);
  }
  void push(const vector<T>& x) {
    BOOST_FOREACH(const T& t, x) {
      push(t);
      exclude(t);
    }
  }
  void exclude(const T& x) {
    m_excluded.insert(x);
  }
  bool empty() {
    return m_frontier.empty();
  }
};

const int UNLABELED = -1;
const int NO_NEXT_LABEL = -1;
int getNextUnlabeled(const vector<int>& labels, int start) {
  for (int i=start; i < labels.size(); ++i) {
    if (labels[i] == UNLABELED) {
      return i;
    }
  }
  return NO_NEXT_LABEL;
}

struct AlmostConvex {
  vector<PointNormal> m_pns;
  vector<int> m_inds;

  float deficit (const PointNormal& pn1, const PointNormal& pn2) {
    Vector3f p2minusp1 = pn2.getVector3fMap() - pn1.getVector3fMap();
    return fmaxf(p2minusp1.dot(pn2.getNormalVector3fMap()), -p2minusp1.dot(pn1.getNormalVector3fMap()))/p2minusp1.norm();
  }

  float calcMaxDeficit(const PointNormal& pn) {
    float out = 0;
    BOOST_FOREACH(const PointNormal& pn1, m_pns) {
      out = fmaxf(out, deficit(pn, pn1));
    }
    return out;
  }
  void add(const PointNormal& pn, int ind) {
    m_pns.push_back(pn);
    m_inds.push_back(ind);
  }


};


vector<int> getNeighbors(const pcl::KdTreeFLANN<PointXYZ>& tree, int i_pt, int k_neighbs, float maxdist) {
  k_neighbs += 1;
  IntVec neighb_inds(k_neighbs, -666);
  FloatVec sqdists(k_neighbs, -666);
//  int n_neighbs = tree.nearestKSearch(i_pt, k_neighbs, neighb_inds, sqdists);
  int n_neighbs = tree.radiusSearch(i_pt, maxdist, neighb_inds, sqdists, k_neighbs);
  return vector<int>(neighb_inds.begin()+1, neighb_inds.begin() + n_neighbs);
}

vector<int> getUnlabeled(const vector<int>& nbs, const vector<int>& labels) {
  vector<int> out;
  BOOST_FOREACH(const int& i, nbs) if (labels[i] == UNLABELED) out.push_back(i);
  return out;
}

}

namespace cloudproc {


void ConvexDecomp(pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud, float thresh,
    /*optional outputs: */ std::vector<IntVec>* indices, std::vector< IntVec >* hull_indices) {

  float nb_rad = 2*thresh;
  int k_neighbs = 5;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>(true));
  tree->setEpsilon(0);
  tree->setInputCloud (toXYZ<PointNormal>(cloud));
  int n_pts = cloud->size();


  vector<int> labels(n_pts, UNLABELED);
  int label_cur = 0;

  int i_seed = 0;
  while ( (i_seed = getNextUnlabeled(labels, i_seed)) != NO_NEXT_LABEL) {


    DEBUG_PRINT("growing cluster from seed %i\n", i_seed);
    Frontier<int> frontier;
    AlmostConvex ac;

    ac.add(cloud->points[i_seed], i_seed);
    frontier.exclude(i_seed);
    labels[i_seed] = label_cur;

    vector<int> nbs = getNeighbors(*tree, i_seed, k_neighbs, nb_rad);
    vector<int> unlabeled_nbs = getUnlabeled(nbs, labels);
    frontier.push(unlabeled_nbs);

    while (!frontier.empty()) {
      int i_cur = frontier.pop();
      float deficit = ac.calcMaxDeficit(cloud->points[i_cur]);
      if (deficit < thresh) {
        ac.add(cloud->points[i_cur], i_cur);
        labels[i_cur] = label_cur;
        vector<int> nbs = getNeighbors(*tree, i_cur, k_neighbs, nb_rad);
        vector<int> unlabeled_nbs = getUnlabeled(nbs, labels);
        frontier.push(unlabeled_nbs);
//        BOOST_FOREACH(int i, unlabeled_nbs) assert( (cloud->points[i].getVector3fMap() - cloud->points[i_cur].getVector3fMap()).norm() < nb_rad );
        DEBUG_PRINT("added %i\n", i_cur);
        if (label_cur==263)cout << cloud->points[i_cur] << endl;
      }
      else {
        DEBUG_PRINT("rejected %i\n", i_cur);
      }
    }


    if (indices != NULL) indices->push_back(ac.m_inds);
    DEBUG_PRINT("finished cluster %i of size %i\n", label_cur, ac.m_inds.size());
    ++label_cur;


  }


}



}

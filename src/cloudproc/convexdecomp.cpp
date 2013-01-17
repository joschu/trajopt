//#include "convexdecomp.hpp"
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <set>
#include <map>
#include <queue>
#include <boost/foreach.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include "sphere_sampling.hpp"
#include <iostream>
#include "utils/stl_to_string.hpp"
#include "convexdecomp.hpp"
using namespace pcl;
using namespace Eigen;
using namespace std;
using namespace util;

typedef Matrix<bool, Dynamic, Dynamic> MatrixXb;
typedef Matrix<bool, Dynamic, 1> VectorXb;
typedef vector<int> IntVec;
typedef set<int> IntSet;
typedef vector<float> FloatVec;
typedef map<int, int> Int2Int; // ass to ass
typedef map<int, IntSet> Int2IntSet;

//#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#define DEBUG_PRINT(...)

namespace {

struct SupInfo {
  IntVec inds;
  FloatVec sups;
  float best;
};

vector<int> getNeighbors(const pcl::KdTreeFLANN<PointXYZ>& tree, int i_pt, int k_neighbs, float maxdist) {
  k_neighbs += 1;
  IntVec neighb_inds(k_neighbs, -666);
  FloatVec sqdists(k_neighbs, -666);
//  int n_neighbs = tree.nearestKSearch(i_pt, k_neighbs, neighb_inds, sqdists);
  int n_neighbs = tree.radiusSearch(i_pt, maxdist, neighb_inds, sqdists, k_neighbs);
  return vector<int>(neighb_inds.begin()+1, neighb_inds.begin() + n_neighbs);
}


}


namespace cloudproc {

void ConvexDecomp1(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, float thresh,
    /*optional outputs: */ std::vector<IntVec>* indices, std::vector< IntVec >* hull_indices) {
  MatrixXf dirs = getSpherePoints(1);
  ConvexDecomp(cloud, dirs, thresh, indices, hull_indices);
}


void ConvexDecomp(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const Eigen::MatrixXf& dirs, float thresh,
    /*optional outputs: */ std::vector<IntVec>* indices, std::vector< IntVec >* hull_indices) {

  int k_neighbs = 5;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>(true));
  tree->setEpsilon(0);
  tree->setInputCloud (cloud);
  int n_pts = cloud->size();
  int n_dirs = dirs.rows();




  DEBUG_PRINT("npts, ndirs %i %i\n", n_pts, n_dirs);

  MatrixXf dirs4(n_dirs, 4);
  dirs4.leftCols(3) = dirs;
  dirs4.col(3).setZero();
  MatrixXf pt2supports = Map< const Matrix<float, Dynamic, Dynamic,RowMajor > >(reinterpret_cast<const float*>(cloud->points.data()), n_pts, 4) * dirs4.transpose();


  const int UNLABELED = -1;
  IntVec pt2label(n_pts, UNLABELED);

  IntSet alldirs;
  for (int i=0; i < n_dirs; ++i) alldirs.insert(i);

  int i_seed=0;
  int i_label = 0;

  // each loop cycle, add a new cluster
  while (true) {




    // find first unlabeled point
    while (true) {
      if (i_seed == n_pts) return;
      if (pt2label[i_seed] == UNLABELED) break;
      ++i_seed;
    }

    pt2label[i_seed] = i_label;
    map<int, IntSet> pt2dirs;
    pt2dirs[i_seed] = alldirs;
    vector<SupInfo> dir2supinfo(n_dirs);
    for (int i_dir=0; i_dir < n_dirs; ++i_dir) {
      float seedsup = pt2supports(i_seed, i_dir);
      dir2supinfo[i_dir].inds.push_back(i_seed);
      dir2supinfo[i_dir].sups.push_back(seedsup);
      dir2supinfo[i_dir].best = seedsup;

    }


    DEBUG_PRINT("seed: %i\n", i_seed);

    IntSet exclude_frontier;
    exclude_frontier.insert(i_seed);
    queue<int> frontier;

    BOOST_FOREACH(const int& i_nb,  getNeighbors(*tree, i_seed, k_neighbs, 2*thresh)) {
      if (pt2label[i_nb]==UNLABELED && exclude_frontier.find(i_nb) == exclude_frontier.end()) {
        DEBUG_PRINT("adding %i to frontier\n", i_nb);
        frontier.push(i_nb);
        exclude_frontier.insert(i_nb);
      }
    }





    while (!frontier.empty()) {

#if 0 // for serious debugging
      vector<int> clu;
      BOOST_FOREACH(Int2IntSet::value_type& pt_dir, pt2dirs) {
        clu.push_back(pt_dir.first);
      }
      MatrixXd sup_pd(clu.size(), n_dirs);
      for (int i=0; i < clu.size(); ++i) {
        for (int i_dir = 0; i_dir < n_dirs; ++i_dir) {
          sup_pd(i,i_dir) = pt2supports(clu[i], i_dir);
        }
      }
      for (int i_dir=0; i_dir < n_dirs; ++i_dir) {
        IntSet nearext;
        for (int i=0; i < clu.size(); ++i) {
          if (sup_pd.col(i_dir).maxCoeff() - sup_pd(i,i_dir) < thresh) {
            nearext.insert(clu[i]);
          }
        }
        assert( toSet(dir2supinfo[i_dir].inds) == nearext );
      }
      printf("ok!\n");
#endif




      int i_cur = frontier.front();
      frontier.pop();
//      printf("cur: %i\n", i_cur);
      DEBUG_PRINT("pt2dirs %s", Str(pt2dirs).c_str());

      bool reject = false;

      Int2Int pt2decrement;
      for (int i_dir = 0; i_dir < n_dirs; ++i_dir) {
        float cursup = pt2supports(i_cur, i_dir);
        SupInfo& si = dir2supinfo[i_dir];
        if (cursup > si.best) {
          for (int i=0; i < si.inds.size(); ++i) {
            float sup = si.sups[i];
            int i_pt = si.inds[i];
            if (cursup - sup > thresh) {
              pt2decrement[i_pt] = pt2decrement[i_pt] + 1;
              DEBUG_PRINT("decrementing %i (dir %i)\n", i_pt, i_dir);
            }
          }
        }
      }

      DEBUG_PRINT("pt2dec: %s", Str(pt2decrement).c_str());

      BOOST_FOREACH(const Int2Int::value_type& pt_dec, pt2decrement) {
        if (pt_dec.second == pt2dirs[pt_dec.first].size()) {
          reject = true;
          break;
        }
      }

      DEBUG_PRINT("reject? %i\n", reject);
      if (!reject) {
        pt2label[i_cur] = i_label;
        pt2dirs[i_cur] = IntSet();
        for (int i_dir = 0; i_dir < n_dirs; ++i_dir) {
          float cursup = pt2supports(i_cur, i_dir);
          if (cursup > dir2supinfo[i_dir].best - thresh) pt2dirs[i_cur].insert(i_dir);
        }


        for (int i_dir = 0; i_dir < n_dirs; ++i_dir) {
          float cursup = pt2supports(i_cur, i_dir);
          SupInfo& si = dir2supinfo[i_dir];
          if (cursup > si.best) {

            IntVec filtinds;
            FloatVec filtsups;
            for (int i = 0; i < si.inds.size(); ++i) {
              float sup = si.sups[i];
              int i_pt = si.inds[i];
              if (cursup - sup > thresh) {
                pt2dirs[i_pt].erase(i_dir);
              }
              else {
                filtinds.push_back(i_pt);
                filtsups.push_back(sup);
              }
            }
            si.inds = filtinds;
            si.sups = filtsups;
            si.inds.push_back(i_cur);
            si.sups.push_back(cursup);
            si.best = cursup;
          }
          else if (cursup > si.best - thresh) {
            si.inds.push_back(i_cur);
            si.sups.push_back(cursup);
          }
        }
        BOOST_FOREACH(const int& i_nb,  getNeighbors(*tree, i_cur, k_neighbs, 2*thresh)) {
          if (pt2label[i_nb]==UNLABELED && exclude_frontier.find(i_nb) == exclude_frontier.end()) {
            DEBUG_PRINT("adding %i to frontier\n", i_nb);
            frontier.push(i_nb);
            exclude_frontier.insert(i_nb);
          }
        }

      } // if !reject
      else {
      }

    } // while frontier nonempty

    if (indices != NULL) {
      indices->push_back(IntVec());
      BOOST_FOREACH(Int2IntSet::value_type& pt_dir, pt2dirs) {
        indices->back().push_back(pt_dir.first);
      }
    }
    if (hull_indices != NULL) {
      hull_indices->push_back(IntVec());
      BOOST_FOREACH(Int2IntSet::value_type& pt_dirs, pt2dirs) {
        BOOST_FOREACH(const int& dir, pt_dirs.second) {
          if (pt2supports(pt_dirs.first, dir) == dir2supinfo[dir].best) {
            hull_indices->back().push_back(pt_dirs.first);
            break;
          }
        }
      }
      DEBUG_PRINT("hull size: %i/%i\n", hull_indices->back().size(), pt2dirs.size());
    }

    ++i_label;
  }


}

}

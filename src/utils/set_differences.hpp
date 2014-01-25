#include <boost/foreach.hpp>
#include <set>


template <typename T>
void SetDifferences(const std::vector<T>& A, const std::vector<T>& B, std::vector<T>& AMinusB, std::vector<T>& BMinusA) {
  std::set<T> Aset, Bset;
  AMinusB.clear();
  BMinusA.clear();
  BOOST_FOREACH(const T& a, A) {
    Aset.insert(a);
  }
  BOOST_FOREACH(const T& b, B) {
    Bset.insert(b);
  }
  BOOST_FOREACH(const T& a, A) {
    if (Bset.count(a) == 0) AMinusB.push_back(a);
  }
  BOOST_FOREACH(const T& b, B) {
    if (Aset.count(b) == 0) BMinusA.push_back(b);
  }
}
#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <sstream>
namespace util {

using std::string;
using std::vector;

//std::string Str(const vector<double>& x);
//std::string Str(const vector<float>& x);
//std::string Str(const vector<int>& x);

template<class T>
std::string Str(const vector<T>& x) {
  std::stringstream ss;
  ss << "(";
  if (x.size() > 0) ss << x[0];
  for(size_t i = 1; i < x.size(); ++i)
    ss << ", " << x[i];
  ss << ")";
  return ss.str();
}

template<class T>
std::string Str(const T& x) {
  std::stringstream ss;
  ss << x;
  return ss.str();
}

}


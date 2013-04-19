
namespace py = boost::python;
py::object np_mod; 

py::list toPyList(const std::vector<int>& x) {
  py::list out;
  for (int i=0; i < x.size(); ++i) out.append(x[i]);
  return out;
}

template<typename T>
struct type_traits {
  static const char* npname;
};
template<> const char* type_traits<float>::npname = "float32";
template<> const char* type_traits<int>::npname = "int32";
template<> const char* type_traits<double>::npname = "float64";
template<> const char* type_traits<unsigned char>::npname = "uint8";

template <typename T>
T* getPointer(const py::object& arr) {
  long int i = py::extract<long int>(arr.attr("ctypes").attr("data"));
  T* p = (T*)i;
  return p;
}

template<typename T>
py::object toNdarray1(const T* data, size_t dim0) {
  py::object out = np_mod.attr("empty")(py::make_tuple(dim0), type_traits<T>::npname);
  T* p = getPointer<T>(out);
  memcpy(p, data, dim0*sizeof(T));
  return out;
}
template<typename T>
py::object toNdarray2(const T* data, size_t dim0, size_t dim1) {
  py::object out = np_mod.attr("empty")(py::make_tuple(dim0, dim1), type_traits<T>::npname);
  T* pout = getPointer<T>(out);
  memcpy(pout, data, dim0*dim1*sizeof(T));
  return out;
}
template<typename T>
py::object toNdarray3(const T* data, size_t dim0, size_t dim1, size_t dim2) {
  py::object out = np_mod.attr("empty")(py::make_tuple(dim0, dim1, dim2), type_traits<T>::npname);
  T* pout = getPointer<T>(out);
  memcpy(pout, data, dim0*dim1*dim2*sizeof(T));
  return out;
}
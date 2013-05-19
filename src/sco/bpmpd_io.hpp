#include <boost/serialization/vector.hpp>
#include <vector>
using std::vector;

// bpmpd(&m, &n, &nz, &qn, &qnz, acolcnt.data(), acolidx.data(), acolnzs.data(), qcolcnt.data(), qcolidx.data(), qcolnzs.data(),
//     rhs.data(), obj.data(), lbound.data(), ubound.data(), 
//     primal.data(), dual.data(), status.data(), &BIG, &code, &opt, &memsiz);


struct bpmpd_input
{

    friend class boost::serialization::access;
    // When the class Archive corresponds to an output archive, the
    // & operator is defined similar to <<.  Likewise, when the class Archive
    // is a type of input archive the & operator is defined similar to >>.
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & m;
        ar & n;
        ar & nz;
        ar & qn;
        ar & qnz;
        ar & acolcnt;
        ar & acolidx;
        ar & acolnzs;
        ar & qcolcnt;
        ar & qcolidx;
        ar & qcolnzs;
        ar & rhs;
        ar & obj;
        ar & lbound;
        ar & ubound;
    }
    int m, n, nz, qn, qnz;
    vector<int> acolcnt, acolidx;
    vector<double> acolnzs;
    vector<int> qcolcnt, qcolidx;
    vector<double> qcolnzs;
    vector<double> rhs, obj, lbound, ubound;

  bpmpd_input() {}
  bpmpd_input(int m, int n, int nz, int qn, int qnz, 
              const vector<int>& acolcnt, const vector<int>& acolidx, const vector<double>& acolnzs, 
              const vector<int>& qcolcnt, const vector<int>& qcolidx, const vector<double>& qcolnzs, 
              const vector<double>& rhs, const vector<double>& obj, const vector<double>& lbound, const vector<double>& ubound) :
    m(m), n(n), nz(nz), qn(qn), qnz(qnz), acolcnt(acolcnt), acolidx(acolidx), acolnzs(acolnzs), qcolcnt(qcolcnt), qcolidx(qcolidx), qcolnzs(qcolnzs),
    rhs(rhs), obj(obj), lbound(lbound), ubound(ubound) {}
};



struct bpmpd_output
{

    friend class boost::serialization::access;
    // When the class Archive corresponds to an output archive, the
    // & operator is defined similar to <<.  Likewise, when the class Archive
    // is a type of input archive the & operator is defined similar to >>.
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & primal;
        ar & dual;
        ar & status;
        ar & code;
        ar & opt;
    }
    // primal.data(), dual.data(), status.data(), &BIG, &code, &optpublic:

  vector<double> primal, dual;
  vector<int> status;
  int code;    
  double opt;
  bpmpd_output() {}    
  bpmpd_output(const vector<double>& primal, const vector<double>& dual, const vector<int>& status, int code, double opt)  :
    primal(primal), dual(dual), status(status), code(code), opt(opt) {}
};

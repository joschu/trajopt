#include "bpmpd_io.hpp"
#include <unistd.h>
#include <boost/filesystem.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <iostream>
#include <fstream>
#include "utils/stl_to_string.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

using namespace std;

namespace fs=boost::filesystem;
extern "C" {
  extern void bpmpd(int *, int *, int *, int *, int *, int *, int *,
         double *, int *, int *, double *, double *, double *, double *,
         double *, double *, double *, int *, double *, int *, double *, int *);  
}

int main(int argc, char** argv) {
  int counter=0;
  while (true) {
    if  (fs::exists("/tmp/fs/bpmpd_input_ready")) {
      ifstream ifs("/tmp/fs/bpmpd_input");
      boost::archive::binary_iarchive ia(ifs);
      bpmpd_input bi;
      ia >> bi;
      remove("/tmp/fs/bpmpd_input_ready");
      bpmpd_output bo;
   
      int memsiz = 0;
      double BIG = 1e30;
      bo.primal.resize(bi.m+bi.n);
      bo.dual.resize(bi.m+bi.n);
      bo.status.resize(bi.m+bi.n);
      
// #define DBG(expr) cout << #expr << ": " << CSTR(expr) << std::endl
//       DBG(bi.m);
//       DBG(bi.n);
//       DBG(bi.nz);
//       DBG(bi.qn);
//       DBG(bi.qnz);
//       DBG(bi.acolcnt);
//       DBG(bi.acolidx);
//       DBG(bi.acolnzs);
//       DBG(bi.qcolcnt);
//       DBG(bi.qcolidx);
//       DBG(bi.qcolnzs);
//       DBG(bi.rhs);
//       DBG(bi.obj);
//       DBG(bi.lbound);
//       DBG(bi.ubound);
      
      boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time( );      
      bpmpd(&bi.m, &bi.n, &bi.nz, &bi.qn, &bi.qnz, bi.acolcnt.data(), bi.acolidx.data(), bi.acolnzs.data(), bi.qcolcnt.data(), bi.qcolidx.data(), bi.qcolnzs.data(),
          bi.rhs.data(), bi.obj.data(), bi.lbound.data(), bi.ubound.data(), 
          bo.primal.data(), bo.dual.data(), bo.status.data(), &BIG, &bo.code, &bo.opt, &memsiz);
      boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time( );      
      std::cout << "ELAPSED" << end-start << std::endl;
   
      {
        ofstream ofs("/tmp/fs/bpmpd_output");
        boost::archive::binary_oarchive oa(ofs);
        oa << bo;                
      }
      ofstream ofs("/tmp/fs/bpmpd_output_ready");
     }
     else {
       usleep(100);
       if (counter % 10000 == 0) printf("bpmpd_caller waiting\n");
       ++counter;
     }    
  }
   
}
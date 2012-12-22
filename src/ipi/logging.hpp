// Copyright (c) 2012, Industrial Perception, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//     * Redistributions of source code must retain the above
//       copyright notice, this list of conditions and the following
//       disclaimer.
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials
//       provided with the distribution.
//     * Neither the name of the Industrial Perception, Inc. nor the
//       names of its contributors may be used to endorse or promote
//       products derived from this software without specific prior
//       written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Industrial
// Perception, Inc. BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once
#include <iostream>
#include <boost/format.hpp>
#include <boost/format/exceptions.hpp>
#include <boost/noncopyable.hpp>
namespace ipi
{
/**
   \namespace ipi::logging
   \brief Simple logging library.

  Design goals:

  - typesafe
  - minimal dependencies (boost::format)
  - easy to swap out backend
  - compliant with printf style formatting
  - if it has an ostream operator<< then it should be loggable.
  - compile away at below a threshold
  - runtime filters
  - syslog compliant
  - low cognitive load
  - debugger friendly
  - emacs friendly
  - allow trace statements used during development to be left in place

  \h1  Usage

  Prefer the macros.  Here is a simple example::

  \code
  //force the compile time logging threshold
  #undef IPI_LOG_THRESH
  #define IPI_LOG_THRESH IPI_LEVEL_WARNING
  #include "ipi/logging.hpp" //or similar
  int main(){
    // set the filter level, "runtime" threshold
    IPI_LOG_FILTER(ipi::logging::ERR);
    // some debug statements
    IPI_LOG_DEBUG("a format string: %0.3f %s", 3.14, foo_obj);
    IPI_LOG_ERR("A error, no format");
  }
  \endcode
  All log macros, follow the following signature::

  \code
  IPI_LOG_<LEVEL>(<format string>, <format args>...)
  \endcode

  You may pass upto 10 arguments to the format string.  If you wish
  to send an arbitrary type, pleas use %s.  See boost::format documentation.

  Levels may be (in decreasing severity order):

  - EMERG
  - ALERT
  - CRIT
  - ERR
  - WARNING
  - NOTICE
  - INFO
  - DEBUG
  - TRACE

  See their definitions for semantic comments.

  The default levels for compile time and runtime are WARNING

  The header format may be customized at runtime through the IPI_LOG_FORMAT
  environment variable.

  The format string should look something like :
  \code
  "[%1%] %2%:%3%: %4%"
  \endcode

  - 1 is the level
  - 2 is the file
  - 3 is the line number
  - 4 is the pretty function name

  You may omit these and put them in any order:

  \code
  $ IPI_LOG_FORMAT="[%1%] %4%: " ipi/bin/ipi-core-unit
  \endcode

  You may also set the starting runtime level with an environment variable,
  IPI_LOG_THRESH=<LEVEL>::

  $ IPI_LOG_THRESH=TRACE ipi/bin/ipi-core-unit

  Compile options
  ---------------

  To set a compile time logging threshhold, define IPI_LOG_THRESH
  before the logging header is included.  Try to include logging only
  in implementation files if possible::

  #undef IPI_LOG_THRESH
  #define IPI_LOG_THRESH IPI_LEVEL_WARNING
  #include "logging.hpp"

  Or you may define IPI_LOG_THRESH in compile flags.. -DIPI_LOG_THRESH=IPI_LEVEL_TRACE

  The value of IPI_LOG_THRESH should be one of the defined IPI_LEVEL_<LEVEL>
  so that the compiler can succeed in comparing values.

  The backend may be specified as follows::

  #define IPI_LOG_BACKEND ipi::sys_log
  #include "logging.hpp"

  Available backends are ipi::sys_log, ipi::stdio_log.  stdio_log is the default.

  To implement a backend, look at the stdio_log or the tests for an example.
*/
namespace logging {
/**
 * Logging levels, used for filtering at compile time or at run time.
 */
enum level_t
  {
    EMERG=0, //!<      system is unusable
    ALERT, //!<      action must be taken immediately
    CRIT, //!<       critical conditions
    ERR, //!<        error conditions
    WARNING, //!<    warning conditions
    NOTICE, //!<     normal, but significant, condition
    INFO, //!<       informational message
    DEBUG, //!<      debug-level message
    TRACE, //!<      inner loop devel debug
  };

/**
 * in gdb try:
 * break ipi::logging::log_break()
 */
void log_break();

/** global log filter level, runtime.
 */
level_t filter();

/** set the global log filter level, runtime.
 */
void filter(level_t);

/**
 * return a string repr of a log level_t.
 */
const char* level_str(level_t l);
level_t str_level(const char* l);

template <typename backend_t>
struct logger: boost::noncopyable
{
  /**
   * Set runtime filter.  This will toggle all loggers
   * that are less then or equal to the filter level.
   */
  void filter(level_t l) const{
    ipi::logging::filter(l);
  }

  level_t filter() const{
    return ipi::logging::filter();
  }

  void flush(level_t l, const std::string& header, const std::string& message){
    backend_t()(l, header,message);
  }

  static inline logger<backend_t>& instance(){
    static logger<backend_t> lg; //singleton
    return lg;
  }

};

/**
 *  create a log header string. should include level string, pretty function name, file and line number.
 */
std::string log_header(const char* level, const char* pretty_function, const char* filename, unsigned line);

/**
 * nullery format string, just pass through.
 */
std::string log_message(const char* fmt);

template<typename T0>
inline std::string log_message( const char* fmt, const T0& t0){
  return boost::str(boost::format(fmt) % t0);
}
template<typename T0,typename T1>
inline std::string log_message( const char* fmt, const T0& t0, const T1& t1){
  return boost::str(boost::format(fmt) % t0 % t1);
}
template<typename T0,typename T1,typename T2>
inline std::string log_message( const char* fmt, const T0& t0, const T1& t1, const T2& t2){
  return boost::str(boost::format(fmt) % t0 % t1 % t2);
}
template<typename T0,typename T1,typename T2, typename T3>
inline std::string log_message( const char* fmt, const T0& t0, const T1& t1, const T2& t2, const T3& t3){
  return boost::str(boost::format(fmt) % t0 % t1 % t2 % t3);
}

template<typename T0,typename T1,typename T2, typename T3, typename T4>
inline std::string log_message( const char* fmt, const T0& t0, const T1& t1, const T2& t2, const T3& t3, const T4& t4){
  return boost::str(boost::format(fmt) % t0 % t1 % t2 % t3 % t4);
}

template<typename T0,typename T1,typename T2, typename T3, typename T4, typename T5>
inline std::string log_message( const char* fmt, const T0& t0, const T1& t1, const T2& t2, const T3& t3, const T4& t4, const T5& t5){
  return boost::str(boost::format(fmt) % t0 % t1 % t2 % t3 % t4 % t5);
}

template<typename T0,typename T1,typename T2, typename T3, typename T4, typename T5, typename T6>
inline std::string log_message( const char* fmt, const T0& t0, const T1& t1, const T2& t2, const T3& t3, const T4& t4, const T5& t5, const T6& t6){
  return boost::str(boost::format(fmt) % t0 % t1 % t2 % t3 % t4 % t5 % t6);
}

template<typename T0,typename T1,typename T2, typename T3, typename T4, typename T5, typename T6, typename T7>
inline std::string log_message( const char* fmt, const T0& t0, const T1& t1, const T2& t2, const T3& t3, const T4& t4, const T5& t5, const T6& t6, const T7& t7){
  return boost::str(boost::format(fmt) % t0 % t1 % t2 % t3 % t4 % t5 % t6 % t7);
}

template<typename T0,typename T1,typename T2, typename T3, typename T4, typename T5, typename T6, typename T7, typename T8>
inline std::string log_message( const char* fmt, const T0& t0, const T1& t1, const T2& t2, const T3& t3, const T4& t4, const T5& t5, const T6& t6, const T7& t7, const T8& t8){
  return boost::str(boost::format(fmt) % t0 % t1 % t2 % t3 % t4 % t5 % t6 % t7 % t8);
}
template<typename T0,typename T1,typename T2, typename T3, typename T4, typename T5, typename T6, typename T7, typename T8, typename T9>
inline std::string log_message( const char* fmt, const T0& t0, const T1& t1, const T2& t2, const T3& t3, const T4& t4, const T5& t5, const T6& t6, const T7& t7, const T8& t8, const T9& t9){
  return boost::str(boost::format(fmt) % t0 % t1 % t2 % t3 % t4 % t5 % t6 % t7 % t8 % t9);
}

//backends
/**
 * A stdio implementation.  all logging statements are sent to stderr.
 */
struct backend_stdio{
  void operator()(level_t, const std::string& header, const std::string& message) const;
};

/**
 * A syslog implementation, where all messages are sent to syslog, with the correct
 * syslog number
 */
struct backend_syslog{
  void operator()(level_t l, const std::string& header, const std::string& message) const;
};
}

typedef logging::logger<logging::backend_syslog> sys_log;
typedef logging::logger<logging::backend_stdio> stdio_log;
}

#ifdef LOGGING_SHORT_FILENAMES
#include <cstring>
#define FILEMACRO (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#else
#define FILEMACRO __FILE__
#endif

#define IPI_LEVEL_EMERG 0 //this doesn't work: ipi::logging::EMERG //double colons are not allowed
#define IPI_LEVEL_ALERT 1
#define IPI_LEVEL_CRIT 2
#define IPI_LEVEL_ERR 3
#define IPI_LEVEL_WARNING 4
#define IPI_LEVEL_NOTICE 5
#define IPI_LEVEL_INFO 6
#define IPI_LEVEL_DEBUG 7
#define IPI_LEVEL_TRACE 8

#define IPI_FMT(FMT, FMT_ARGS... )              \
  ::ipi::logging::log_message(FMT, ##FMT_ARGS)

#ifndef NDEBUG
//if condition is false, print nice message and abort
#define IPI_ASSERT(CONDITION, FMT, FMT_ARGS...)                         \
  do{                                                                   \
    if(!(CONDITION)){                                                   \
      std::cerr << IPI_FMT("%1%:%2%: %3%\n", FILEMACRO, __LINE__, __PRETTY_FUNCTION__) \
                << IPI_FMT("assertion failed, (%1%) is not true.\n", #CONDITION) \
                << IPI_FMT(FMT, ##FMT_ARGS) << std::endl;               \
      abort();                                                          \
    }                                                                   \
  }while(false)
#else //release mode don't compile assert, as its a condition and may be inside of a loop
#define IPI_ASSERT(CONDITION, FMT, FMT_ARGS...)
#endif

//just abort with a nice message.
//This should always be compiled.. if the code hits this line then
//unconditional badness!
#define IPI_ABORT(FMT, FMT_ARGS...)                                     \
  do{                                                                   \
    std::cerr << IPI_FMT("%1%:%2%: %3% ABORT!\n", FILEMACRO, __LINE__, __PRETTY_FUNCTION__) \
              << IPI_FMT(FMT, ##FMT_ARGS) << std::endl;                 \
    abort();                                                            \
  }while(false)


#define IPI_LOG_HEADER(LEVEL)                                           \
  ::ipi::logging::log_header(                                           \
                             ::ipi::logging::level_str(::ipi::logging::LEVEL), \
                             __PRETTY_FUNCTION__, FILEMACRO, __LINE__)

// low level logging macro
#define IPI_LOG(LOGGER, LEVEL, FMT, FMT_ARGS... )                       \
  do{                                                                   \
    try{                                                                \
      LOGGER& logger_ = LOGGER::instance();                             \
      const ::ipi::logging::level_t level_ = ::ipi::logging::LEVEL;     \
      if(level_ <= logger_.filter()){                                   \
        logger_.flush(level_,                                           \
                      IPI_LOG_HEADER(LEVEL),                            \
                      IPI_FMT(FMT, ##FMT_ARGS));                        \
        ::ipi::logging::log_break();                                    \
      }                                                                 \
    }catch(const std::exception& e){                                    \
      throw ::std::logic_error(IPI_FMT("%s:%d "                         \
                                       "Its likely your formatting"     \
                                       " is foobar. %s",                \
                                       FILEMACRO, __LINE__, e.what()));  \
    }catch(...){IPI_ABORT("An unknown exception was caught inside a logging statement... Fail.");} \
  } while(false)

// set the default logger backend
#ifndef IPI_LOG_BACKEND
#define IPI_LOG_BACKEND ::ipi::stdio_log
#endif

// default compile time threshold is warning
#ifndef IPI_LOG_THRESH
#define IPI_LOG_THRESH IPI_LEVEL_DEBUG
#endif

// set the runtime filter level
#define IPI_LOG_FILTER(LEVEL)                   \
  do{                                           \
    IPI_LOG_BACKEND::instance().filter(LEVEL);  \
  }while(false)


// Logging macros, will do nothing if IPI_LOG_THRESH is
// not satisfied for the given level
#if IPI_LOG_THRESH >= IPI_LEVEL_TRACE
#define IPI_LOG_TRACE(FMT, FMT_ARGS...)                 \
  IPI_LOG(IPI_LOG_BACKEND, TRACE, FMT, ##FMT_ARGS)
#else
#define IPI_LOG_TRACE(FMT, FMT_ARGS...)
#endif

#if IPI_LOG_THRESH >= IPI_LEVEL_DEBUG
#define IPI_LOG_DEBUG(FMT, FMT_ARGS...)                 \
  IPI_LOG(IPI_LOG_BACKEND, DEBUG, FMT, ##FMT_ARGS)
#else
#define IPI_LOG_DEBUG(FMT, FMT_ARGS...)
#endif

#if IPI_LOG_THRESH >= IPI_LEVEL_INFO
#define IPI_LOG_INFO(FMT, FMT_ARGS...)                  \
  IPI_LOG(IPI_LOG_BACKEND, INFO, FMT, ##FMT_ARGS)
#else
#define IPI_LOG_INFO(FMT, FMT_ARGS...)
#endif

#if IPI_LOG_THRESH >= IPI_LEVEL_NOTICE
#define IPI_LOG_NOTICE(FMT, FMT_ARGS...)                \
  IPI_LOG(IPI_LOG_BACKEND, NOTICE, FMT, ##FMT_ARGS)
#else
#define IPI_LOG_NOTICE(FMT, FMT_ARGS...)
#endif

#if IPI_LOG_THRESH >= IPI_LEVEL_WARNING
#define IPI_LOG_WARNING(FMT, FMT_ARGS...)               \
  IPI_LOG(IPI_LOG_BACKEND, WARNING, FMT, ##FMT_ARGS)
#else
#define IPI_LOG_WARNING(FMT, FMT_ARGS...)
#endif

#if IPI_LOG_THRESH >= IPI_LEVEL_ERR
#define IPI_LOG_ERR(FMT, FMT_ARGS...)                   \
  IPI_LOG(IPI_LOG_BACKEND, ERR, FMT, ##FMT_ARGS)
#else
#define IPI_LOG_ERR(FMT, FMT_ARGS...)
#endif

#if IPI_LOG_THRESH >= IPI_LEVEL_CRIT
#define IPI_LOG_CRIT(FMT, FMT_ARGS...)                  \
  IPI_LOG(IPI_LOG_BACKEND, CRIT, FMT, ##FMT_ARGS)
#else
#define IPI_LOG_CRIT(FMT, FMT_ARGS...)
#endif

#if IPI_LOG_THRESH >= IPI_LEVEL_ALERT
#define IPI_LOG_ALERT(FMT, FMT_ARGS...)                 \
  IPI_LOG(IPI_LOG_BACKEND, ALERT, FMT, ##FMT_ARGS)
#else
#define IPI_LOG_ALERT(FMT, FMT_ARGS...)
#endif

#if IPI_LOG_THRESH >= IPI_LEVEL_EMERG
#define IPI_LOG_EMERG(FMT, FMT_ARGS...)                 \
  IPI_LOG(IPI_LOG_BACKEND, EMERG, FMT, ##FMT_ARGS)
#else
#define IPI_LOG_EMERG(FMT, FMT_ARGS...)
#endif

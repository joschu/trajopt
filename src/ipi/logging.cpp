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
#include <iostream>
#include <boost/format.hpp>
#include <syslog.h>
#include "logging.hpp"
#include <cstdlib>
#include <locale>

namespace ipi
{
namespace logging{
struct formatter{
  boost::format fmt_;

  formatter()
  {
    try{
      fmt_ = boost::format(getenv("IPI_LOG_FORMAT") ? getenv("IPI_LOG_FORMAT") : "[%1%] ");
    }catch(const std::exception& e){
      std::cerr << "Could not used IPI_LOG_FORMAT!\n" << fmt_ << "\n" << e.what() << std::endl;
      abort();
    }

    fmt_.exceptions( boost::io::all_error_bits ^ ( boost::io::too_many_args_bit
                                                   | boost::io::too_few_args_bit )  );
  }

  std::string operator()(const char* level,
                         const char* pretty_function,
                         const char* filename,
                         unsigned line)
  {
    return boost::str(fmt_ % level % filename % line % pretty_function);
  }
};

formatter fmtr;

void log_break(){
  //TODO optionally add fuzz here.
}

level_t getenv_filter()
{
  static const std::string lvl = getenv("IPI_LOG_THRESH")
    ? getenv("IPI_LOG_THRESH") : "WARNING";

  if (lvl == "EMERG")    return EMERG;
  if (lvl == "ALERT")    return ALERT;
  if (lvl == "CRIT")     return CRIT;
  if (lvl == "ERR")      return ERR;
  if (lvl == "WARNING")  return WARNING;
  if (lvl == "NOTICE")   return NOTICE;
  if (lvl == "INFO")     return INFO;
  if (lvl == "DEBUG")    return DEBUG;
  if (lvl == "TRACE")    return TRACE;

  std::cerr << "Unknown logging level '" << lvl << "' found in environment in IPI_LOG_THRESH.\n"
            << "Legal values are:\n"
            << "EMERG ALERT CRIT ERR WARNING NOTICE INFO DEBUG TRACE" << std::endl;
  abort();
}

static level_t filter_g_=getenv_filter();

level_t filter()
{
  return filter_g_;
}

void filter(level_t l)
{
  filter_g_ = l;
}

const char* level_str(level_t l)
{
  switch(l){
  case EMERG:   return "EMERG";
  case ALERT:   return "ALERT";
  case CRIT:    return "CRIT";
  case ERR:     return "ERR";
  case WARNING: return "WARNING";
  case NOTICE:  return "NOTICE";
  case INFO:    return "INFO";
  case DEBUG:   return "DEBUG";
  case TRACE:   return "TRACE";
  default:      return "NA";
  }
}

std::string log_header(const char* level, const char* pretty_function, const char* filename, unsigned line)
{
  return fmtr(level, pretty_function, filename, line);
}

std::string log_message(const char* fmt)
{
  return std::string(fmt);
}

void backend_stdio::operator()(level_t, const std::string& header, const std::string& message) const
{
  std::cerr << header << message << std::endl;
}

namespace{
  int tosyslog(level_t l){
    switch(l){
    case EMERG:
      return LOG_EMERG;
    case ALERT:
      return LOG_ALERT;
    case CRIT:
      return LOG_CRIT;
    case ERR:
      return LOG_ERR;
    case WARNING:
      return LOG_WARNING;
    case NOTICE:
      return LOG_NOTICE;
    case INFO:
      return LOG_INFO;
    case DEBUG:
    case TRACE:
      return LOG_DEBUG;
    }
    return LOG_INFO;
  }
}
/**
 * A syslog implementation, where all messages are sent to syslog, with the correct
 * syslog number
 */
void backend_syslog::operator()(level_t l, const std::string& header, const std::string& message) const
{
  //according to man page, never pass the user string directly to the
  //formatting string... use "%s" instead
  syslog(tosyslog(l), "%s%s", header.c_str(), message.c_str());
}

}
}

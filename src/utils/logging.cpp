#include <string>
#include <iostream>
#include "logging.hpp"
#include <cstdlib>
using namespace std;

namespace util {

LogLevel gLogLevel;

int LoggingInit() {
  char* lvlc = getenv("TRAJOPT_LOG_THRESH");
  string lvlstr;
  if (lvlc == NULL) {
    printf("you can set logging level with TRAJOPT_LOG_THRESH. defaulting to INFO\n");
    lvlstr = "INFO";
  }
  else lvlstr = string(lvlc);
  if (lvlstr == "FATAL") gLogLevel = LevelFatal;
  else if (lvlstr == "ERROR") gLogLevel =  LevelError;
  else if (lvlstr == "INFO") gLogLevel = LevelInfo;
  else if (lvlstr == "DEBUG") gLogLevel = LevelDebug;
  else if (lvlstr == "TRACE") gLogLevel = LevelTrace;
  else {
    printf("Invalid value for environment variable TRAJOPT_LOG_THRESH: %s\n", lvlstr.c_str());
    printf("Valid values: FATAL ERROR INFO DEBUG TRACE\n");
    abort();
  }
  return 1;  
}
int this_is_a_hack_but_rhs_executes_on_library_load = LoggingInit();

}


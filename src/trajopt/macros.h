// Generic helper definitions for shared library support
#if defined _WIN32 || defined __CYGWIN__
  #define TRAJOPT_HELPER_DLL_IMPORT __declspec(dllimport)
  #define TRAJOPT_HELPER_DLL_EXPORT __declspec(dllexport)
  #define TRAJOPT_HELPER_DLL_LOCAL
#else
  #if __GNUC__ >= 4
    #define TRAJOPT_HELPER_DLL_IMPORT __attribute__ ((visibility ("default")))
    #define TRAJOPT_HELPER_DLL_EXPORT __attribute__ ((visibility ("default")))
    #define TRAJOPT_HELPER_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
  #else
    #define TRAJOPT_HELPER_DLL_IMPORT
    #define TRAJOPT_HELPER_DLL_EXPORT
    #define TRAJOPT_HELPER_DLL_LOCAL
  #endif
#endif

// Now we use the generic helper definitions above to define TRAJOPT_API and TRAJOPT_LOCAL.
// TRAJOPT_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// TRAJOPT_LOCAL is used for non-api symbols.

#define TRAJOPT_DLL

#ifdef TRAJOPT_DLL // defined if TRAJOPT is compiled as a DLL
  #ifdef TRAJOPT_DLL_EXPORTS // defined if we are building the TRAJOPT DLL (instead of using it)
    #define TRAJOPT_API TRAJOPT_HELPER_DLL_EXPORT
  #else
    #define TRAJOPT_API TRAJOPT_HELPER_DLL_IMPORT
  #endif // TRAJOPT_DLL_EXPORTS
  #define TRAJOPT_LOCAL TRAJOPT_HELPER_DLL_LOCAL
#else // TRAJOPT_DLL is not defined: this means TRAJOPT is a static lib.
  #define TRAJOPT_API
  #define TRAJOPT_LOCAL
#endif // TRAJOPT_DLL

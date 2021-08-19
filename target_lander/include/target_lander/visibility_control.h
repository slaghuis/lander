#ifndef TARGET_LANDER__VISIBILITY_CONTROL_H_
#define TARGET_LANDER__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TARGET_LANDER_EXPORT __attribute__ ((dllexport))
    #define TARGET_LANDER_IMPORT __attribute__ ((dllimport))
  #else
    #define TARGET_LANDER_EXPORT __declspec(dllexport)
    #define TARGET_LANDER_IMPORT __declspec(dllimport)
  #endif
  #ifdef TARGET_LANDER_BUILDING_DLL
    #define TARGET_LANDER_PUBLIC TARGET_LANDER_EXPORT
  #else
    #define TARGET_LANDER_PUBLIC TARGET_LANDER_IMPORT
  #endif
  #define TARGET_LANDER_PUBLIC_TYPE TARGET_LANDER_PUBLIC
  #define TARGET_LANDER_LOCAL
#else
  #define TARGET_LANDER_EXPORT __attribute__ ((visibility("default")))
  #define TARGET_LANDER_IMPORT
  #if __GNUC__ >= 4
    #define TARGET_LANDER_PUBLIC __attribute__ ((visibility("default")))
    #define TARGET_LANDER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TARGET_LANDER_PUBLIC
    #define TARGET_LANDER_LOCAL
  #endif
  #define TARGET_LANDER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // TARGET_LANDER__VISIBILITY_CONTROL_H_

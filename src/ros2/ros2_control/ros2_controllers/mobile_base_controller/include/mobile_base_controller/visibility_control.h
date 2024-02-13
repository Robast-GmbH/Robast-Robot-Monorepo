
#ifndef MOBILE_BASE_CONTROLLER__VISIBILITY_CONTROL_H_
#define MOBILE_BASE_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MOBILE_BASE_CONTROLLER_EXPORT __attribute__((dllexport))
#define MOBILE_BASE_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define MOBILE_BASE_CONTROLLER_EXPORT __declspec(dllexport)
#define MOBILE_BASE_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef MOBILE_BASE_CONTROLLER_BUILDING_DLL
#define MOBILE_BASE_CONTROLLER_PUBLIC MOBILE_BASE_CONTROLLER_EXPORT
#else
#define MOBILE_BASE_CONTROLLER_PUBLIC MOBILE_BASE_CONTROLLER_IMPORT
#endif
#define MOBILE_BASE_CONTROLLER_PUBLIC_TYPE MOBILE_BASE_CONTROLLER_PUBLIC
#define MOBILE_BASE_CONTROLLER_LOCAL
#else
#define MOBILE_BASE_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define MOBILE_BASE_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define MOBILE_BASE_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define MOBILE_BASE_CONTROLLER_LOCAL  __attribute__((visibility("hidden")))
#else
#define MOBILE_BASE_CONTROLLER_PUBLIC
#define MOBILE_BASE_CONTROLLER_LOCAL
#endif
#define MOBILE_BASE_CONTROLLER_PUBLIC_TYPE
#endif

#endif   // MOBILE_BASE_CONTROLLER__VISIBILITY_CONTROL_H_

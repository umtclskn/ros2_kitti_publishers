#ifndef ROS2_KITTI_PUBLISHERS__VISIBILITY_H_
#define ROS2_KITTI_PUBLISHERS__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define KITTI_PUBLISHERS_NODE_EXPORT __attribute__ ((dllexport))
    #define KITTI_PUBLISHERS_NODE_IMPORT __attribute__ ((dllimport))
  #else
    #define KITTI_PUBLISHERS_NODE_EXPORT __declspec(dllexport)
    #define KITTI_PUBLISHERS_NODE_IMPORT __declspec(dllimport)
  #endif

  #ifdef KITTI_PUBLISHERS_NODE_DLL
    #define KITTI_PUBLISHERS_NODE_PUBLIC KITTI_PUBLISHERS_NODE_EXPORT
  #else
    #define KITTI_PUBLISHERS_NODE_PUBLIC KITTI_PUBLISHERS_NODE_IMPORT
  #endif

  #define KITTI_PUBLISHERS_NODE_PUBLIC_TYPE KITTI_PUBLISHERS_NODE_PUBLIC

  #define KITTI_PUBLISHERS_NODE_LOCAL

#else

  #define KITTI_PUBLISHERS_NODE_EXPORT __attribute__ ((visibility("default")))
  #define KITTI_PUBLISHERS_NODE_IMPORT

  #if __GNUC__ >= 4
    #define KITTI_PUBLISHERS_NODE_PUBLIC __attribute__ ((visibility("default")))
    #define KITTI_PUBLISHERS_NODE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KITTI_PUBLISHERS_NODE_PUBLIC
    #define KITTI_PUBLISHERS_NODE_LOCAL
  #endif

  #define KITTI_PUBLISHERS_NODE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROS2_KITTI_PUBLISHERS__VISIBILITY_H_
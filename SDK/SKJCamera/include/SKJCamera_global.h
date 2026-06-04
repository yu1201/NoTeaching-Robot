#ifndef SKJCAMERA_GLOBAL_H
#define SKJCAMERA_GLOBAL_H

/**
 * @file SKJCamera_global.h
 * @brief SDK 导出宏和调用约定定义。
 *
 * 普通业务程序通常不需要直接包含本头文件，包含 skjcamera.h 即可。
 *
 * 构建宏说明：
 * - 构建 DLL 或共享库时定义 SKJCAMERA_LIBRARY 或 SKJCAMERA_BUILD。
 * - 构建或使用静态库时定义 SKJCAMERA_STATIC。
 * - 使用 DLL 或共享库的业务程序通常不需要定义任何宏。
 *
 * 跨语言调用说明：
 * - Windows 下 SKJCAMERA_CALL 固定为 __cdecl，便于 Python ctypes、C# P/Invoke、
 *   Java JNA/JNI 等语言声明相同调用约定。
 * - 非 Windows 平台使用系统默认 C 调用约定。
 */

#if defined(_MSC_VER) || defined(WIN64) || defined(_WIN64) || defined(__WIN64__) || defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#  define SKJCAMERA_CALL __cdecl
#else
#  define SKJCAMERA_CALL
#endif

#if defined(SKJCAMERA_STATIC)
#  define SKJCAMERA_API
#elif defined(_MSC_VER) || defined(WIN64) || defined(_WIN64) || defined(__WIN64__) || defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#  if defined(SKJCAMERA_LIBRARY) || defined(SKJCAMERA_BUILD)
#    define SKJCAMERA_API __declspec(dllexport)
#  else
#    define SKJCAMERA_API __declspec(dllimport)
#  endif
#elif defined(SKJCAMERA_LIBRARY) || defined(SKJCAMERA_BUILD)
#  define SKJCAMERA_API __attribute__((visibility("default")))
#else
#  define SKJCAMERA_API
#endif

#endif // SKJCAMERA_GLOBAL_H

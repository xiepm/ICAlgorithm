#pragma once

#if defined(_WIN32) || defined(_WIN64)
#if defined(HIC_BUILD_SHARED)
#define HIC_EXPORT __declspec(dllexport)
#else
#define HIC_EXPORT __declspec(dllimport)
#endif
#else
#define HIC_EXPORT __attribute__((visibility("default")))
#endif

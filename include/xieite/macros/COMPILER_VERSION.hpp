#pragma once
#include <xieite/macros/COMPILER_TYPE.hpp> // XIEITE_COMPILER_TYPE_GCC, XIEITE_COMPILER_TYPE_CLANG, XIEITE_COMPILER_TYPE_MINGW32, XIEITE_COMPILER_TYPE_MINGW64, XIEITE_COMPILER_TYPE_MSVC

#define XIEITE_COMPILER_VERSION_MAJOR 0
#define XIEITE_COMPILER_VERSION_MINOR 0
#define XIEITE_COMPILER_VERSION_PATCH 0

#if defined(XIEITE_COMPILER_TYPE_GCC)
#	define XIEITE_COMPILER_VERSION_MAJOR __GNUC__
#	define XIEITE_COMPILER_VERSION_MINOR __GNUC_MINOR__
#	define XIEITE_COMPILER_VERSION_PATCH __GNUC_PATCHLEVEL__
#endif

#if defined(XIEITE_COMPILER_TYPE_CLANG)
#	define XIEITE_COMPILER_VERSION_MAJOR __clang_major__
#	define XIEITE_COMPILER_VERSION_MINOR __clang_minor__
#	define XIEITE_COMPILER_VERSION_PATCH __clang_patchlevel__
#endif

#if defined(XIEITE_COMPILER_TYPE_MINGW_32)
#	include <windef.h>
#	define XIEITE_COMPILER_VERSION_MAJOR __MINGW32_MAJOR_VERSION
#	define XIEITE_COMPILER_VERSION_MINOR __MINGW32_MINOR_VERSION
#endif

#if defined(XIEITE_COMPILER_TYPE_MINGW_64)
#	include <windef.h>
#	define XIEITE_COMPILER_VERSION_MAJOR __MINGW64_VERSION_MAJOR
#	define XIEITE_COMPILER_VERSION_MINOR __MINGW64_VERSION_MINOR
#endif

#if defined(XIEITE_COMPILER_TYPE_MSVC)
#	define XIEITE_COMPILER_VERSION_MAJOR (_MSC_FULL_VER / 10000000)
#	define XIEITE_COMPILER_VERSION_MINOR (_MSC_FULL_VER / 100000 - XIEITE_COMPILER_VERSION_MAJOR * 100)
#	define XIEITE_COMPILER_VERSION_PATCH (_MSC_FULL_VER - XIEITE_COMPILER_VERSION_MINOR * 100000 - XIEITE_COMPILER_VERSION_MAJOR * 10000000)
#endif

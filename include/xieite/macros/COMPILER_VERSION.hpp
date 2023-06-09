#ifndef XIEITE_HEADER_MACROS_COMPILER_VERSION
#	define XIEITE_HEADER_MACROS_COMPILER_VERSION

#	include <xieite/macros/COMPILER_TYPE.hpp>

#	define XIEITE_COMPILER_VERSION_MAJOR 0
#	define XIEITE_COMPILER_VERSION_MINOR 0
#	define XIEITE_COMPILER_VERSION_PATCH 0

#	ifdef XIEITE_COMPILER_TYPE_ACPP
#		define XIEITE_COMPILER_VERSION_MAJOR (__HP_aCC / 10000)
#		define XIEITE_COMPILER_VERSION_MINOR (__HP_aCC % 10000 / 100)
#		define XIEITE_COMPILER_VERSION_PATCH (__HP_aCC % 100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_ARM
#		define XIEITE_COMPILER_VERSION_MAJOR (__ARMCC_VERSION / 100000)
#		define XIEITE_COMPILER_VERSION_MINOR (__ARMCC_VERSION % 100000 / 10000)
#		define XIEITE_COMPILER_VERSION_PATCH (__ARMCC_VERSION % 10000 / 1000)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_AZTEC
#		define XIEITE_COMPILER_VERSION_MAJOR (__VERSION / 100)
#		define XIEITE_COMPILER_VERSION_MINOR (__VERSION % 100)
#	endif

#	if defined(XIEITE_COMPILER_TYPE_C_TO_HARDWARE) || defined(XIEITE_COMPILER_TYPE_MICROBLAZE)
#		define XIEITE_COMPILER_VERSION_MAJOR (__VERSION__ / 1000)
#		define XIEITE_COMPILER_VERSION_MINOR (__VERSION__ % 1000)
#		define XIEITE_COMPILER_VERSION_PATCH __REVISION__
#	endif

#	ifdef XIEITE_COMPILER_TYPE_C166
#		define XIEITE_COMPILER_VERSION_MAJOR (__C166__ / 100)
#		define XIEITE_COMPILER_VERSION_MINOR (__C166__ % 100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_C51
#		define XIEITE_COMPILER_VERSION_MAJOR (__C51__ / 100)
#		define XIEITE_COMPILER_VERSION_MINOR (__C51__ % 100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_CARM
#		define XIEITE_COMPILER_VERSION_MAJOR (__CA__ / 100)
#		define XIEITE_COMPILER_VERSION_MINOR (__CA__ % 100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_CC65
#		define XIEITE_COMPILER_VERSION_MAJOR (__CC65__ / 0x100)
#		define XIEITE_COMPILER_VERSION_MINOR (__CC65__ % 0x100 / 0x10)
#		define XIEITE_COMPILER_VERSION_PATCH (__CC65__ % 0x10)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_CLANG
#		define XIEITE_COMPILER_VERSION_MAJOR __clang_major__
#		define XIEITE_COMPILER_VERSION_MINOR __clang_minor__
#		define XIEITE_COMPILER_VERSION_PATCH __clang_patchlevel__
#	endif

#	ifdef XIEITE_COMPILER_TYPE_CODEWARRIOR
#		define XIEITE_COMPILER_VERSION_MAJOR (__MWERKS__ / 0x1000)
#		define XIEITE_COMPILER_VERSION_MINOR (__MWERKS__ % 0x1000 / 0x100)
#		define XIEITE_COMPILER_VERSION_PATCH (__MWERKS__ % 0x100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_COMEAU
#		define XIEITE_COMPILER_VERSION_MAJOR (__COMO_VERSION__ / 100)
#		define XIEITE_COMPILER_VERSION_MINOR (__COMO_VERSION__ % 100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_COMPAQ_C
#		define XIEITE_COMPILER_VERSION_MAJOR (__DECC_VER / 10000000)
#		define XIEITE_COMPILER_VERSION_MINOR (__DECC_VER % 10000000 / 100000)
#		define XIEITE_COMPILER_VERSION_PATCH (__DECC_VER % 10000)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_COMPAQ_CPP
#		define XIEITE_COMPILER_VERSION_MAJOR (__DECCXX_VER / 10000000)
#		define XIEITE_COMPILER_VERSION_MINOR (__DECCXX_VER % 10000000 / 100000)
#		define XIEITE_COMPILER_VERSION_PATCH (__DECCXX_VER % 10000)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_CRAY
#		define XIEITE_COMPILER_VERSION_MAJOR _RELEASE
#		define XIEITE_COMPILER_VERSION_MINOR _RELEASE_MINOR
#	endif

#	ifdef XIEITE_COMPILER_TYPE_DIAB
#		define XIEITE_COMPILER_VERSION_MAJOR (__VERSION_NUMBER__ / 1000)
#		define XIEITE_COMPILER_VERSION_MINOR (__VERSION_NUMBER__ % 1000 / 100)
#		define XIEITE_COMPILER_VERSION_PATCH (__VERSION_NUMBER__ % 100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_DIGITAL_MARS
#		define XIEITE_COMPILER_VERSION_MAJOR (__DMC__ / 0x100)
#		define XIEITE_COMPILER_VERSION_MINOR (__DMC__ % 0x100 / 0x10)
#		define XIEITE_COMPILER_VERSION_PATCH (__DMC__ % 0x10)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_DIGNUS
#		define XIEITE_COMPILER_VERSION_MAJOR (__SYSC_VER__ / 10000)
#		define XIEITE_COMPILER_VERSION_MINOR (__SYSC_VER__ % 10000 / 100)
#		define XIEITE_COMPILER_VERSION_PATCH (__SYSC_VER__ % 100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_DJGPP
#		define XIEITE_COMPILER_VERSION_MAJOR __DJGPP__
#		define XIEITE_COMPILER_VERSION_MINOR __DJGPP_MINOR__
#	endif

#	ifdef XIEITE_COMPILER_TYPE_EDG
#		define XIEITE_COMPILER_VERSION_MAJOR (__EDG_VERSION__ / 100)
#		define XIEITE_COMPILER_VERSION_MINOR (__EDG_VERSION__ % 100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_EKOPATH
#		define XIEITE_COMPILER_VERSION_MAJOR __PATHCC__
#		define XIEITE_COMPILER_VERSION_MINOR __PATHCC_MINOR__
#		define XIEITE_COMPILER_VERSION_PATCH __PATHCC_PATCHLEVEL__
#	endif

#	ifdef XIEITE_COMPILER_TYPE_GCC
#		define XIEITE_COMPILER_VERSION_MAJOR __GNUC__
#		define XIEITE_COMPILER_VERSION_MINOR __GNUC_MINOR__
#		define XIEITE_COMPILER_VERSION_PATCH __GNUC_PATCHLEVEL__
#	endif

#	ifdef XIEITE_COMPILER_TYPE_GREEN_HILLS
#		define XIEITE_COMPILER_VERSION_MAJOR (__GHS_VERSION_NUMBER__ / 100)
#		define XIEITE_COMPILER_VERSION_MINOR (__GHS_VERSION_NUMBER__ % 100 / 10)
#		define XIEITE_COMPILER_VERSION_PATCH (__GHS_VERSION_NUMBER__ % 10)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_IAR
#		define XIEITE_COMPILER_VERSION_MAJOR (__VER__ / 100)
#		define XIEITE_COMPILER_VERSION_MINOR (__VER__ % 100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_INTEL
#		define XIEITE_COMPILER_VERSION_MAJOR (__INTEL_COMPILER / 100)
#		define XIEITE_COMPILER_VERSION_MINOR (__INTEL_COMPILER % 100 / 10)
#		define XIEITE_COMPILER_VERSION_PATCH (__INTEL_COMPILER % 10)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_KAI
#		define XIEITE_COMPILER_VERSION_MAJOR (__KCC_VERSION / 0x1000)
#		define XIEITE_COMPILER_VERSION_MINOR (__KCC_VERSION % 0x1000 / 0x100)
#		define XIEITE_COMPILER_VERSION_PATCH (__KCC_VERSION % 0x100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_MACINTOSH_PROGRAMMERS_WORKSHOP
#		define XIEITE_COMPILER_VERSION_MAJOR (__MRC__ / 0x100)
#		define XIEITE_COMPILER_VERSION_MINOR (__MRC__ % 0x100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_MINGW32
#		include <windef.h>
#		define XIEITE_COMPILER_VERSION_MAJOR __MINGW32_MAJOR_VERSION
#		define XIEITE_COMPILER_VERSION_MINOR __MINGW32_MINOR_VERSION
#	endif

#	ifdef XIEITE_COMPILER_TYPE_MINGW64
#		include <windef.h>
#		define XIEITE_COMPILER_VERSION_MAJOR __MINGW64_VERSION_MAJOR
#		define XIEITE_COMPILER_VERSION_MINOR __MINGW64_VERSION_MINOR
#	endif

#	ifdef XIEITE_COMPILER_TYPE_MIPSPRO
#		define XIEITE_COMPILER_VERSION_MAJOR (_SGI_COMPILER_VERSION / 100)
#		define XIEITE_COMPILER_VERSION_MINOR (_SGI_COMPILER_VERSION % 100 / 10)
#		define XIEITE_COMPILER_VERSION_PATCH (_SGI_COMPILER_VERSION % 10)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_OPEN64
#		define XIEITE_COMPILER_VERSION_MAJOR __OPENCC__
#		define XIEITE_COMPILER_VERSION_MINOR __OPENCC_MINOR__
#	endif

#	ifdef XIEITE_COMPILER_TYPE_PALM
#		define XIEITE_COMPILER_VERSION_MAJOR (_PACC_VER / 0x10000000)
#		define XIEITE_COMPILER_VERSION_MINOR (_PACC_VER % 0x10000000 / 0x100000)
#		define XIEITE_COMPILER_VERSION_PATCH (_PACC_VER % 0x100000 / 0x1000)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_PELLES
#		define XIEITE_COMPILER_VERSION_MAJOR (__POCC__ / 100)
#		define XIEITE_COMPILER_VERSION_MINOR (__POCC__ % 100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_PORTLAND_GROUP
#		define XIEITE_COMPILER_VERSION_MAJOR __PGIC__
#		define XIEITE_COMPILER_VERSION_MINOR __PGIC_MINOR__
#		define XIEITE_COMPILER_VERSION_PATCH __PGIC_PATCHLEVEL__
#	endif

#	ifdef XIEITE_COMPILER_TYPE_RENESAS
#		define XIEITE_COMPILER_VERSION_MAJOR (__RENESAS_VERSION__ / 0x1000000)
#		define XIEITE_COMPILER_VERSION_MINOR (__RENESAS_VERSION__ % 0x1000000 / 0x10000)
#		define XIEITE_COMPILER_VERSION_PATCH (__RENESAS_VERSION__ % 0x10000 / 0x100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_SAS
#		define XIEITE_COMPILER_VERSION_MAJOR (__SASC__ / 100)
#		define XIEITE_COMPILER_VERSION_MINOR (__SASC__ % 100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_SMALL_DEVICE
#		define XIEITE_COMPILER_VERSION_MAJOR (SDCC / 100)
#		define XIEITE_COMPILER_VERSION_MINOR (SDCC % 100 / 10)
#		define XIEITE_COMPILER_VERSION_PATCH (SDCC % 10)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_SOLARIS_STUDIO_C
#		define XIEITE_COMPILER_VERSION_MAJOR (__SUNPRO_C / 0x1000)
#		define XIEITE_COMPILER_VERSION_MINOR (__SUNPRO_C % 0x1000 / 0x10)
#		define XIEITE_COMPILER_VERSION_PATCH (__SUNPRO_C % 0x10)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_SOLARIS_STUDIO_CPP
#		define XIEITE_COMPILER_VERSION_MAJOR (__SUNPRO_CC / 0x1000)
#		define XIEITE_COMPILER_VERSION_MINOR (__SUNPRO_CC % 0x1000 / 0x10)
#		define XIEITE_COMPILER_VERSION_PATCH (__SUNPRO_CC % 0x10)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_SYMANTEC
#		define XIEITE_COMPILER_VERSION_MAJOR (__SC__ / 0x100)
#		define XIEITE_COMPILER_VERSION_MINOR (__SC__ % 0x100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_TMS320
#		define XIEITE_COMPILER_VERSION_MAJOR (__TI_COMPILER_VERSION__ / 1000000)
#		define XIEITE_COMPILER_VERSION_MINOR (__TI_COMPILER_VERSION__ % 1000000 / 1000)
#		define XIEITE_COMPILER_VERSION_PATCH (__TI_COMPILER_VERSION__ % 1000)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_TURBO
#		define XIEITE_COMPILER_VERSION_MAJOR (__TURBOC__ / 0x100)
#		define XIEITE_COMPILER_VERSION_MINOR (__TURBOC__ % 0x100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_ULTIMATE
#		define XIEITE_COMPILER_VERSION_MAJOR _MAJOR_REV
#		define XIEITE_COMPILER_VERSION_MINOR _MINOR_REV
#	endif

#	ifdef XIEITE_COMPILER_TYPE_USL
#		define XIEITE_COMPILER_VERSION_MAJOR (__SCO_VERSION__ / 100000000)
#		define XIEITE_COMPILER_VERSION_MINOR (__SCO_VERSION__ % 100000000 / 1000000)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_VISUAL
#		define XIEITE_COMPILER_VERSION_MAJOR (_MSC_FULL_VER / 10000000)
#		define XIEITE_COMPILER_VERSION_MINOR (_MSC_FULL_VER % 10000000 / 100000)
#		define XIEITE_COMPILER_VERSION_PATCH (_MSC_FULL_VER % 100000)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_WATCOM
#		define XIEITE_COMPILER_VERSION_MAJOR (__WATCOMC__ / 100)
#		define XIEITE_COMPILER_VERSION_MINOR (__WATCOMC__ % 100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_XL_CLANG
#		define XIEITE_COMPILER_VERSION_MAJOR __ibmxl_version__
#		define XIEITE_COMPILER_VERSION_MINOR __ibmxl_release__
#		define XIEITE_COMPILER_VERSION_PATCH __ibmxl_modification__
#	endif

#	ifdef XIEITE_COMPILER_TYPE_XL_LEGACY
#		define XIEITE_COMPILER_VERSION_MAJOR (__xlC__ / 0x100)
#		define XIEITE_COMPILER_VERSION_MINOR (__xlC__ % 0x100)
#	endif

#	if defined(XIEITE_COMPILER_TYPE_Z_OS_XL_C) || defined(XIEITE_COMPILER_TYPE_Z_OS_XL_CPP)
#		define XIEITE_COMPILER_VERSION_MAJOR (__COMPILER_VER__ % 0x10000000 / 0x1000000)
#		define XIEITE_COMPILER_VERSION_MINOR (__COMPILER_VER__ % 0x1000000 / 0x10000)
#		define XIEITE_COMPILER_VERSION_PATCH (__COMPILER_VER__ % 0x10000 / 0x100)
#	endif

#	ifdef XIEITE_COMPILER_TYPE_ZORTECH
#		define XIEITE_COMPILER_VERSION_MAJOR (__ZTC__ / 0x100)
#		define XIEITE_COMPILER_VERSION_MINOR (__ZTC__ % 0x100 / 0x10)
#		define XIEITE_COMPILER_VERSION_PATCH (__ZTC__ % 0x10)
#	endif

#endif

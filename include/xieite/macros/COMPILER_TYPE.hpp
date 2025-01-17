#ifndef XIEITE_HEADER_MACROS_COMPILER_TYPE
#	define XIEITE_HEADER_MACROS_COMPILER_TYPE

#	define XIEITE_COMPILER_TYPE_ACC false
#	define XIEITE_COMPILER_TYPE_ACPP false
#	define XIEITE_COMPILER_TYPE_AMSTERDAM_COMPILER_KIT false
#	define XIEITE_COMPILER_TYPE_ANSI false
#	define XIEITE_COMPILER_TYPE_ARM false
#	define XIEITE_COMPILER_TYPE_AZTEC false
#	define XIEITE_COMPILER_TYPE_BORLAND false
#	define XIEITE_COMPILER_TYPE_C166 false
#	define XIEITE_COMPILER_TYPE_C51 false
#	define XIEITE_COMPILER_TYPE_CARM false
#	define XIEITE_COMPILER_TYPE_CC65 false
#	define XIEITE_COMPILER_TYPE_CLANG false
#	define XIEITE_COMPILER_TYPE_CODEWARRIOR false
#	define XIEITE_COMPILER_TYPE_COMEAU false
#	define XIEITE_COMPILER_TYPE_COMPAQ_C false
#	define XIEITE_COMPILER_TYPE_COMPAQ_CPP false
#	define XIEITE_COMPILER_TYPE_COMPCERT false
#	define XIEITE_COMPILER_TYPE_CONVEX false
#	define XIEITE_COMPILER_TYPE_COVERITY false
#	define XIEITE_COMPILER_TYPE_CRAY false
#	define XIEITE_COMPILER_TYPE_C_TO_HARDWARE false
#	define XIEITE_COMPILER_TYPE_DIAB false
#	define XIEITE_COMPILER_TYPE_DICE false
#	define XIEITE_COMPILER_TYPE_DIGITAL_MARS false
#	define XIEITE_COMPILER_TYPE_DIGNUS false
#	define XIEITE_COMPILER_TYPE_DJGPP false
#	define XIEITE_COMPILER_TYPE_EDG false
#	define XIEITE_COMPILER_TYPE_EKOPATH false
#	define XIEITE_COMPILER_TYPE_FUJITSU false
#	define XIEITE_COMPILER_TYPE_GCC false
#	define XIEITE_COMPILER_TYPE_GREEN_HILLS false
#	define XIEITE_COMPILER_TYPE_HIGH false
#	define XIEITE_COMPILER_TYPE_IAR false
#	define XIEITE_COMPILER_TYPE_IMAGECRAFT false
#	define XIEITE_COMPILER_TYPE_INTEL false
#	define XIEITE_COMPILER_TYPE_KAI false
#	define XIEITE_COMPILER_TYPE_LCC false
#	define XIEITE_COMPILER_TYPE_LLVM false
#	define XIEITE_COMPILER_TYPE_MACINTOSH_PROGRAMMERS_WORKSHOP false
#	define XIEITE_COMPILER_TYPE_MICROBLAZE false
#	define XIEITE_COMPILER_TYPE_MICROTEC false
#	define XIEITE_COMPILER_TYPE_MIPSPRO false
#	define XIEITE_COMPILER_TYPE_MIRACLE false
#	define XIEITE_COMPILER_TYPE_MSVC false
#	define XIEITE_COMPILER_TYPE_NDP false
#	define XIEITE_COMPILER_TYPE_NORCROFT false
#	define XIEITE_COMPILER_TYPE_NWCC false
#	define XIEITE_COMPILER_TYPE_OPEN64 false
#	define XIEITE_COMPILER_TYPE_OPENSERVER false
#	define XIEITE_COMPILER_TYPE_PACIFIC false
#	define XIEITE_COMPILER_TYPE_PALM false
#	define XIEITE_COMPILER_TYPE_PELLES false
#	define XIEITE_COMPILER_TYPE_PORTLAND_GROUP false
#	define XIEITE_COMPILER_TYPE_PRO false
#	define XIEITE_COMPILER_TYPE_RENESAS false
#	define XIEITE_COMPILER_TYPE_SAS false
#	define XIEITE_COMPILER_TYPE_SMALL_DEVICE false
#	define XIEITE_COMPILER_TYPE_SN false
#	define XIEITE_COMPILER_TYPE_SOLARIS_STUDIO_C false
#	define XIEITE_COMPILER_TYPE_SOLARIS_STUDIO_CPP false
#	define XIEITE_COMPILER_TYPE_SYMANTEC false
#	define XIEITE_COMPILER_TYPE_TENDRA false
#	define XIEITE_COMPILER_TYPE_THINK false
#	define XIEITE_COMPILER_TYPE_TINY false
#	define XIEITE_COMPILER_TYPE_TMS320 false
#	define XIEITE_COMPILER_TYPE_TURBO false
#	define XIEITE_COMPILER_TYPE_ULTIMATE false
#	define XIEITE_COMPILER_TYPE_USL false
#	define XIEITE_COMPILER_TYPE_VBCC false
#	define XIEITE_COMPILER_TYPE_VOS false
#	define XIEITE_COMPILER_TYPE_WATCOM false
#	define XIEITE_COMPILER_TYPE_WINGW32 false
#	define XIEITE_COMPILER_TYPE_WINGW64 false
#	define XIEITE_COMPILER_TYPE_XL_CLANG false
#	define XIEITE_COMPILER_TYPE_XL_LEGACY false
#	define XIEITE_COMPILER_TYPE_ZORTECH false
#	define XIEITE_COMPILER_TYPE_Z_OS_XL_C false
#	define XIEITE_COMPILER_TYPE_Z_OS_XL_CPP false

#	ifdef _ACC_
#		undef XIEITE_COMPILER_TYPE_ACC
#		define XIEITE_COMPILER_TYPE_ACC true
#	endif

#	ifdef __HP_aCC
#		undef XIEITE_COMPILER_TYPE_ACPP
#		define XIEITE_COMPILER_TYPE_ACPP true
#	endif

#	ifdef __ACK__
#		undef XIEITE_COMPILER_TYPE_AMSTERDAM_COMPILER_KIT
#		define XIEITE_COMPILER_TYPE_AMSTERDAM_COMPILER_KIT true
#	endif

#	ifdef __HP_cc
#		undef XIEITE_COMPILER_TYPE_ANSI
#		define XIEITE_COMPILER_TYPE_ANSI true
#	endif

#	ifdef __CC_ARM
#		undef XIEITE_COMPILER_TYPE_ARM
#		define XIEITE_COMPILER_TYPE_ARM true
#	endif

#	if defined(AZTEC_C) || defined(__AZTEC_C__)
#		undef XIEITE_COMPILER_TYPE_AZTEC
#		define XIEITE_COMPILER_TYPE_AZTEC true
#	endif

#	if defined(__BORLANDC__) || defined(__CODEGEARC__)
#		undef XIEITE_COMPILER_TYPE_BORLAND
#		define XIEITE_COMPILER_TYPE_BORLAND true
#	endif

#	ifdef __C166__
#		undef XIEITE_COMPILER_TYPE_C166
#		define XIEITE_COMPILER_TYPE_C166 true
#	endif

#	if defined(__C51__) || defined(__CX51__)
#		undef XIEITE_COMPILER_TYPE_C51
#		define XIEITE_COMPILER_TYPE_C51 true
#	endif

#	if defined(__CA__) || defined(__KEIL__)
#		undef XIEITE_COMPILER_TYPE_CARM
#		define XIEITE_COMPILER_TYPE_CARM true
#	endif

#	ifdef __CC65__
#		undef XIEITE_COMPILER_TYPE_CC65
#		define XIEITE_COMPILER_TYPE_CC65 true
#	endif

#	ifdef __clang__
#		undef XIEITE_COMPILER_TYPE_CLANG
#		define XIEITE_COMPILER_TYPE_CLANG true
#	endif

#	if defined(__MWERKS__) || defined(__CWCC__)
#		undef XIEITE_COMPILER_TYPE_CODEWARRIOR
#		define XIEITE_COMPILER_TYPE_CODEWARRIOR true
#	endif

#	ifdef __COMO__
#		undef XIEITE_COMPILER_TYPE_COMEAU
#		define XIEITE_COMPILER_TYPE_COMEAU true
#	endif

#	ifdef __DECC
#		undef XIEITE_COMPILER_TYPE_COMPAQ_C
#		define XIEITE_COMPILER_TYPE_COMPAQ_C true
#	endif

#	ifdef __DECCXX
#		undef XIEITE_COMPILER_TYPE_COMPAQ_CPP
#		define XIEITE_COMPILER_TYPE_COMPAQ_CPP true
#	endif

#	ifdef __COMPCERT__
#		undef XIEITE_COMPILER_TYPE_COMPCERT
#		define XIEITE_COMPILER_TYPE_COMPCERT true
#	endif

#	ifdef __convexc__
#		undef XIEITE_COMPILER_TYPE_CONVEX
#		define XIEITE_COMPILER_TYPE_CONVEX true
#	endif

#	ifdef __COVERITY__
#		undef XIEITE_COMPILER_TYPE_COVERITY
#		define XIEITE_COMPILER_TYPE_COVERITY true
#	endif

#	ifdef _CRAYC
#		undef XIEITE_COMPILER_TYPE_CRAY
#		define XIEITE_COMPILER_TYPE_CRAY true
#	endif

#	ifdef __CHC__
#		undef XIEITE_COMPILER_TYPE_C_TO_HARDWARE
#		define XIEITE_COMPILER_TYPE_C_TO_HARDWARE true
#	endif

#	ifdef __DCC__
#		undef XIEITE_COMPILER_TYPE_DIAB
#		define XIEITE_COMPILER_TYPE_DIAB true
#	endif

#	ifdef _DICE
#		undef XIEITE_COMPILER_TYPE_DICE
#		define XIEITE_COMPILER_TYPE_DICE true
#	endif

#	ifdef __DMC__
#		undef XIEITE_COMPILER_TYPE_DIGITAL_MARS
#		define XIEITE_COMPILER_TYPE_DIGITAL_MARS true
#	endif

#	ifdef __SYSC__
#		undef XIEITE_COMPILER_TYPE_DIGNUS
#		define XIEITE_COMPILER_TYPE_DIGNUS true
#	endif

#	if defined(__DJGPP__) || defined(__GO32__)
#		undef XIEITE_COMPILER_TYPE_DJGPP
#		define XIEITE_COMPILER_TYPE_DJGPP true
#	endif

#	ifdef __EDG__
#		undef XIEITE_COMPILER_TYPE_EDG
#		define XIEITE_COMPILER_TYPE_EDG true
#	endif

#	ifdef __PATHCC__
#		undef XIEITE_COMPILER_TYPE_EKOPATH
#		define XIEITE_COMPILER_TYPE_EKOPATH true
#	endif

#	ifdef __FCC_VERSION
#		undef XIEITE_COMPILER_TYPE_FUJITSU
#		define XIEITE_COMPILER_TYPE_FUJITSU true
#	endif

#	ifdef __GNUC__
#		undef XIEITE_COMPILER_TYPE_GCC
#		define XIEITE_COMPILER_TYPE_GCC true
#	endif

#	ifdef __ghs__
#		undef XIEITE_COMPILER_TYPE_GREEN_HILLS
#		define XIEITE_COMPILER_TYPE_GREEN_HILLS true
#	endif

#	ifdef __HIGHC__
#		undef XIEITE_COMPILER_TYPE_HIGH
#		define XIEITE_COMPILER_TYPE_HIGH true
#	endif

#	ifdef __IAR_SYSTEMS_ICC__
#		undef XIEITE_COMPILER_TYPE_IAR
#		define XIEITE_COMPILER_TYPE_IAR true
#	endif

#	ifdef __IMAGECRAFT__
#		undef XIEITE_COMPILER_TYPE_IMAGECRAFT
#		define XIEITE_COMPILER_TYPE_IMAGECRAFT true
#	endif

#	if defined(__INTEL_COMPILER) || defined(__ICL)
#		undef XIEITE_COMPILER_TYPE_INTEL
#		define XIEITE_COMPILER_TYPE_INTEL true
#	endif

#	ifdef __KCC
#		undef XIEITE_COMPILER_TYPE_KAI
#		define XIEITE_COMPILER_TYPE_KAI true
#	endif

#	ifdef __LCC__
#		undef XIEITE_COMPILER_TYPE_LCC
#		define XIEITE_COMPILER_TYPE_LCC true
#	endif

#	ifdef __llvm__
#		undef XIEITE_COMPILER_TYPE_LLVM
#		define XIEITE_COMPILER_TYPE_LLVM true
#	endif

#	if defined(__MRC__) || defined(MPW_C) || defined(MPW_CPLUS)
#		undef XIEITE_COMPILER_TYPE_MACINTOSH_PROGRAMMERS_WORKSHOP
#		define XIEITE_COMPILER_TYPE_MACINTOSH_PROGRAMMERS_WORKSHOP true
#	endif

#	ifdef __CMB__
#		undef XIEITE_COMPILER_TYPE_MICROBLAZE
#		define XIEITE_COMPILER_TYPE_MICROBLAZE true
#	endif

#	ifdef _MRI
#		undef XIEITE_COMPILER_TYPE_MICROTEC
#		define XIEITE_COMPILER_TYPE_MICROTEC true
#	endif

#	if defined(__sgi) || defined(sgi)
#		undef XIEITE_COMPILER_TYPE_MIPSPRO
#		define XIEITE_COMPILER_TYPE_MIPSPRO true
#	endif

#	ifdef MIRACLE
#		undef XIEITE_COMPILER_TYPE_MIRACLE
#		define XIEITE_COMPILER_TYPE_MIRACLE true
#	endif

#	ifdef _MSC_VER
#		undef XIEITE_COMPILER_TYPE_MSVC
#		define XIEITE_COMPILER_TYPE_MSVC true
#	endif

#	if defined(__NDPC__) || defined(__NDPX__)
#		undef XIEITE_COMPILER_TYPE_NDP
#		define XIEITE_COMPILER_TYPE_NDP true
#	endif

#	ifdef __CC_NORCROFT
#		undef XIEITE_COMPILER_TYPE_NORCROFT
#		define XIEITE_COMPILER_TYPE_NORCROFT true
#	endif

#	ifdef __NWCC__
#		undef XIEITE_COMPILER_TYPE_NWCC
#		define XIEITE_COMPILER_TYPE_NWCC true
#	endif

#	if defined(__OPEN64__) || defined(__OPENCC__)
#		undef XIEITE_COMPILER_TYPE_OPEN64
#		define XIEITE_COMPILER_TYPE_OPEN64 true
#	endif

#	ifdef _SCO_DS
#		undef XIEITE_COMPILER_TYPE_OPENSERVER
#		define XIEITE_COMPILER_TYPE_OPENSERVER true
#	endif

#	ifdef __PACIFIC__
#		undef XIEITE_COMPILER_TYPE_PACIFIC
#		define XIEITE_COMPILER_TYPE_PACIFIC true
#	endif

#	ifdef _PACC_VER
#		undef XIEITE_COMPILER_TYPE_PALM
#		define XIEITE_COMPILER_TYPE_PALM true
#	endif

#	ifdef __POCC__
#		undef XIEITE_COMPILER_TYPE_PELLES
#		define XIEITE_COMPILER_TYPE_PELLES true
#	endif

#	ifdef __PGI
#		undef XIEITE_COMPILER_TYPE_PORTLAND_GROUP
#		define XIEITE_COMPILER_TYPE_PORTLAND_GROUP true
#	endif

#	ifdef ORA_PROC
#		undef XIEITE_COMPILER_TYPE_PRO
#		define XIEITE_COMPILER_TYPE_PRO true
#	endif

#	if defined(__RENESAS__) || defined(__HITACHI__)
#		undef XIEITE_COMPILER_TYPE_RENESAS
#		define XIEITE_COMPILER_TYPE_RENESAS true
#	endif

#	if defined(SASC) || defined(__SASC) || defined(__SASC__)
#		undef XIEITE_COMPILER_TYPE_SAS
#		define XIEITE_COMPILER_TYPE_SAS true
#	endif

#	ifdef SDCC
#		undef XIEITE_COMPILER_TYPE_SMALL_DEVICE
#		define XIEITE_COMPILER_TYPE_SMALL_DEVICE true
#	endif

#	ifdef __SNC__
#		undef XIEITE_COMPILER_TYPE_SN
#		define XIEITE_COMPILER_TYPE_SN true
#	endif

#	ifdef __SUNPRO_C
#		undef XIEITE_COMPILER_TYPE_SOLARIS_STUDIO_C
#		define XIEITE_COMPILER_TYPE_SOLARIS_STUDIO_C true
#	endif

#	ifdef __SUNPRO_CC
#		undef XIEITE_COMPILER_TYPE_SOLARIS_STUDIO_CPP
#		define XIEITE_COMPILER_TYPE_SOLARIS_STUDIO_CPP true
#	endif

#	ifdef __SC__
#		undef XIEITE_COMPILER_TYPE_SYMANTEC
#		define XIEITE_COMPILER_TYPE_SYMANTEC true
#	endif

#	ifdef __TenDRA__
#		undef XIEITE_COMPILER_TYPE_TENDRA
#		define XIEITE_COMPILER_TYPE_TENDRA true
#	endif

#	if defined(THINKC3) || defined(THINKC4)
#		undef XIEITE_COMPILER_TYPE_THINK
#		define XIEITE_COMPILER_TYPE_THINK true
#	endif

#	ifdef __TINYC__
#		undef XIEITE_COMPILER_TYPE_TINY
#		define XIEITE_COMPILER_TYPE_TINY true
#	endif

#	if defined(__TI_COMPILER_VERION__) || defined(_TMS320C6X)
#		undef XIEITE_COMPILER_TYPE_TMS320
#		define XIEITE_COMPILER_TYPE_TMS320 true
#	endif

#	ifdef __TURBOC__
#		undef XIEITE_COMPILER_TYPE_TURBO
#		define XIEITE_COMPILER_TYPE_TURBO true
#	endif

#	ifdef _UCC
#		undef XIEITE_COMPILER_TYPE_ULTIMATE
#		define XIEITE_COMPILER_TYPE_ULTIMATE true
#	endif

#	ifdef __USLC__
#		undef XIEITE_COMPILER_TYPE_USL
#		define XIEITE_COMPILER_TYPE_USL true
#	endif

#	ifdef __VBCC__
#		undef XIEITE_COMPILER_TYPE_VBCC
#		define XIEITE_COMPILER_TYPE_VBCC true
#	endif

#	ifdef __VOSC__
#		undef XIEITE_COMPILER_TYPE_VOS
#		define XIEITE_COMPILER_TYPE_VOS true
#	endif

#	ifdef __WATCOMC__
#		undef XIEITE_COMPILER_TYPE_WATCOM
#		define XIEITE_COMPILER_TYPE_WATCOM true
#	endif

#	ifdef __MINGW32__
#		undef XIEITE_COMPILER_TYPE_WINGW32
#		define XIEITE_COMPILER_TYPE_WINGW32 true
#	endif

#	ifdef __MINGW64__
#		undef XIEITE_COMPILER_TYPE_WINGW64
#		define XIEITE_COMPILER_TYPE_WINGW64 true
#	endif

#	ifdef __ibmxl__
#		undef XIEITE_COMPILER_TYPE_XL_CLANG
#		define XIEITE_COMPILER_TYPE_XL_CLANG true
#	endif

#	ifdef __xlC__
#		undef XIEITE_COMPILER_TYPE_XL_LEGACY
#		define XIEITE_COMPILER_TYPE_XL_LEGACY true
#	endif

#	ifdef __ZTC__
#		undef XIEITE_COMPILER_TYPE_ZORTECH
#		define XIEITE_COMPILER_TYPE_ZORTECH true
#	endif

#	ifdef __IBMC__
#		undef XIEITE_COMPILER_TYPE_Z_OS_XL_C
#		define XIEITE_COMPILER_TYPE_Z_OS_XL_C true
#	endif

#	ifdef __IBMCPP__
#		undef XIEITE_COMPILER_TYPE_Z_OS_XL_CPP
#		define XIEITE_COMPILER_TYPE_Z_OS_XL_CPP true
#	endif

#endif

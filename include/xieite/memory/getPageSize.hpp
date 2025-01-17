#ifndef XIETIE_HEADER_MEMORY_GETPAGESIZE
#	define XIETIE_HEADER_MEMORY_GETPAGESIZE

#	include "../macros/SYSTEM_TYPE.hpp"

#	if XIEITE_SYSTEM_TYPE_UNIX
#		include <cstddef>
#		include <unistd.h>

namespace xieite::memory {
	[[nodiscard]]
	inline std::size_t getPageSize() noexcept {
		return ::sysconf(_SC_PAGE_SIZE);
	}
}

#	elif XIEITE_SYSTEM_TYPE_WINDOWS
#		include <cstddef>
#		include <windows.h>

namespace xieite::memory {
	[[nodiscard]]
	inline std::size_t getPageSize() noexcept {
		::SYSTEM_INFO info;
		::GetSystemInfo(&info);
		return info.dwPageSize;
	}
}

#	else
#		error "System not supported"
#	endif

#endif

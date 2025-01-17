#ifndef XIEITE_HEADER_SYSTEM_BITSPERBYTE
#	define XIEITE_HEADER_SYSTEM_BITSPERBYTE

#	include <cstddef>
#	include <limits>

namespace xieite::system {
	inline constexpr std::size_t bitsPerByte = std::numeric_limits<unsigned char>::digits;
}

// After a long and very intense debate between FOUR different wizards on several topics of varying relativity, it was deemed that `std::numeric_limits<unsigned char>::digits` and `CHAR_BIT` both are terrible options and there's no good way of doing anything at all

#endif

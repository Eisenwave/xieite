#ifndef XIEITE_HEADER_STRINGS_TRIMFRONT
#	define XIEITE_HEADER_STRINGS_TRIMFRONT

#	include <array>
#	include <cstddef>
#	include <limits>
#	include <string>
#	include "../concepts/RangeOf.hpp"

namespace xieite::strings {
	[[nodiscard]]
	constexpr std::string trimFront(const std::string& string, const char character) noexcept {
		const std::size_t stringSize = string.size();
		for (std::size_t i = 0; i < stringSize; ++i) {
			if (string[i] != character) {
				return string.substr(i);
			}
		}
		return "";
	}

	template<xieite::concepts::RangeOf<char> CharacterRange>
	[[nodiscard]]
	constexpr std::string trimFront(const std::string& string, const CharacterRange& characters) noexcept {
		std::array<bool, std::numeric_limits<char>::max() - std::numeric_limits<char>::min() + 1> characterMap;
		for (const char character : characters) {
			characterMap[character - std::numeric_limits<char>::min()] = true;
		}
		const std::size_t stringSize = string.size();
		for (std::size_t i = 0; i < stringSize; ++i) {
			if (!characterMap[string[i]]) {
				return string.substr(i);
			}
		}
		return "";
	}
}

#endif

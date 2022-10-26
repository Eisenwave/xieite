#pragma once
#include <concepts>
#include <cstddef>
#include <string>
#include <string_view>
#include <type_traits>

namespace xieite::math {
	template<std::integral N = int>
	[[nodiscard]]
	constexpr std::string toBase(N value, const unsigned int base, const std::string_view digits = "0123456789abcdefghijklmnopqrstuvwxyz") noexcept {
		std::make_unsigned_t<N> abs = static_cast<std::make_unsigned_t<N>>(value);
		std::string result;
		do {
			result = digits[abs % base] + result;
			abs /= base;
		} while (abs);
		return result;
	}
}
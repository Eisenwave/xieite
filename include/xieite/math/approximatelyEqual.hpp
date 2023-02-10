#pragma once
#include <cmath> // std::abs
#include <limits> // std::numeric_limits
#include <xieite/concepts/Arithmetic.hpp>

namespace xieite::math {
	template<xieite::concepts::Arithmetic N>
	[[nodiscard]]
	constexpr bool approximatelyEqual(const N value1, const N value2) noexcept {
		return std::abs(value1 - value2) <= std::numeric_limits<N>::epsilon();
	}
}
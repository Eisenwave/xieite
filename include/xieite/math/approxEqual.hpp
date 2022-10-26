#pragma once
#include <cmath>
#include <xieite/concepts/Arithmetic.hpp>
#include <limits>

namespace xieite::math {
	template<xieite::concepts::Arithmetic N>
	[[nodiscard]]
	constexpr bool approxEqual(const N value1, const N value2) noexcept {
		return std::fabs(value1 - value2) <= std::numeric_limits<N>::epsilon();
	}
}
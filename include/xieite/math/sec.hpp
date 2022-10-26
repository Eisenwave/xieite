#pragma once
#include <cmath>
#include <concepts>

namespace xieite::math {
	template<std::floating_point N>
	[[nodiscard]]
	constexpr N sec(const N value) noexcept {
		return 1 / std::cos(value);
	}
}
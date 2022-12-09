#pragma once
#include <xieite/concepts/Arithmetic.hpp> // xieite::concepts::Arithmetic
#include <xieite/math/difference.hpp> // xieite::math::difference

namespace xieite::math {
	template<xieite::concepts::Arithmetic N>
	[[nodiscard]]
	constexpr N& closestTo(const N target, N& a, N& b) noexcept {
		return (a >= target)
			? (b >= target)
				? (a < b)
					? a
					: b
				: (xieite::math::difference(target, a) > xieite::math::difference(target, b))
					? a
					: b
			: (b < target)
				? (a > b)
					? a
					: b
				: (xieite::math::difference(target, a) < xieite::math::difference(target, b))
					? a
					: b;
	}

	template<xieite::concepts::Arithmetic N>
	[[nodiscard]]
	constexpr const N& closestTo(const N target, const N& a, const N& b) noexcept {
		return (a >= target)
			? (b >= target)
				? (a < b)
					? a
					: b
				: (xieite::math::difference(target, a) > xieite::math::difference(target, b))
					? a
					: b
			: (b < target)
				? (a > b)
					? a
					: b
				: (xieite::math::difference(target, a) < xieite::math::difference(target, b))
					? a
					: b;
	}
}

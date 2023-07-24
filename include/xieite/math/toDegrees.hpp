#ifndef XIEITE_HEADER_MATH_TODEGREES
#	define XIEITE_HEADER_MATH_TODEGREES

#	include <xieite/concepts/Arithmetic.hpp>
#	include <xieite/math/Result.hpp>
#	include <xieite/math/pi.hpp>

namespace xieite::math {
	template<xieite::concepts::Arithmetic Arithmetic>
	[[nodiscard]]
	constexpr xieite::math::Result<Arithmetic> toDegrees(const Arithmetic radians) noexcept {
		return radians * 180.0 / xieite::math::pi<Arithmetic>;
	}
}

#endif

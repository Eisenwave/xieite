#pragma once
#include <concepts> // std::derived_from
#include <cmath> // std::atan2, std::fmod
#include <xieite/geometry/AbstractLine.hpp>
#include <xieite/geometry/Point.hpp>
#include <xieite/math/pi.hpp>
#include <xieite/math/tau.hpp>

namespace xieite::geometry {
	[[nodiscard]]
	constexpr long double getAngle(const xieite::geometry::Point point1, const xieite::geometry::Point point2) noexcept {
		return std::fmod(std::atan2(point1.y - point2.y, point1.x - point2.x) + xieite::math::tau<long double>, xieite::math::pi<long double>);
	}

	template<std::derived_from<xieite::geometry::AbstractLine> LineLike>
	[[nodiscard]]
	constexpr long double getAngle(const LineLike& lineLike) noexcept {
		return xieite::geometry::getAngle(lineLike.start, lineLike.end);
	}
}
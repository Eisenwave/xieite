#ifndef XIEITE_HEADER_GEOMETRY_GETROTATED
#	define XIEITE_HEADER_GEOMETRY_GETROTATED

#	include <concepts>
#	include <cmath>
#	include <vector>
#	include <xieite/concepts/LinearShape.hpp>
#	include <xieite/geometry/Point.hpp>
#	include <xieite/geometry/Polygon.hpp>
#	include <xieite/math/tau.hpp>

namespace xieite::geometry {
	[[nodiscard]]
	constexpr xieite::geometry::Point getRotated(const xieite::geometry::Point point, const double angle, const xieite::geometry::Point pivot = xieite::geometry::Point(0, 0)) noexcept {
		return xieite::geometry::Point(pivot.x + std::cos(angle) * (point.x - pivot.x) - std::sin(angle) * (point.y - pivot.y), pivot.y + std::cos(angle) * (point.y - pivot.y) + std::sin(angle) * (point.x - pivot.x));
	}

	template<xieite::concepts::LinearShape LinearShape>
	[[nodiscard]]
	constexpr LinearShape getRotated(const LinearShape& linearShape, const double angle, const xieite::geometry::Point pivot = xieite::geometry::Point(0, 0)) noexcept {
		return LinearShape(xieite::geometry::getRotated(linearShape.start, angle, pivot), xieite::geometry::getRotated(linearShape.end, angle, pivot));
	}

	[[nodiscard]]
	constexpr xieite::geometry::Polygon getRotated(const xieite::geometry::Polygon& polygon, const double angle, const xieite::geometry::Point pivot = xieite::geometry::Point(0, 0)) noexcept {
		std::vector<xieite::geometry::Point> rotatedPoints;
		for (const xieite::geometry::Point point : polygon.points) {
			rotatedPoints.push_back(xieite::geometry::getRotated(point, angle, pivot));
		}
		return xieite::geometry::Polygon(rotatedPoints);
	}
}

#endif
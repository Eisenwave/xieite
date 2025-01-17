#ifndef XIEITE_HEADER_GEOMETRY_LINE
#	define XIEITE_HEADER_GEOMETRY_LINE

#	include <cmath>
#	include <limits>
#	include "../geometry/Point.hpp"
#	include "../math/almostEqual.hpp"
#	include "../math/almostEqualSlope.hpp"

namespace xieite::geometry {
	struct Line {
		xieite::geometry::Point start;
		xieite::geometry::Point end;

		constexpr Line(const xieite::geometry::Point start, const xieite::geometry::Point end) noexcept
		: start(start), end(end) {}

		constexpr Line(const xieite::geometry::Point start, const double angle) noexcept
		: start(start), end(std::cos(angle), std::sin(angle)) {}

		[[nodiscard]]
		constexpr bool operator==(const xieite::geometry::Line& line) const noexcept {
			const double slope = (this->start.x == this->end.x) ? std::numeric_limits<double>::infinity() : ((this->end.y - this->start.y) / (this->end.x - this->start.x));
			return (std::isinf(slope) ? xieite::math::almostEqual(this->start.x, line.start.x) : xieite::math::almostEqual(line.start.x * slope - this->start.x * slope + this->start.y, line.start.y)) && xieite::math::almostEqualSlope(slope, (line.start.x == line.end.x) ? std::numeric_limits<double>::infinity() : ((line.end.y - line.start.y) / (line.end.x - line.start.x)));
		}
	};
}

#endif

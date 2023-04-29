#pragma once

#include <cmath>
#include <vector>
#include <xieite/concepts/LinearShape.hpp>
#include <xieite/geometry/Ellipse.hpp>
#include <xieite/geometry/Point.hpp>
#include <xieite/geometry/Segment.hpp>
#include <xieite/geometry/containsPoint.hpp>
#include <xieite/geometry/getBoundingBox.hpp>
#include <xieite/geometry/getDistance.hpp>
#include <xieite/geometry/getSides.hpp>
#include <xieite/geometry/rotate.hpp>
#include <xieite/math/approximatelyEqual.hpp>

#include <iostream>
#include <thread>

namespace xieite::geometry {
	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Point point1, const xieite::geometry::Point point2) noexcept {
		std::vector<xieite::geometry::Point> intersections;
		if (point1 == point2) {
			intersections.push_back(point1);
		}
		return intersections;
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Point point, const xieite::concepts::LinearShape auto& linearShape) noexcept {
		std::vector<xieite::geometry::Point> intersections;
		if (xieite::geometry::containsPoint(linearShape, point)) {
			intersections.push_back(point);
		}
		return intersections;
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Point point, const xieite::geometry::Polygon& polygon) noexcept {
		std::vector<xieite::geometry::Point> intersections1;
		for (const xieite::geometry::Segment& side : xieite::geometry::getSides(polygon)) {
			const std::vector<xieite::geometry::Point> intersections2 = xieite::geometry::getIntersections(point, side);
			if (intersections2.size()) {
				intersections1.push_back(intersections2[0]);
			}
		}
		return intersections1;
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Point point, const xieite::geometry::Circle& circle) noexcept {
		std::vector<xieite::geometry::Point> intersections;
		if (xieite::math::approximatelyEqual(xieite::geometry::getDistance(point, circle.center), circle.radius)) {
			intersections.push_back(point);
		}
		return intersections;
	}
	
	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::concepts::LinearShape auto& linearShape, const xieite::geometry::Point point) noexcept {
		return xieite::geometry::getIntersections(point, linearShape);
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::concepts::LinearShape auto& linearShape1, const xieite::concepts::LinearShape auto& linearShape2) noexcept {
		std::vector<xieite::geometry::Point> intersections;
		const double a = (linearShape1.start.x - linearShape1.end.x) * (linearShape2.start.y - linearShape2.end.y) - (linearShape1.start.y - linearShape1.end.y) * (linearShape2.start.x - linearShape2.end.x);
		if (!xieite::math::approximatelyEqual(a, 0.0)) {
			const xieite::geometry::Point intersection(((linearShape2.start.x - linearShape2.end.x) * (linearShape1.start.x * linearShape1.end.y - linearShape1.start.y * linearShape1.end.x) - (linearShape1.start.x - linearShape1.end.x) * (linearShape2.start.x * linearShape2.end.y - linearShape2.start.y * linearShape2.end.x)) / a, ((linearShape2.start.y - linearShape2.end.y) * (linearShape1.start.x * linearShape1.end.y - linearShape1.start.y * linearShape1.end.x) - (linearShape1.start.y - linearShape1.end.y) * (linearShape2.start.x * linearShape2.end.y - linearShape2.start.y * linearShape2.end.x)) / a);
			if (xieite::geometry::containsPoint(linearShape1, intersection) && xieite::geometry::containsPoint(linearShape2, intersection)) {
				intersections.push_back(intersection);
			}
		}
		return intersections;
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::concepts::LinearShape auto& linearShape, const xieite::geometry::Polygon& polygon) noexcept {
		std::vector<xieite::geometry::Point> intersections1;
		for (const xieite::geometry::Segment& side : xieite::geometry::getSides(polygon)) {
			const std::vector<xieite::geometry::Point> intersections2 = xieite::geometry::getIntersections(linearShape, side);
			if (intersections2.size()) {
				intersections1.push_back(intersections2[0]);
			}
		}
		return intersections1;
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::concepts::LinearShape auto& linearShape, const xieite::geometry::Ellipse& ellipse) noexcept {
		std::vector<xieite::geometry::Point> intersections;
		const xieite::geometry::Point start = xieite::geometry::rotate(xieite::geometry::Point(linearShape.start.x - ellipse.center.x, linearShape.start.y - ellipse.center.y), ellipse.rotation);
		const xieite::geometry::Point end = xieite::geometry::rotate(xieite::geometry::Point(linearShape.end.x - ellipse.center.x, linearShape.end.y - ellipse.center.y), ellipse.rotation);
		const double a = ellipse.radii.y * ellipse.radii.y * (end.x - start.x) * (end.x - start.x) + ellipse.radii.x * ellipse.radii.x * (end.y - start.y) * (end.y - start.y);
		const double b = 2.0 * ellipse.radii.y * ellipse.radii.y * start.x * (end.x - start.x) + 2.0 * ellipse.radii.x * ellipse.radii.x * start.y * (end.y - start.y);
		const double c = ellipse.radii.y * ellipse.radii.y * start.x * start.x + ellipse.radii.x * ellipse.radii.x * start.y * start.y - ellipse.radii.y * ellipse.radii.y * ellipse.radii.x * ellipse.radii.x;
		const double d = b * b - 4.0 * a * c;
		if (d >= 0.0) {
			const double e = (-b + std::sqrt(d)) / 2.0 / a;
			const double f = (-b - std::sqrt(d)) / 2.0 / a;
			const xieite::geometry::Point g = xieite::geometry::rotate(xieite::geometry::Point(start.x + e * (end.x - start.x), start.y + e * (end.y - start.y)), -ellipse.rotation);
			const xieite::geometry::Point h(g.x + ellipse.center.x, g.y + ellipse.center.y);
			if (xieite::geometry::getIntersections(linearShape, h).size()) {
				intersections.push_back(h);
			}
			const xieite::geometry::Point i = xieite::geometry::rotate(xieite::geometry::Point(start.x + f * (end.x - start.x), start.y + f * (end.y - start.y)), -ellipse.rotation);
			const xieite::geometry::Point j(i.x + ellipse.center.x, i.y + ellipse.center.y);
			if (xieite::geometry::getIntersections(linearShape, j).size()) {
				intersections.push_back(j);
			}
		}
		return intersections;
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Point point, const xieite::geometry::Ellipse& ellipse) noexcept {
		std::vector<xieite::geometry::Point> intersections1;
		std::vector<xieite::geometry::Point> intersections2 = xieite::geometry::getIntersections(xieite::geometry::Ray(point, ellipse.center), ellipse);
		if ((intersections2.size() && (intersections2[0] == point)) || ((intersections2.size() > 1) && (intersections2[1] == point))) {
			intersections1.push_back(point);
		}
		return intersections1;
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::concepts::LinearShape auto& linearShape, const xieite::geometry::Circle& circle) noexcept {
		return xieite::geometry::getIntersections(linearShape, xieite::geometry::Ellipse(circle.center, xieite::geometry::Point(circle.radius, circle.radius)));
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Polygon& polygon, const xieite::geometry::Point point) noexcept {
		return xieite::geometry::getIntersections(point, polygon);
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Polygon& polygon, const xieite::concepts::LinearShape auto& linearShape) noexcept {
		return xieite::geometry::getIntersections(linearShape, polygon);
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Polygon& polygon1, const xieite::geometry::Polygon& polygon2) noexcept {
		std::vector<xieite::geometry::Point> intersections1;
		for (const xieite::geometry::Segment& side1 : xieite::geometry::getSides(polygon1)) {
			for (const xieite::geometry::Segment& side2 : xieite::geometry::getSides(polygon2)) {
				const std::vector<xieite::geometry::Point> intersections2 = xieite::geometry::getIntersections(side1, side2);
				if (intersections2.size()) {
					intersections1.push_back(intersections2[0]);
				}
			}
		}
		return intersections1;
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Polygon& polygon, const xieite::geometry::Circle& circle) noexcept {
		std::vector<xieite::geometry::Point> intersections1;
		for (const xieite::geometry::Segment& side : xieite::geometry::getSides(polygon)) {
			const std::vector<xieite::geometry::Point> intersections2 = xieite::geometry::getIntersections(side, circle);
			if (intersections2.size()) {
				intersections1.push_back(intersections2[0]);
			}
		}
		return intersections1;
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Polygon& polygon, const xieite::geometry::Ellipse& ellipse) noexcept {
		std::vector<xieite::geometry::Point> intersections1;
		for (const xieite::geometry::Segment& side : xieite::geometry::getSides(polygon)) {
			const std::vector<xieite::geometry::Point> intersections2 = xieite::geometry::getIntersections(side, ellipse);
			if (intersections2.size()) {
				intersections1.push_back(intersections2[0]);
			}
		}
		return intersections1;
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Ellipse& ellipse, const xieite::geometry::Point point) noexcept {
		return xieite::geometry::getIntersections(point, ellipse);
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Ellipse& ellipse, const xieite::concepts::LinearShape auto& linearShape) noexcept {
		return xieite::geometry::getIntersections(linearShape, ellipse);
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Ellipse& ellipse, const xieite::geometry::Polygon& polygon) noexcept {
		return xieite::geometry::getIntersections(polygon, ellipse);
	}

	std::vector<xieite::geometry::Point> test(const xieite::geometry::Ellipse& ellipse1, const xieite::geometry::Ellipse& ellipse2, const xieite::geometry::Polygon& selection) noexcept {
		static std::size_t level = 0;
		std::cout << "level: " << level++ << '\n';
		std::vector<xieite::geometry::Point> intersections;
		if (xieite::geometry::getIntersections(ellipse1, selection).size() && xieite::geometry::getIntersections(ellipse2, selection).size()) {
			std::cout << "has both ellipses\n";
			const double selectionWidth = selection.points[1].x - selection.points[0].x;
			const double selectionHeight = selection.points[3].y - selection.points[0].y;
			if (xieite::math::approximatelyEqual(selectionWidth, 0.0) && xieite::math::approximatelyEqual(selectionHeight, 0.0)) {
				std::cout << "point: (" << selection.points[0].x << ", " << selection.points[0].y << ")\n";
				intersections.push_back(selection.points[0]);
			} else {
				if (selectionWidth >= selectionHeight) {
					std::cout << "recursing left\n";
					const std::vector<xieite::geometry::Point> subIntersections1 = xieite::geometry::test(ellipse1, ellipse2, xieite::geometry::Polygon(std::vector<xieite::geometry::Point> {
						selection.points[0],
						xieite::geometry::Point(selection.points[0].x + selectionWidth / 2, selection.points[1].y),
						xieite::geometry::Point(selection.points[0].x + selectionWidth / 2, selection.points[2].y),
						selection.points[3]
					}));
					intersections.insert(intersections.end(), subIntersections1.begin(), subIntersections1.end());
					std::cout << "recursing right\n";
					const std::vector<xieite::geometry::Point> subIntersections2 = xieite::geometry::test(ellipse1, ellipse2, xieite::geometry::Polygon(std::vector<xieite::geometry::Point> {
						xieite::geometry::Point(selection.points[0].x + selectionWidth / 2, selection.points[0].y),
						selection.points[1],
						selection.points[2],
						xieite::geometry::Point(selection.points[0].x + selectionWidth / 2, selection.points[3].y)
					}));
					intersections.insert(intersections.end(), subIntersections2.begin(), subIntersections2.end());
				} else {
					std::cout << "recursing bottom\n";
					const std::vector<xieite::geometry::Point> subIntersections3 = xieite::geometry::test(ellipse1, ellipse2, xieite::geometry::Polygon(std::vector<xieite::geometry::Point> {
						selection.points[0],
						selection.points[1],
						xieite::geometry::Point(selection.points[2].x, selection.points[0].y + selectionHeight / 2),
						xieite::geometry::Point(selection.points[3].x, selection.points[0].y + selectionHeight / 2)
					}));
					intersections.insert(intersections.end(), subIntersections3.begin(), subIntersections3.end());
					std::cout << "recursing top\n";
					const std::vector<xieite::geometry::Point> subIntersections4 = xieite::geometry::test(ellipse1, ellipse2, xieite::geometry::Polygon(std::vector<xieite::geometry::Point> {
						xieite::geometry::Point(selection.points[0].x, selection.points[0].y + selectionHeight / 2),
						xieite::geometry::Point(selection.points[1].x, selection.points[0].y + selectionHeight / 2),
						selection.points[2],
						selection.points[3]
					}));
					intersections.insert(intersections.end(), subIntersections4.begin(), subIntersections4.end());
				}
			}
		}
		std::cout << "level: " << --level << '\n';
		return intersections;
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Ellipse& ellipse1, const xieite::geometry::Ellipse& ellipse2) noexcept {
		std::cout << "start\n";
		return xieite::geometry::test(ellipse1, ellipse2, xieite::geometry::getBoundingBox(ellipse1));
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Circle& circle, const xieite::geometry::Point point) noexcept {
		return xieite::geometry::getIntersections(point, circle);
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Circle& circle, const xieite::concepts::LinearShape auto& linearShape) noexcept {
		return xieite::geometry::getIntersections(linearShape, circle);
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Circle& circle, const xieite::geometry::Polygon& polygon) noexcept {
		return xieite::geometry::getIntersections(polygon, circle);
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Circle& circle1, const xieite::geometry::Circle& circle2) noexcept {
		return xieite::geometry::getIntersections(xieite::geometry::Ellipse(circle1.center, xieite::geometry::Point(circle1.radius, circle1.radius)), xieite::geometry::Ellipse(circle2.center, xieite::geometry::Point(circle2.radius, circle2.radius)));
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Circle& circle, const xieite::geometry::Ellipse& ellipse) noexcept {
		return xieite::geometry::getIntersections(xieite::geometry::Ellipse(circle.center, xieite::geometry::Point(circle.radius, circle.radius)), ellipse);
	}

	[[nodiscard]]
	std::vector<xieite::geometry::Point> getIntersections(const xieite::geometry::Ellipse& ellipse, const xieite::geometry::Circle& circle) noexcept {
		return xieite::geometry::getIntersections(circle, ellipse);
	}
}

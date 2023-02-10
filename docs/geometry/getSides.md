# xieite::geometry::getSides
Declared in `<xieite/geometry/getSides.hpp>`

<br/>

Creates line segments between the points of a polygon.

<br/><br/>

## Declaration
```cpp
[[nodiscard]]
constexpr std::vector<xieite::geometry::Segment> getSides(const xieite::geometry::Polygon& polygon) noexcept;
```

<br/><br/>

## Example
```cpp
#include <iostream>
#include <xieite/geometry/Polygon.hpp>
#include <xieite/geometry/Segment.hpp>
#include <xieite/geometry/getLength.hpp>
#include <xieite/geometry/getSides.hpp>

int main() {
	const xieite::geometry::Polygon rectangle({
		{ 0, 0 },
		{ 3, 0 },
		{ 3, 2 },
		{ 0, 2 }
	});
	for (const xieite::geometry::Segment& side : xieite::geometry::getSides(rectangle))
		std::cout << xieite::geometry::getLength(side) << '\n';
}
```
Output:
```
3
2
3
2
```
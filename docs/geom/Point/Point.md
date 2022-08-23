# gcufl::geom::Point::Point
```cpp
Point(const double x = 0, const double y = 0) noexcept;
```
Creates a 2D point.
## Example
```cpp
#include <iostream>
#include <gcufl/geom>

int main() {
	gcufl::geom::Point point(0, 0);

	std::cout << point.x << ", " << point.y << '\n';
}
```
Output:
```
0, 0
```
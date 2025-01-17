# [xieite](../xieite.md)::[algorithms](../algorithms.md)::rotateMatrix
Defined in header [<xieite/algorithms/rotateMatrix.hpp>](../../include/xieite/algorithms/rotateMatrix.hpp)

<br/>

Rotates a 2D vector matrix by 90* up to 3 times in either direction

<br/><br/>

## Synopsis

<br/>

```cpp
template<typename Value>
[[nodiscard]]
constexpr std::vector<std::vector<Value>> rotateMatrix(const std::vector<std::vector<Value>>& matrix, int rotations) noexcept;
```
### Template parameters
- `Value` - Any type
### Parameters
- `matrix` - A constant reference to a `std::vector` of `std::vector`s of `Value`s
- `rotations` - An `int` for which direction to rotate in. Positive = clockwise, negative = counter-clockwise
### Return value
- A `std::vector` of `std::vector`s of the same `Value`s but in a different order

<br/><br/>

## Example
```cpp
#include <iostream>
#include <vector>
#include <xieite/algorithms/rotateMatrix.hpp>

int main() {
	std::vector<std::vector<int>> matrix {
		{ 1, 2, 3 },
		{ 4, 5, 6 }
	};

	for (std::vector<int>& row : xieite::algorithms::rotateMatrix(matrix, 1)) {
		for (int value : row) {
			std::cout << value << ' ';
		}
		std::cout << '\n';
	}
}
```
Output:
```
4 1
5 2
6 3
```

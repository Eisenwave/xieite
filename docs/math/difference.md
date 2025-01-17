# [xieite](../xieite.md)::[math](../math.md)::difference
Defined in header [<xieite/math/difference.hpp>](../../include/xieite/math/difference.hpp)

<br/>

Calculates the absolute difference between two values. Handles some edge cases

<br/><br/>

## Synopses

<br/><br/>

```cpp
template<std::integral Integral>
[[nodiscard]]
constexpr std::make_unsigned<Integral> difference(Integral a, Integral b) noexcept;
```
### Template parameters
- `Integral` - An integral type satisfying `std::integral`
### Parameters
- `a` - An `Integral`
- `b` - Another `Integral`
### Return value
- A `std::make_unsigned` of `Integral`, the absolute difference between `a` and `b`

<br/><br/>

```cpp
template<std::floating_point FloatingPoint>
[[nodiscard]]
constexpr FloatingPoint difference(FloatingPoint a, FloatingPoint b) noexcept;
```
### Template parameters
- `FloatingPoint` - A floating point type satisfying `std::floating_point`
### Parameters
- `a` - A `FloatingPoint`
- `b` - Also a `FloatingPoint`
### Return value
- A `FloatingPoint`, the absolute difference between `a` and `b`

<br/><br/>

## Example
```cpp
#include <cstdint>
#include <iostream>
#include <limits>
#include <xieite/math/difference.hpp>

int main() {
	std::int8_t min = std::numeric_limits<std::int8_t>::min();
	std::int8_t max = std::numeric_limits<std::int8_t>::max();

	std::cout
		<< min << '\n'
		<< max << '\n'
		<< xieite::math::difference(min, max) << '\n';
}
```
Output:
```
-128
127
255
```

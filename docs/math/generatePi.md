# [xieite](../xieite.md)::[math](../math.md)::generatePi
Defined in header [<xieite/math/generatePi.hpp>](../../include/xieite/math/generatePi.hpp)

<br/>

Calculates digits of Pi

<br/><br/>

## Synopsis

<br/>

```cpp
template<std::integral Integral = int>
constexpr std::vector<Integral> generatePi(std::size_t digits) noexcept;
```
### Template parameters
- `Integral` - An integral type satisfying `std::integral`
### Parameters
- `digits` - A `std::size_t`, how many digits to calculate
### Return value
- A `std::vector` of `Integral`s, digits of Pi in order

<br/><br/>

## Example
```cpp
#include <iostream>
#include <xieite/math/generatePi.hpp>

int main() {
	for (int digit : xieite::math::generatePi(50)) {
		std::cout << digit;
	}
	std::cout << '\n';
}
```
Output:
```
31415926535897932384626433832795028841971693993751
```

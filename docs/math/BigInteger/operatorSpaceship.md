# [xieite](../../xieite.md)::[math](../../math.md)::[BigInteger](../BigInteger.md)::operator/
Defined in header [<xieite/math/BigInteger.hpp>](../../../include/xieite/math/BigInteger.hpp)

<br/><br/>

## Synopses

<br/><br/>

```cpp
[[nodiscard]]
constexpr std::strong_ordering operator<=>(const xieite::math::BigInteger& comparand) const noexcept;
```
### Parameters
- `comparand` - A `xieite::math::BigInteger` constant reference
### Return value
- A `std::strong_ordering`

<br/><br/>

```cpp
template<std::integral Integral>
[[nodiscard]]
constexpr std::strong_ordering operator<=>(Integral comparand) const noexcept;
```
### Template parameters
- `Integral` - A type satisfying `std::integral`
### Parameters
- `comparand` - - An `Integral`
### Return value
- A `std::strong_ordering`

<br/><br/>

## Example
```cpp
#include <iostream>
#include <xieite/math/BigInteger.hpp>

int main() {
	std::cout
		<< std::boolalpha
		<< (xieite::math::BigInteger(9) > 3) << '\n';
}
```
Output:
```
true
```

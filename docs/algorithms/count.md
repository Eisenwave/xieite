# [xieite](../xieite.md)::[algorithms](../algorithms.md)::count
Defined in header [<xieite/algorithms/count.hpp>](../../include/xieite/algorithms/count.hpp)

<br/>

Counts the number of arguments provided

<br/><br/>

## Synopsis

<br/>

```cpp
template<typename... Values>
[[nodiscard]]
constexpr std::size_t count(const Values&... values) noexcept;
```
### Template parameters
- `Values...` - Any types
### Parameters
- `values` - Constant `Values...` references
### Return value
- A `std::size_t`, the number of arguments passed

<br/><br/>

## Example
```cpp
#include <iostream>
#include <xieite/algorithms/count.hpp>

int main() {
	std::cout
		<< xieite::algorithms::count(1, 0.4, 'e', "aaaaaaaaaaaa") << '\n';
}
```
Output:
```
4
```

# [xieite](../xieite.md)::[types](../types.md)::demangle
Defined in header [<xieite/types/demangle.hpp>](../../include/xieite/types/demangle.hpp)

<br/>

Demangles the name of a `std::type_info`. Returns original string if demangling fails

<br/><br/>

## Synopsis

<br/>

```cpp
[[nodiscard]]
inline std::string demangle(std::string mangled) noexcept;
```
### Parameters
- `mangled` - A `std::string`, the mangled type name
### Return value
- A `std::string`

<br/><br/>

## Example
```cpp
#include <iostream>
#include <string>
#include <xieite/types/demangle.hpp>

template<typename>
struct Type {};

int main() {
	std::string typeName = typeid(Type<int>).name();

	std::cout
		<< typeName << '\n'
		<< xieite::types::demangle(typeName) << '\n';
}
```
Possible output:
```
4TypeIiE
Type<int>
```

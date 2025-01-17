# [xieite](../xieite.md)::[concepts](../concepts.md)::StreamableIn
Defined in header [<xieite/concepts/StreamableIn.hpp>](../../include/xieite/concepts/StreamableIn.hpp)

<br/>

Specifies that a type can be "streamed" into

<br/><br/>

## Synopsis

<br/>

```cpp
template<typename Any>
concept StreamableIn = requires(std::istream& inputStream, Any value) {
	{ inputStream >> value } -> std::convertible_to<std::istream&>;
};
```
### Template parameters
- `Any` - Any type

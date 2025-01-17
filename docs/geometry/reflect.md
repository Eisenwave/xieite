# [xieite](../xieite.md)::[geometry](../geometry.md)::reflect
Defined in header [<xieite/geometry/reflect.hpp>](../../include/xieite/geometry/reflect.hpp)

<br/>

Reflects a shape across any line

<br/><br/>

## Synopses

<br/><br/>

```cpp
template<xieite::concepts::LinearShape LinearShape>
[[nodiscard]]
constexpr xieite::geometry::Point reflect(const xieite::geometry::Point point, const LinearShape& mirror) noexcept;
```
### Template parameters
- `LinearShape` - A type satisfying `xieite::concepts::LinearShape`
### Parameters
- `point` - A `xieite::geometry::Point`
- `mirror` - A constant `LinearShape` reference
### Return value
- A new reflected `xieite::geometry::Point`

<br/><br/>

```cpp
template<xieite::concepts::LinearShape LinearShape1, xieite::concepts::LinearShape LinearShape2>
[[nodiscard]]
constexpr LinearShape1 reflect(const LinearShape1& line, const LinearShape2& mirror) noexcept;
```
### Template parameters
- `LinearShape1` - A type satisfying `xieite::concepts::LinearShape`
- `LinearShape2` - A type satisfying `xieite::concepts::LinearShape`
### Parameters
- `linearShape` - A constant `LinearShape1` reference
- `mirror` - A constant `LinearShape2` reference
### Return value
- A new reflected `LinearShape`

<br/><br/>

```cpp
template<xieite::concepts::LinearShape LinearShape>
[[nodiscard]]
constexpr xieite::geometry::Polygon reflect(xieite::geometry::Polygon polygon, const LinearShape& mirror) noexcept;
```
### Template parameters
- `LinearShape` - A type satisfying `xieite::concepts::LinearShape`
### Parameters
- `polygon` - A `xieite::geometry::Polygon` constant reference
- `mirror` - A constant `LinearShape` reference
### Return value
- A new reflected `xieite::geometry::Polygon`

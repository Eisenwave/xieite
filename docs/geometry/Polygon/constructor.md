# [xieite](../../xieite.md)::[geometry](../../geometry.md)::[Polygon](../Polygon.md)::Polygon
Defined in header [<xieite/geometry/Polygon.hpp>](../../../include/xieite/geometry/Polygon.hpp)

<br/>

Constructs a `xieite::geometry::Polygon`

<br/><br/>

## Synopsis

<br/>

```cpp
template<xieite::concepts::RangeOf<xieite::geometry::Point> PointRange>
constexpr Polygon(const PointRange& points) noexcept;
```
### Template parameters
- `PointRange` - A type satisfying `xieite::concepts::RangeOf` of `xieite::geometry::Point`
### Parameters
- `points` - A constant `PointRange` reference

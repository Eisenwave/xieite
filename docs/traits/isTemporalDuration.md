# [xieite](../xieite.md)::[traits](../traits.md)::isTemporalDuration
Defined in header [<xieite/traits/isTemporalDuration.hpp>](../../include/xieite/traits/isTemporalDuration.hpp)

<br/>

A boolean representing whether an arithmetic type and two std::intmax_t values can create a std::chrono::duration

<br/><br/>

## Synopses

<br/><br/>

```cpp
template<typename>
inline constexpr bool isTemporalDuration = false;
```
### Template parameters
- Unconstrained parameter

<br/><br/>

```cpp
template<xieite::concepts::Arithmetic Arithmetic, std::intmax_t numerator, std::intmax_t denominator>
inline constexpr bool isTemporalDuration<std::chrono::duration<Arithmetic, std::ratio<numerator, denominator>>> = true;
```
### Template parameters
- `Arithmetic` - An arithmetic type satisfying `xieite::concepts::Arithmetic`
- `numerator` - A `std::intmax_t`
- `denominator` - Another `std::int_max_t`

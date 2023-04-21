#pragma once

#include <compare>
#include <concepts>
#include <cstddef>
#include <ostream>
#include <xieite/types/ConditionalIntegerSign.hpp>
#include <xieite/types/OptimalIntegerSize.hpp>

namespace xieite::math {
	template<std::size_t bits, bool sign>
	class Integer {
	public:
		template<std::integral Integral = int>
		constexpr Integer(const Integral value = 0) noexcept
		: value(value) {}

		constexpr xieite::math::Integer<bits, sign> operator=(const xieite::math::Integer<bits, sign> integer) noexcept {
			this->value = integer.value;
		}

		template<std::integral Integral>
		[[nodiscard]]
		constexpr operator Integral() const noexcept {
			return this->value;
		}

		template<std::size_t integerBits, bool integerSign>
		[[nodiscard]]
		constexpr operator xieite::math::Integer<integerBits, integerSign>() const noexcept {
			return this->value;
		}

		[[nodiscard]]
		constexpr bool operator==(const xieite::math::Integer<bits, sign> integer) const noexcept {
			return this->value == integer.value;
		}

		[[nodiscard]]
		constexpr std::strong_ordering operator<=>(const xieite::math::Integer<bits, sign> integer) const noexcept {
			return this->value <=> integer.value;
		}

		[[nodiscard]]
		constexpr xieite::math::Integer<bits, sign> operator+() const noexcept {
			return this->value;
		}

		[[nodiscard]]
		constexpr xieite::math::Integer<bits, sign> operator+(const xieite::math::Integer<bits, sign> integer) const noexcept {
			return this->value + integer.value;
		}

		constexpr xieite::math::Integer<bits, sign> operator+=(const xieite::math::Integer<bits, sign> integer) noexcept {
			return this->value += integer.value;
		}

		constexpr xieite::math::Integer<bits, sign> operator++() noexcept {
			return ++this->value;
		}

		constexpr xieite::math::Integer<bits, sign> operator++(int) noexcept {
			return this->value++;
		}

		[[nodiscard]]
		constexpr xieite::math::Integer<bits, sign> operator-() const noexcept {
			return -this->value;
		}

		[[nodiscard]]
		constexpr xieite::math::Integer<bits, sign> operator-(const xieite::math::Integer<bits, sign> integer) const noexcept {
			return this->value - integer.value;
		}

		constexpr xieite::math::Integer<bits, sign> operator-=(const xieite::math::Integer<bits, sign> integer) noexcept {
			return this->value -= integer.value;
		}

		constexpr xieite::math::Integer<bits, sign> operator--() noexcept {
			return --this->value;
		}

		constexpr xieite::math::Integer<bits, sign> operator--(int) noexcept {
			return this->value--;
		}

		[[nodiscard]]
		constexpr xieite::math::Integer<bits, sign> operator*(const xieite::math::Integer<bits, sign> integer) const noexcept {
			return this->value * integer.value;
		}

		constexpr xieite::math::Integer<bits, sign> operator*=(const xieite::math::Integer<bits, sign> integer) noexcept {
			return this->value *= integer.value;
		}

		[[nodiscard]]
		constexpr xieite::math::Integer<bits, sign> operator/(const xieite::math::Integer<bits, sign> integer) const noexcept {
			return this->value / integer.value;
		}

		constexpr xieite::math::Integer<bits, sign> operator/=(const xieite::math::Integer<bits, sign> integer) noexcept {
			return this->value /= integer.value;
		}

		[[nodiscard]]
		constexpr xieite::math::Integer<bits, sign> operator%(const xieite::math::Integer<bits, sign> integer) const noexcept {
			return this->value % integer.value;
		}

		constexpr xieite::math::Integer<bits, sign> operator%=(const xieite::math::Integer<bits, sign> integer) noexcept {
			return this->value %= integer.value;
		}

		[[nodiscard]]
		constexpr xieite::math::Integer<bits, sign> operator~() const noexcept {
			return ~this->value;
		}

		[[nodiscard]]
		constexpr xieite::math::Integer<bits, sign> operator|(const xieite::math::Integer<bits, sign> integer) const noexcept {
			return this->value | integer.value;
		}

		constexpr xieite::math::Integer<bits, sign> operator|=(const xieite::math::Integer<bits, sign> integer) noexcept {
			return this->value |= integer.value;
		}

		[[nodiscard]]
		constexpr xieite::math::Integer<bits, sign> operator&(const xieite::math::Integer<bits, sign> integer) const noexcept {
			return this->value & integer.value;
		}

		constexpr xieite::math::Integer<bits, sign> operator&=(const xieite::math::Integer<bits, sign> integer) noexcept {
			return this->value &= integer.value;
		}

		[[nodiscard]]
		constexpr xieite::math::Integer<bits, sign> operator^(const xieite::math::Integer<bits, sign> integer) const noexcept {
			return this->value ^ integer.value;
		}

		constexpr xieite::math::Integer<bits, sign> operator^=(const xieite::math::Integer<bits, sign> integer) noexcept {
			return this->value ^= integer.value;
		}

		[[nodiscard]]
		constexpr xieite::math::Integer<bits, sign> operator<<(const std::size_t distance) const noexcept {
			return this->value << integer;
		}

		constexpr xieite::math::Integer<bits, sign> operator<<=(const std::size_t distance) noexcept {
			return this->value <<= integer;
		}

		[[nodiscard]]
		constexpr xieite::math::Integer<bits, sign> operator>>(const std::size_t distance) const noexcept {
			return this->value >> integer;
		}

		constexpr xieite::math::Integer<bits, sign> operator>>=(const std::size_t distance) noexcept {
			return this->value >>= integer;
		}

		friend constexpr std::ostream& operator<<(std::ostream& outStream, const xieite::math::Integer<bits, sign> self) noexcept {
			return outStream << self.value;
		}

		friend constexpr std::istream& operator>>(std::istream& inStream, xieite::math::Integer<bits, sign>& self) noexcept {
			return inStream >> self.value;
		}

	private:
		xieite::types::ConditionalIntegerSign<xieite::types::OptimalIntegerSize<bits>, sign> value: bits;
	};
}
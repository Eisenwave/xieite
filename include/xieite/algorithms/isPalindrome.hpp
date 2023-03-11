#pragma once
#include <cstddef> // std::size_t
#include <functional> // std::equal_to
#include <iterator> // std::forward_iterator, std::iterator_traits
#include <xieite/concepts/ComparatorCallback.hpp>

namespace xieite::algorithms {
	template<std::forward_iterator I, xieite::concepts::ComparatorCallback<typename std::iterator_traits<I>::value_type> C = std::equal_to<typename std::iterator_traits<I>::value_type>>
	[[nodiscard]]
	constexpr bool isPalindrome(I begin, I end, C&& comparator = C()) noexcept {
		const std::size_t size = std::distance(begin, end) / 2;
		--end;
		for (std::size_t i = 0; i < size; ++i) {
			if (!comparator(*begin, *end))
				return false;
			++begin;
			--end;
		}
		return true;
	}
}

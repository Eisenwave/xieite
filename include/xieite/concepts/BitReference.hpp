#ifndef XIEITE_HEADER_CONCEPTS_BITREFERENCE
#	define XIEITE_HEADER_CONCEPTS_BITREFERENCE

#	include <bitset>
#	include <concepts>
#	include <utility>

namespace xieite::concepts {
	template<typename Any>
	concept BitReference = requires(Any any) {
		{ any.~Any() };
		{ any = std::declval<bool>() } -> std::same_as<Any&>;
		{ any = std::declval<Any>() } -> std::same_as<Any&>;
		{ static_cast<bool>(any) } -> std::same_as<bool>;
		{ ~any } -> std::same_as<bool>;
		{ any.flip() } -> std::same_as<Any&>;
	};
}

// I stole this from someone and made it better

#endif

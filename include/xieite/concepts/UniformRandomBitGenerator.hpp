#ifndef XIEITE_HEADER_CONCEPTS_UNIFORMRANDOMBITGENERATOR
#	define XIEITE_HEADER_CONCEPTS_UNIFORMRANDOMBITGENERATOR

#	include <random>
#	include <type_traits>

namespace xieite::concepts {
	template<typename Any>
	concept UniformRandomBitGenerator = std::uniform_random_bit_generator<std::remove_reference_t<Any>>;
}

#endif

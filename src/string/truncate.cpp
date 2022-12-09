#include <cstddef> // std::size_t
#include <string> // std::string
#include <xieite/string/truncate.hpp> // xieite::string::truncate

std::string xieite::string::truncate(const std::string& string, const std::size_t length, const std::string& suffix) noexcept {
	return string.size() > length		
		? string.substr(0, length - suffix.size()) + suffix
		: string;
}

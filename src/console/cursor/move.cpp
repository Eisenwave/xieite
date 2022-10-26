#include <xieite/console/cursor/move.hpp>
#include <iostream>

void xieite::console::cursor::move(const char direction, const int distance) noexcept {
	std::cout << "\x1b[" << distance << direction;
}

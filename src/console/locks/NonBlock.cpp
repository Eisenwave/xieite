#include <fcntl.h>
#include <xieite/console/locks/NonBlock.hpp>

xieite::console::locks::NonBlock::NonBlock() noexcept {
	fcntl(STDIN_FILENO, F_SETFL, blockingMode | O_NONBLOCK);
}

xieite::console::locks::NonBlock::~NonBlock() {
	fcntl(STDIN_FILENO, F_SETFL, blockingMode);
}

#pragma once

namespace xieite::errors {
	[[noreturn]]
	inline void segmentationFault() noexcept {
		*(volatile int*)(nullptr);
	}
}
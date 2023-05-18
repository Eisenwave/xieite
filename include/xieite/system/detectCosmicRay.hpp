#pragma once

#include <cstddef>
#include <memory>

namespace xieite::system {
	constexpr void detectCosmicRay(const std::size_t detectionBytes) noexcept {
		const std::size_t detectorSize = detectionBytes / sizeof(std::size_t);
		const std::unique_ptr<volatile const std::size_t[]> detector(new volatile const std::size_t[detectorSize] {});
		std::size_t i = 0;
		while (true) {
			if (detector[i]) {
				return;
			}
			i = (i + 1) % detectorSize;
		}
	}
}
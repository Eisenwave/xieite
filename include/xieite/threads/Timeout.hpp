#ifndef XIEITE_HEADER_THREADS_TIMEOUT
#	define XIEITE_HEADER_THREADS_TIMEOUT

#	include <concepts>
#	include "../concepts/TemporalDuration.hpp"
#	include "../threads/Interval.hpp"

namespace xieite::threads {
	class Timeout {
	public:
		template<std::invocable<> Invocable, xieite::concepts::TemporalDuration TemporalDuration>
		Timeout(const Invocable& callback, const TemporalDuration duration) noexcept
		: interval([this, &callback] -> void {
			this->stop();
			callback();
		}, duration) {}

		bool good() const noexcept {
			return this->interval.good();
		}

		void stop() noexcept {
			this->interval.stop();
		}

	private:
		xieite::threads::Interval interval;
	};
}

#endif

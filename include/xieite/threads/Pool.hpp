#ifndef XIEITE_HEADER_THREADS_POOL
#	define XIEITE_HEADER_THREADS_POOL

#	include <condition_variable>
#	include <cstddef>
#	include <functional>
#	include <mutex>
#	include <queue>
#	include <thread>
#	include <vector>

namespace xieite::threads {
	class Pool {
	public:
		Pool(const std::size_t threadCount = std::thread::hardware_concurrency()) {
			this->setThreadCount(threadCount);
		}

		~Pool() {
			for (std::jthread& thread : this->threads) {
				thread.request_stop();
			}
			this->jobsCondition.notify_all();
		}

		void setThreadCount(std::size_t threadCount) {
			if (!threadCount) {
				throw std::invalid_argument("Cannot resize thread pool to zero");
			}
			const std::size_t currentThreadCount = std::ranges::size(this->threads);
			if (threadCount < currentThreadCount) {
				this->threads.resize(threadCount);
				return;
			}
			while (threadCount-- > currentThreadCount) {
				this->threads.emplace_back([this](const std::stop_token stopToken) -> void {
					while (true) {
						auto jobsLock = std::unique_lock<std::mutex>(this->jobsMutex);
						this->jobsCondition.wait(jobsLock, [this, stopToken] noexcept -> bool {
							return std::ranges::size(this->jobs) || stopToken.stop_requested();
						});
						if (!std::ranges::size(this->jobs) && stopToken.stop_requested()) {
							break;
						}
						std::function<void()> job = this->jobs.front();
						this->jobs.pop();
						jobsLock.unlock();
						job();
					}
				});
			}
		}

		std::size_t getThreadCount() const noexcept {
			return std::ranges::size(this->threads);
		}

		void enqueue(const std::function<void()>& job) noexcept {
			auto jobsLock = std::unique_lock<std::mutex>(this->jobsMutex);
			this->jobs.push(job);
			jobsLock.unlock();
			this->jobsCondition.notify_one();
		}

	private:
		std::vector<std::jthread> threads;
		std::queue<std::function<void()>> jobs;
		std::mutex jobsMutex;
		std::condition_variable jobsCondition;
	};
}

// Thanks to evan for help

#endif

#pragma once

#include <termios.h>
#include <unistd.h>
#include <thread>

namespace util {
	namespace io {
		class raw {
			private:
				termios cooked;

			public:
				raw (const bool echo = true);

				~raw ();
		};

		void nonblock ();

		void ignore (char until = 0);

		void clr_scrn ();

		char wait_char ();

		char read_char ();

		template <typename DurationType>
		char read_char_timeout (const DurationType timeout, const bool echo = true, const char defaultChar = 0) {
			util::io::raw lock(echo);
			util::io::nonblock();
			std::this_thread::sleep_for(timeout);
			char input = defaultChar;
			while (read(STDIN_FILENO, &input, 1) == 1);
			return input;
		}
	}
}
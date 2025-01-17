#ifndef XIEITE_HEADER_SYSTEM_STANDARDSTREAMSCONTROLLER
#	define XIEITE_HEADER_SYSTEM_STANDARDSTREAMSCONTROLLER

#	include "../macros/SYSTEM_TYPE.hpp"

#	if XIEITE_SYSTEM_TYPE_UNIX
#		include <cstddef>
#		include <cstdio>
#		include <fcntl.h>
#		include <istream>
#		include <ostream>
#		include <stdio.h>
#		include <string>
#		include <string_view>
#		include <sys/ioctl.h>
#		include <termios.h>
#		include <unistd.h>
#		include "../graphics/Color.hpp"
#		include "../math/baseTo.hpp"
#		include "../memory/bufferSize.hpp"
#		include "../system/OutputPosition.hpp"
#		include "../system/getStreamFile.hpp"

namespace xieite::system {
	template<std::istream& inputStream, std::ostream& outputStream>
	class StandardStreamsController {
	public:
		StandardStreamsController() noexcept
		: inputStreamFile(xieite::system::getStreamFile(inputStream)), inputFileDescriptor(::fileno(this->inputStreamFile)), blockingMode(::fcntl(this->inputFileDescriptor, F_GETFL)), blocking(true), echo(true), canonical(true), signals(true), processing(true) {
			::tcgetattr(this->inputFileDescriptor, &this->cookedMode);
			this->update();
		}

		~StandardStreamsController() {
			::fcntl(this->inputFileDescriptor, F_SETFL, this->blockingMode);
			::tcsetattr(this->inputFileDescriptor, TCSANOW, &this->cookedMode);
			this->resetStyles();
		}

		template<std::istream& otherInputStream, std::ostream& otherOutputStream>
		void operator=(const xieite::system::StandardStreamsController<otherInputStream, otherOutputStream>&) = delete;

		void setInputBlocking(const bool value) noexcept {
			if (this->blocking != value) {
				this->blocking = value;
				this->update();
			}
		}

		void setInputEcho(const bool value) noexcept {
			if (this->echo != value) {
				this->echo = value;
				this->update();
			}
		}

		void setInputCanonical(const bool value) noexcept {
			if (this->canonical != value) {
				this->canonical = value;
				this->update();
			}
		}

		void setInputSignals(const bool value) noexcept {
			if (this->signals != value) {
				this->signals = value;
				this->update();
			}
		}

		void setOutputProcessing(const bool value) noexcept {
			if (this->processing != value) {
				this->processing = value;
				this->update();
			}
		}

		void setTextColor(const xieite::graphics::Color& color) noexcept {
			outputStream << "\x1B[38;2;" << xieite::math::baseTo(10, color.red) << ';' << xieite::math::baseTo(10, color.green) << ';' << xieite::math::baseTo(10, color.blue) << 'm';
		}

		void resetTextColor() noexcept {
			outputStream << "\x1B[38m";
		}

		void setHighlightColor(const xieite::graphics::Color& color) noexcept {
			outputStream << "\x1B[48;2;" << xieite::math::baseTo(10, color.red) << ';' << xieite::math::baseTo(10, color.green) << ';' << xieite::math::baseTo(10, color.blue) << 'm';
		}

		void resetHighlightColor() noexcept {
			outputStream << "\x1B[48m";
		}

		void setTextBold(const bool value) noexcept {
			outputStream << (value ? "\x1B[1m" : "\x1B[21m");
		}

		void setTextItalic(const bool value) noexcept {
			outputStream << (value ? "\x1B[3m" : "\x1B[23m");
		}

		void setTextUnderline(const bool value) noexcept {
			outputStream << (value ? "\x1B[4m" : "\x1B[24m");
		}

		void setTextBlinking(const bool value) noexcept {
			outputStream << (value ? "\x1B[5m" : "\x1B[25m");
		}

		void setColorsSwapped(const bool value) noexcept {
			outputStream << (value ? "\x1B[7m" : "\x1B[27m");
		}

		void setTextVisible(const bool value) noexcept {
			outputStream << (value ? "\x1B[28m" : "\x1B[8m");
		}

		void setTextStrikethrough(const bool value) noexcept {
			outputStream << (value ? "\x1B[9m" : "\x1B[29m");
		}

		void resetStyles() noexcept {
			outputStream << "\x1B[0m";
		}

		void clearScreen() noexcept {
			outputStream << "\x1B[2J";
		}

		void clearLine() noexcept {
			outputStream << "\x1B[2K";
		}

		[[nodiscard]]
		xieite::system::OutputPosition getCursorPosition() noexcept {
			outputStream << "\x1B[6n";
			xieite::system::OutputPosition position = xieite::system::OutputPosition(0, 0);
			std::fscanf(this->inputStreamFile, "\x1B[%i;%iR", &position.row, &position.column);
			return xieite::system::OutputPosition(position.row - 1, position.column - 1);
		}

		void setCursorPosition(const xieite::system::OutputPosition position) noexcept {
			outputStream << "\x1B[" << xieite::math::baseTo(10, position.row + 1) << ';' << xieite::math::baseTo(10, position.column + 1) << 'H';
		}

		void setCursorVisible(const bool value) noexcept {
			outputStream << (value ? "\x1B[?25h" : "\x1B[?25l");
		}

		void setCursorAlternative(const bool value) noexcept {
			outputStream << (value ? "\x1B[s" : "\x1B[u");
		}

		void setScreenAlternative(const bool value) noexcept {
			outputStream << (value ? "\x1B[?47h" : "\x1B[?47l");
		}

		[[nodiscard]]
		xieite::system::OutputPosition getScreenSize() noexcept {
			::winsize size;
			::ioctl(this->inputFileDescriptor, TIOCGWINSZ, &size);
			return xieite::system::OutputPosition(size.ws_row, size.ws_col);
		}

		[[nodiscard]]
		char readCharacter() noexcept {
			const bool blocking = this->blocking;
			const bool canonical = this->canonical;
			this->setInputBlocking(false);
			this->setInputCanonical(false);
			const char input = inputStream.get();
			if (blocking) {
				this->setInputBlocking(true);
			}
			if (canonical) {
				this->setInputCanonical(true);
			}
			return input;	
		}

		[[nodiscard]]
		std::string readString() noexcept {
			const bool blocking = this->blocking;
			const bool canonical = this->canonical;
			this->setInputBlocking(false);
			this->setInputCanonical(false);
			std::string input;
			char buffer;
			while (inputStream.get(buffer)) {
				input += buffer;
			}
			this->setInputBlocking(blocking);
			this->setInputCanonical(canonical);
			return input;
		}

		void putBackString(const std::string_view value) noexcept {
			for (std::size_t i = value.size(); i--;) {
				inputStream.putback(value[i]);
			}
		}

	private:
		std::FILE* const inputStreamFile;
		const int inputFileDescriptor;

		termios cookedMode;
		int blockingMode;

		bool blocking;
		bool echo;
		bool canonical;
		bool signals;
		bool processing;

		void update() noexcept {
			::fcntl(this->inputFileDescriptor, F_SETFL, this->blockingMode | (O_NONBLOCK * !this->blocking));
			::termios rawMode = this->cookedMode;
			rawMode.c_iflag &= ~((ICRNL * !this->signals) | (IXON * !this->signals));
			rawMode.c_lflag &= ~((ECHO * !this->echo) | (ICANON * !this->canonical) | (IEXTEN * !this->signals) | (ISIG * !this->signals));
			rawMode.c_oflag &= ~(OPOST * !this->processing);
			::tcsetattr(this->inputFileDescriptor, TCSANOW, &rawMode);
		}
	};
}

#	else
#		error "System not supported"
#	endif

#endif

#pragma once


namespace gcufl::io::escapeCodes {
	enum struct Background
	: int {
		Black = 40,
		Red,
		Green,
		Yellow,
		Blue,
		Magenta,
		Cyan,
		White,
		Default = 49,
		BrightBlack = 100,
		BrightRed,
		BrightGreen,
		BrightYellow,
		BrightBlue,
		BrightMagenta,
		BrightCyan,
		BrightWhite
	};
}
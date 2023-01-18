#pragma once

#include <stdint.h>

extern "C" int32_t scd30_main();

enum class EventID {
	None,
	Unknown,
	Key,
	Tick,
};

struct EventMessage {
	EventID id;
	InputEvent event;
};

struct State {
	FuriTimer *timer = nullptr;
	uint32_t timerFrequency = 1;

	State(Gui &);
};

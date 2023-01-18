#pragma once

#include <memory>
#include <stdint.h>

#include "i2c.h"

struct BME680;

extern "C" int32_t bme680_main();

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
	I2C *i2c = nullptr;
	uint32_t timerFrequency = 1;
	std::unique_ptr<BME680> bme;
	bool lastReadResult = false;
	bool hasRead = false;

	bool init(FuriMessageQueue *);
	bool readData();
};

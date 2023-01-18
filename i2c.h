#pragma once
// Credit: https://github.com/NaejEL/flipperzero-i2ctools

#include <furi.h>
#include <furi_hal.h>
#include <stdint.h>

#define SCL gpio_ext_pc0
#define SDA gpio_ext_pc1

#define MAX_MESSAGE_SIZE 128
#define MAX_RECORDS 128

namespace I2C {
	enum class BusState {Free, Started};

	struct Frame {
		uint8_t data[MAX_MESSAGE_SIZE];
		bool ack[MAX_MESSAGE_SIZE];
		uint8_t bitIndex;
		uint8_t dataIndex;
	};

	struct Sniffer {
		bool started = false;
		bool first = true;
		BusState state = BusState::Free;
		Frame frames[MAX_RECORDS];
		uint8_t frameIndex = 0;
		uint8_t menuIndex = 0;
		uint8_t rowIndex = 0;

		Sniffer();
		~Sniffer();

		void clearBuffers();
		void startInterrupts();

		static int count;
	};

	void stopInterrupts();
}

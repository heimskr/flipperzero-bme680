// Credit: https://github.com/NaejEL/flipperzero-i2ctools

#include "i2c.h"

namespace I2C {
	int Sniffer::count = 0;

	Sniffer::Sniffer() {
		furi_assert(++count == 1);
		clearBuffers();
	}

	Sniffer::~Sniffer() {
		--count;
		stopInterrupts();
	}

	void Sniffer::clearBuffers() {
		furi_assert(this);
		for (uint8_t i = 0; i < MAX_RECORDS; i++) {
			for (uint8_t j = 0; j < MAX_MESSAGE_SIZE; j++) {
				frames[i].ack[j] = false;
				frames[i].data[j] = 0;
			}

			frames[i].bitIndex = 0;
			frames[i].dataIndex = 0;
		}

		frameIndex = 0;
		state = BusState::Free;
		first = true;
	}

	void Sniffer::startInterrupts() {
		furi_assert(this);
		furi_hal_gpio_init(&SCL, GpioModeInterruptRise, GpioPullNo, GpioSpeedHigh);
		furi_hal_gpio_add_int_callback(&SCL, +[](void *userdata) {
			auto &sniffer = *reinterpret_cast<Sniffer *>(userdata);
			if (sniffer.state == BusState::Free)
				return;
			uint8_t frame = sniffer.frameIndex;
			uint8_t bit = sniffer.frames[frame].bitIndex;
			uint8_t data_idx = sniffer.frames[frame].dataIndex;
			if (bit < 8) {
				sniffer.frames[frame].data[data_idx] <<= 1;
				sniffer.frames[frame].data[data_idx] |= static_cast<int>(furi_hal_gpio_read(&SDA));
				sniffer.frames[frame].bitIndex++;
			} else {
				sniffer.frames[frame].ack[data_idx] = !furi_hal_gpio_read(&SDA);
				sniffer.frames[frame].dataIndex++;
				sniffer.frames[frame].bitIndex = 0;
			}
		}, this);

		// Add Rise and Fall Interrupt on SDA pin
		furi_hal_gpio_init(&SDA, GpioModeInterruptRiseFall, GpioPullNo, GpioSpeedHigh);
		furi_hal_gpio_add_int_callback(&SDA, +[](void *userdata) {
			auto &sniffer = *reinterpret_cast<Sniffer *>(userdata);
			
			// SCL is low, maybe cclock stretching
			if (!furi_hal_gpio_read(&SCL))
				return;

			if (sniffer.state == BusState::Started && furi_hal_gpio_read(&SDA)) {
				// Check for stop condition: SDA rising while SCL is High
				sniffer.state = BusState::Free;
			} else if (!furi_hal_gpio_read(&SDA)) {
				// Check for start condition: SDA falling while SCL is high
				sniffer.state = BusState::Started;
				if (sniffer.first) {
					sniffer.first = false;
					return;
				}

				sniffer.frameIndex++;
				if (MAX_RECORDS <= sniffer.frameIndex)
					sniffer.clearBuffers();
			}
		}, this);
	}

	void stopInterrupts() {
		furi_hal_gpio_remove_int_callback(&SCL);
		furi_hal_gpio_remove_int_callback(&SDA);
		// Reset GPIO pins to default state
		furi_hal_gpio_init(&SCL, GpioModeAnalog, GpioPullNo, GpioSpeedLow);
		furi_hal_gpio_init(&SDA, GpioModeAnalog, GpioPullNo, GpioSpeedLow);
	}
}
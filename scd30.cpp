#include <stdarg.h>
#include <stdlib.h>

#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <input/input.h>

// #include <memory>

#define LOG_LEVEL 6

#include "scd30_logging.h"
#include "err.h"
#include "scd30.h"
#include "bme680.h"

static void input_cb(InputEvent *event, FuriMessageQueue *queue) {
	SCOPED_ENTER;

	furi_assert(event);
	furi_assert(queue);

	EventMessage message {EventID::Key, *event};
	furi_message_queue_put(queue, &message, FuriWaitForever);
}

static void draw_cb(Canvas * const canvas, void *ctx) {

}

static void timer_cb(FuriMessageQueue *queue) {
	SCOPED_ENTER;

	furi_assert(queue);

	EventMessage message {EventID::Tick};
	furi_message_queue_put(queue, &message, 0);
}

extern "C" int32_t scd30_main() {
	SCOPED_ENTER;

	err_t error = static_cast<err_t>(0);
	Gui *gui = nullptr;
	// std::unique_ptr<State> state;
	State *state = nullptr;
	ViewPort *viewport = nullptr;
	ValueMutex mutex {0};
	FuriMessageQueue *queue = nullptr;
	constexpr uint32_t  queue_size = 8;
	EventMessage message;
	BME680 *bme = nullptr;

	if (!(queue = furi_message_queue_alloc(queue_size, sizeof(message)))) {
		ERROR(errs[(error = ERR_MALLOC_QUEUE)]);
		goto bail;
	}

	if (!(gui = reinterpret_cast<Gui *>(furi_record_open("gui")))) {
		ERROR(errs[(error = ERR_NO_GUI)]);
		goto bail;
	}

	// if (!(state = std::make_unique<State>())) {
	if (!(state = new State)) {
		ERROR(errs[(error = ERR_MALLOC_STATE)]);
		goto bail;
	}

	// if (!init_mutex(&mutex, state.get(), sizeof(*state))) {
	if (!init_mutex(&mutex, state, sizeof(*state))) {
		ERROR(errs[(error = ERR_NO_MUTEX)]);
		goto bail;
	}

	if (!(viewport = view_port_alloc())) {
		ERROR(errs[(error = ERR_MALLOC_VIEW)]);
		goto bail;
	}

	if (!state->init(queue)) {
		ERROR(errs[(error = ERR_INIT_STATE)]);
		goto bail;
	}

	view_port_input_callback_set(viewport, input_cb, queue);
	view_port_draw_callback_set(viewport, draw_cb, &mutex);
	gui_add_view_port(gui, viewport, GuiLayerFullscreen);

	if (!(state->timer = furi_timer_alloc(timer_cb, FuriTimerTypePeriodic, queue))) {
		ERROR(errs[(error = ERR_NO_TIMER)]);
		goto bail;
	}

	INFO("Initialized");

	for (bool run = true; run;) {
		FuriStatus status = FuriStatusErrorTimeout;
		while ((status = furi_message_queue_get(queue, &message, 60000)) == FuriStatusErrorTimeout);

		if (status != FuriStatusOk) {
			switch (status) {
				case FuriStatusErrorTimeout:    DEBUG(errs[       DEBUG_QUEUE_TIMEOUT]);    break;
				case FuriStatusError:           ERROR(errs[(error = ERR_QUEUE_RTOS)]);      goto bail;
				case FuriStatusErrorResource:   ERROR(errs[(error = ERR_QUEUE_RESOURCE)]);  goto bail;
				case FuriStatusErrorParameter:  ERROR(errs[(error = ERR_QUEUE_BADPRM)]);    goto bail;
				case FuriStatusErrorNoMemory:   ERROR(errs[(error = ERR_QUEUE_NOMEM)]);     goto bail;
				case FuriStatusErrorISR:        ERROR(errs[(error = ERR_QUEUE_ISR)]);       goto bail;
				default:                        ERROR(errs[(error = ERR_QUEUE_UNK)]);       goto bail;
			}
		} else {
			// if (!(state = reinterpret_cast<State *>(acquire_mutex_block(&mutex)))) {
			if (!acquire_mutex_block(&mutex)) {
				ERROR(errs[(error = ERR_MUTEX_BLOCK)]);
				goto bail;
			}

			if (message.id == EventID::Tick) {
				INFO("Tick.");
			} else if (message.id == EventID::Key) {
				if (message.event.type == InputTypeRelease) {
					// INFO("Key: [%d]", static_cast<int>(message.event.key));
					if (message.event.key == InputKeyBack) {
						run = false;
					} else if (message.event.key == InputKeyDown) {
						if (bme == nullptr)
							bme = new BME680;

						static bool begun = false;
						static bool ready = false;
						if (!begun) {
							begun = true;
							if (!bme->begin())
								ERROR("bme.begin() returned false");
							else
								ready = true;
						}

						if (ready)
							INFO("Temperature: [%f]", bme->readTemperature());
					}
				}
			} else {
				WARN("Unknown message ID [%d]", static_cast<int>(message.id));
			}

			view_port_update(viewport);

			// if (!release_mutex(&mutex, state.get())) {
			if (!release_mutex(&mutex, state)) {
				ERROR(errs[(error = ERR_MUTEX_RELEASE)]);
				goto bail;
			}
		}
	}
	
	
bail:
	if (state->timer) {
		furi_timer_stop(state->timer);
		furi_timer_free(state->timer);
		state->timer = nullptr;
	}

	gui_remove_view_port(gui, viewport);

	if (viewport) {
		view_port_enabled_set(viewport, false);
		view_port_free(viewport);
		viewport = nullptr;
	}

	if (mutex.mutex) {
		delete_mutex(&mutex);
		mutex.mutex = nullptr;
	}

	// if (state->text) {
	// 	free(state->text);
	// 	state->text = nullptr;
	// }

	// state.reset();
	delete state;
	state = nullptr;

	delete bme;

	furi_record_close("gui");

	if (queue) {
		furi_message_queue_free(queue);
		queue = nullptr;
	}

	return 0;
}

bool State::init(FuriMessageQueue *queue) {
	furi_assert(this);

	if (!(timer = furi_timer_alloc(timer_cb, FuriTimerTypePeriodic, queue))) {
		ERROR(errs[ERR_NO_TIMER]);
		return false;
	}

	// if (!(i2c = new I2C)) {
	// 	ERROR(errs[ERR_NO_I2C]);
	// 	return false;
	// }

	return true;
}

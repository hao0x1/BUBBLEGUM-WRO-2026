#pragma once

using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(mux) static_cast<void>(mux)
#define portEXIT_CRITICAL(mux) static_cast<void>(mux)

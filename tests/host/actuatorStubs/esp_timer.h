#pragma once

#include <cstdint>

struct FakeEspTimer;
using esp_timer_handle_t = FakeEspTimer*;
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
constexpr int ESP_TIMER_TASK = 0;

struct esp_timer_create_args_t {
  void (*callback)(void*) = nullptr;
  void* arg = nullptr;
  int dispatch_method = ESP_TIMER_TASK;
  const char* name = nullptr;
  bool skip_unhandled_events = false;
};

int64_t esp_timer_get_time();
esp_err_t esp_timer_create(const esp_timer_create_args_t* args,
                           esp_timer_handle_t* handle);
esp_err_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period);
esp_err_t esp_timer_stop(esp_timer_handle_t timer);
esp_err_t esp_timer_delete(esp_timer_handle_t timer);

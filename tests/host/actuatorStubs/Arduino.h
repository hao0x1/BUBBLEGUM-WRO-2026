#pragma once

#include <cstddef>
#include <cstdint>

constexpr uint8_t LOW = 0;
constexpr uint8_t HIGH = 1;
constexpr uint8_t OUTPUT = 1;

void digitalWrite(uint8_t pin, uint8_t value);
void pinMode(uint8_t pin, uint8_t mode);
double ledcSetup(uint8_t channel, double frequency, uint8_t resolution);
void ledcWrite(uint8_t channel, uint32_t duty);
void ledcAttachPin(uint8_t pin, uint8_t channel);

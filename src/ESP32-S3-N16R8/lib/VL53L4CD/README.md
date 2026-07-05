# Vendored VL53L4CD driver

This is Pololu VL53L4CD Arduino library 1.0.0 under its included license.
It is vendored so the car diagnostic uses a reviewed implementation rather
than a mutable PlatformIO cache. Local hardening validates every I2C block-read
length and treats a zero-SPAD result as invalid instead of dividing by zero.

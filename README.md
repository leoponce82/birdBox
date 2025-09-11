# BirdBox: Mega, Sensors & Uno R4 WiFi

Welcome to **BirdBox**, where a flock of laser rangefinders and a WiFi-enabled sidekick keep watch over our feathered  Ibis friends.

## allSensors (Arduino Mega)

* Drives **12 VL53L1X** time-of-flight sensors through a TCA9548A I2C multiplexer.
* Shares the bus with an SSD1306 OLED and DS3231 RTC on their own channel.
* Triggers a stepper motor when sensor 1 sees movement and shows live distances on the OLED.
* Hands off readings to the Uno R4 over I2C.


## Uno R4 WiFi

* Listens for framed distance data via I2C and flashes status icons on the built-in LED matrix.
* Connects to WiFi and publishes JSON payloads to `birdBox/unoR4/sensors` over MQTT.
* Replies with a friendly `'K'` when the Mega checks that it’s awake.

## Project Structure

* `allSensors/` – Mega sketch that powers sensors, motor and display.
* `unoR4/` – WiFi companion that forwards data to the network.
* Other folders are early experiments and are not part of the main build.


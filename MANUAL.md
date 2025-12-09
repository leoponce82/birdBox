# BirdBox Operation Manual

This guide explains how to operate the BirdBox training system from the front panel menu. It covers the hardware roles, day-to-day interaction steps, and the limits of each configurable feature.

## System overview
- The Arduino Mega sketch (`allSensors`) drives twelve VL53L1X time-of-flight sensors through a multiplexer, handles the OLED and RTC, and moves the stepper-driven food tunnel when triggered.【F:README.md†L5-L10】
- The Uno R4 WiFi listens to framed sensor data over I2C, lights its LED matrix for status, and publishes JSON payloads to `birdBox/unoR4/sensors` via MQTT while acknowledging the Mega with a `K` heartbeat.【F:README.md†L13-L17】

### Front panel layout
- Four tunnel sides are addressed as **Side 1–4** with up to four physical buttons each; the code expects 16 button inputs mapped to panels 4→1, with panel 1 wired in reverse order for buttons 1–4.【F:allSensors/allSensors/allSensors.ino†L606-L615】
- Sequences per side can be between **1 and 8 button presses**, defaulting to **1-2-3-4** (length 4).【F:allSensors/allSensors/allSensors.ino†L103-L116】

## Core behavior
- **Display modes** control what appears on the OLED during fast sampling: `Buttons`, `Sensors`, or low-chatter `Deployment` mode that only updates on events.【F:allSensors/allSensors/allSensors.ino†L31-L40】【F:allSensors/allSensors/allSensors.ino†L2351-L2374】
- **Logic modes** govern when rewards fire: `Ordered` (must match sequence order), `Set` (any order), or `Immediate` (any approved button rewards).【F:allSensors/allSensors/allSensors.ino†L43-L48】【F:allSensors/allSensors/allSensors.ino†L2390-L2394】 In Immediate mode on **Side 1**, reward delivery is deferred briefly to allow menu entry before feeding.【F:allSensors/allSensors/allSensors.ino†L2244-L2276】
- Baseline, sleep, and fast sampling states are automatic; wake events come from sensor movement, pecks, or button presses and push the system into fast sampling and logging.【F:allSensors/allSensors/allSensors.ino†L2664-L2726】

## Entering and exiting the menu
1. **Hold all four Side 1 buttons for ~3 seconds**. From idle, this opens the menu; if you are already in the menu, the same hold saves and exits.【F:allSensors/allSensors/allSensors.ino†L2290-L2314】
2. When the menu opens, the OLED shows “Menu mode” and asks you to release the panel 1 buttons before navigation begins.【F:allSensors/allSensors/allSensors.ino†L2005-L2016】
3. Menu navigation only accepts presses from Side 1; other panels are ignored while in menu mode.【F:allSensors/allSensors/allSensors.ino†L2573-L2583】

## Menu map and workflows
### Main Menu
- **1. Change Sequences** → opens the sequence editor.
- **2. Change Display** → pick the fast-sampling display mode.
- **3. Options** → advanced settings.
- **4. POWER OFF** → prompts for shutdown confirmation.【F:allSensors/allSensors/allSensors.ino†L2337-L2349】【F:allSensors/allSensors/allSensors.ino†L2542-L2550】

### Editing sequences
1. **Select side (1–4).**【F:allSensors/allSensors/allSensors.ino†L2421-L2434】
2. **Adjust length.** `1` decreases and `2` increases between the 1–8 limit; `3` proceeds, `4` goes back.【F:allSensors/allSensors/allSensors.ino†L2437-L2458】
3. **Enter the new sequence.** Button entries fill the requested length, then the system shows a confirmation screen.【F:allSensors/allSensors/allSensors.ino†L2461-L2469】
4. **Confirm.** `1` saves to EEPROM (and resets progress), `2` re-enters, `3` returns to Main Menu, `4` returns to side selection.【F:allSensors/allSensors/allSensors.ino†L2473-L2488】【F:allSensors/allSensors/allSensors.ino†L2056-L2078】
5. **Factory reset** is available under Options and restores default sequences for all sides.【F:allSensors/allSensors/allSensors.ino†L2377-L2398】【F:allSensors/allSensors/allSensors.ino†L2454-L2458】

### Display selection
- Choose `Buttons`, `Sensors`, or `Deployment`; selection is persisted immediately. Press `4` to go back.【F:allSensors/allSensors/allSensors.ino†L2351-L2374】【F:allSensors/allSensors/allSensors.ino†L2490-L2507】

### Options
- **Factory Reset** restores defaults and exits.【F:allSensors/allSensors/allSensors.ino†L2510-L2539】【F:allSensors/allSensors/allSensors.ino†L2046-L2054】
- **Uno R4 link** toggles whether the Mega expects the Uno; the setting is saved instantly.【F:allSensors/allSensors/allSensors.ino†L2512-L2519】【F:allSensors/allSensors/allSensors.ino†L2377-L2398】
- **Logic mode** cycles through `Ordered`, `Set`, and `Immediate`; each press saves the choice.【F:allSensors/allSensors/allSensors.ino†L2519-L2526】【F:allSensors/allSensors/allSensors.ino†L2390-L2394】
- **Back** returns to the Main Menu.【F:allSensors/allSensors/allSensors.ino†L2526-L2529】

### Shutdown
- Selecting POWER OFF prompts for confirmation; `1` initiates the shutdown sequence, `2` cancels back to the menu.【F:allSensors/allSensors/allSensors.ino†L2321-L2334】【F:allSensors/allSensors/allSensors.ino†L2542-L2550】

## Practical tips and limits
- Menu input is rate-limited: a short delay is added before handling a stored button press to filter accidental clicks.【F:allSensors/allSensors/allSensors.ino†L123-L125】【F:allSensors/allSensors/allSensors.ino†L2584-L2592】
- Holding the menu shortcut while in a submenu always saves changes (including display and logic choices) before exiting.【F:allSensors/allSensors/allSensors.ino†L2306-L2313】【F:allSensors/allSensors/allSensors.ino†L2018-L2034】
- Reward actions respect a cooldown window and will skip immediate repeats if the tunnel just moved.【F:allSensors/allSensors/allSensors.ino†L2232-L2238】【F:allSensors/allSensors/allSensors.ino†L2267-L2274】
- Deploy mode keeps the OLED blank unless an event happens, useful for stealth deployments where light leakage matters.【F:allSensors/allSensors/allSensors.ino†L31-L35】【F:allSensors/allSensors/allSensors.ino†L2616-L2649】

## Communication expectations
- If Uno communication is disabled in Options, the Mega will continue local training but skip I2C handshakes and network publishing; enable it when you need MQTT feeds via the Uno R4 path described above.【F:README.md†L13-L17】【F:allSensors/allSensors/allSensors.ino†L2377-L2398】


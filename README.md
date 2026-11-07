# ARMAC Fire Defence: Arduino code

ARMAC (Adjustable, Remotely Monitored And Controlled) is a home bushfire defence sprinkler system. It is described in detail at <https://armac.au>.

This repository holds the current code for the system's three Arduino UNO R4 WiFi boards.

| Folder | Board | Role |
|---|---|---|
| `AR1/` | AR1, in the house | Links the system to Home Assistant over MQTT, sends a heartbeat to AR3, and passes manual commands to AR3 over RS-485. |
| `AR2/` | AR2, in the pump shed | Reads the sensors (temperatures, tank level, flow, pressure, ember detector) and broadcasts them by UDP over its own WiFi access point. |
| `AR3/` | AR3, in the pump shed | Controls the pump and valves through the relays. If the heartbeat from AR1 stops, it runs the system on its own (Autonomous mode), with fallbacks if sensors fail. |

## Setting up

1. Copy `config_example.h` to `config.h` in each sketch folder (`AR1/`, `AR2/`, `AR3/`).
2. Fill in your own WiFi networks, MQTT broker address and credentials, and the password for AR2's access point. `config.h` is listed in `.gitignore` so it is never committed.
3. Open each sketch in the Arduino IDE and upload it to its board (board type: Arduino UNO R4 WiFi).

Each sketch starts with a dated change log describing recent changes.

## Notes

- All three boards run a hardware watchdog and restart themselves if they stop responding or lose their connection for too long. A restart keeps the system in Autonomous mode if it was in that mode.
- A TestSys mode (AR1 pin A3 and AR3 pin D11 strapped to GND) runs a second, test copy of the hardware against a separate Home Assistant instance.
- AR1 and AR3 talk over RS-485, with a two-wire backup link (SoftwareSerial) that AR1 switches to if RS-485 goes quiet for 15 s, returning after 60 s of steady RS-485. Every message carries a CRC-8 checksum. By default bad messages are only counted (reported to Home Assistant); setting `FRAME_REJECT` to 1 in both AR1 and AR3 makes them drop bad messages.
- The design, wiring and test history are documented in the Development Notes and Technical Notes at <https://armac.au>.

This code is shared as-is for others building similar systems. It controls pumps and valves in a fire emergency, so test any adaptation thoroughly before relying on it.

## Licence

CC0 1.0 Universal (public domain dedication). See `LICENSE`.

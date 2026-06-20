# ESP-NOW Slave Node

ESP32-based slave transceiver for a home-monitoring sensor network. Scans for the master's soft-AP to lock onto its channel, then listens for discovery and poll requests, responding with sensor data.

## Hardware

| Item | Value |
|------|-------|
| Board | ESP32 DOIT DevKit v1 |
| Framework | Arduino (PlatformIO) |
| LED | Onboard (GPIO 2) |

## Features

- **Auto channel discovery** – scans for the master's soft-AP (`ESP32-HomeMon`) at boot to find the correct ESP-NOW channel; falls back to channel 1 if the AP is not found within 10 s
- **Zero-config node ID** – derived from the last byte of the device MAC, so each slave is self-identifying without manual configuration
- **Collision avoidance** – applies a random 0–199 ms backoff before sending `PKT_DISCOVER_RESP`, preventing simultaneous replies when multiple slaves hear the same broadcast (hidden-node problem)
- **Auto rescan** – if no master packet is received for 30 s the slave deinitialises ESP-NOW and repeats the full channel-scan sequence, handling the case where the master restarts on a different channel
- **LED status** – 1 s blink while scanning / not yet connected; 250 ms blink once the master peer is registered

## Getting Started

1. Flash the master (`ESP_NOW`) to one ESP32 first so its soft-AP is visible during the slave's scan.
2. Open this project in PlatformIO.
3. Build and flash: `pio run -t upload`.
4. Open the serial monitor at 115 200 baud.
5. The slave will scan, find the master's AP, initialise ESP-NOW, and begin responding to discovery and poll requests.

Multiple slaves can run simultaneously. Each picks a unique node ID from its own MAC and staggers its discovery response to avoid collisions.

## LED Indicator

| Pattern | Meaning |
|---------|---------|
| 1 s on / 1 s off | Scanning for master or not yet registered |
| 250 ms on / 250 ms off | Connected — master peer registered |

## Protocol

The slave responds to two packet types initiated by the master. All packets use the shared `espnow_packet_t` wire format (13 bytes: header + sensor payload).

| Type | Value | Direction | Description |
|------|-------|-----------|-------------|
| `PKT_DISCOVER` | `0x01` | Master → broadcast | Invite slaves to identify |
| `PKT_DISCOVER_RESP` | `0x02` | Slave → master | Node ID + initial sensor snapshot |
| `PKT_POLL` | `0x03` | Master → slave | Request latest sensor data |
| `PKT_POLL_RESP` | `0x04` | Slave → master | Current sensor data |

> **Sensor payload** (`sensor_data_t`) is a placeholder. Replace `analogValue`, `digitalInputs`, and `uptimeSec` with real hardware readings — any change must also be made to `espnow_types.h` in the master project.

## Key Configuration

| Constant | Default | Description |
|----------|---------|-------------|
| `MASTER_AP_SSID` | `"ESP32-HomeMon"` | SSID scanned for at boot to find ESP-NOW channel |
| `DEFAULT_ESPNOW_CHANNEL` | `1` | Fallback channel if scan finds no master AP |
| `CHANNEL_SCAN_TIMEOUT_MS` | `10 000` | Max time to wait for scan completion (ms) |
| `SENSOR_UPDATE_INTERVAL_MS` | `500` | How often the sensor snapshot is refreshed (ms) |
| `RESCAN_INTERVAL_MS` | `30 000` | Idle timeout before triggering a channel re-scan (ms) |
| `LED_SCAN_BLINK_MS` | `1 000` | LED toggle period while scanning (ms) |
| `LED_CONN_BLINK_MS` | `250` | LED toggle period when connected (ms) |

## Project Structure

```
ESP_NOW_Slave/
├── src/
│   └── main.cpp       # Slave state machine, ESP-NOW logic, LED
├── include/
│   ├── hardware.h     # Board pin / peripheral definitions
│   └── gprintf/       # Debug UART printf library
└── platformio.ini
```

## Related

- **Master firmware**: [ESP_NOW](https://github.com/gerrieuae/ESP_NOW) — discovery, polling, and web dashboard

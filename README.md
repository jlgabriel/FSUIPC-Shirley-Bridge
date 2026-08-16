# FSUIPC Shirley Bridge

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python Version](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Websockets](https://img.shields.io/badge/library-websockets-green.svg)](https://websockets.readthedocs.io/)

**A high-performance bridge that connects Shirley AI with Microsoft Flight Simulator 2020/2024, using the FSUIPC WebSocket Server. This bridge enables real-time, bidirectional communication, streaming rich flight data to Shirley and translating its commands into in-sim actions.**

This tool is the essential link for creating an immersive, voice-controlled cockpit experience, allowing Shirley to have deep, context-aware conversations about the ongoing flight, from aircraft systems to navigation and environmental conditions.

<br>

## 📜 Table of Contents

- [Features](#-features)
- [High-Level Architecture](#-high-level-architecture)
- [Getting Started](#-getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Configuration](#configuration)
- [Running the Bridge](#-running-the-bridge)
- [Technical Deep Dive](#-technical-deep-dive)
  - [Core Components](#core-components)
  - [The Data Pipeline (Read Path)](#the-data-pipeline-read-path)
  - [The Command Pipeline (Write Path)](#the-command-pipeline-write-path)
  - [Key Data Structures](#key-data-structures)
    - [The `READ_SIGNALS` Dictionary](#the-read_signals-dictionary)
    - [The `SET_FIELDS` Dictionary](#the-set_fields-dictionary)
  - [The Transformation Layer](#the-transformation-layer)
- [Shirley WebSocket API](#shirley-websocket-api)
- [Troubleshooting](#-troubleshooting)
- [Acknowledgments](#-acknowledgments)
- [License](#-license)

<br>

## ✨ Features

This bridge provides a comprehensive set of features to ensure seamless integration between the simulator and AI.

### Real-time Data Streaming (4Hz)
-   **Position & Navigation**: Full GPS data including latitude, longitude, MSL/AGL altitude, indicated airspeed (IAS), ground speed, and vertical speed.
-   **Attitude**: Precise attitude information such as true/magnetic heading, pitch, and roll angles.
-   **Aircraft Systems**: Status of main battery, pitot heat, parking brake, and propeller de-ice.
-   **Lights**: Real-time status of navigation, landing, taxi, and strobe lights, decoded from FSUIPC bitmasks.
-   **Control Surfaces & Levers**: Percentage-based positions for flaps, landing gear, speed brakes, throttle, mixture, propeller and carburettor heat levers.
-   **Engine Indicators**: N1 %, EGT and manifold pressure for two engines, plus the stall warning. Piston RPM has no usable legacy offset in MSFS 2024 and is not published — see the note in `READ_SIGNALS`.
-   **Radios & Navigation**: Active and standby frequencies for COM/NAV radios and the current transponder code.
-   **Autopilot**: Full autopilot status, including engagement state, active modes (HDG, ALT, VS), flight director, wing leveller, and bug settings.
-   **Environment**: Ambient conditions like wind speed, direction, and outside air temperature.

### Bidirectional Aircraft Control
-   **32 settable fields** across levers, autopilot, radios, lights, systems, indicators and time — essentially all of `SetSimData` except weather. See the [capability matrix](docs/FSUIPC-SETPOINT-CAPABILITIES.md).
-   **Control events by default**: commands go out as MSFS calculator code, so a command the aircraft cannot honour leaves nothing behind. Direct offset writes are restricted to the two cases verified against a running simulator.
-   **Honest refusals**: fields MSFS 2024 cannot set — the whole weather group, failures, crash and reset — are rejected with the reason instead of accepted and ignored.

### Advanced Capabilities
-   **Dynamic Capabilities Reporting**: Automatically informs connecting clients (like Shirley) of all readable data points and settable fields upon connection.
-   **Automatic Reconnection**: The FSUIPC WebSocket server drops client connections intermittently and without a close frame. The bridge reconnects with backoff, re-declares its offset group and re-subscribes.
-   **Magnetic Variation Correction**: Automatically calculates and broadcasts the magnetic heading by correcting the true heading with the current magnetic variation.
-   **Calculated Ground Track**: Derives the aircraft's true ground track by calculating the bearing between consecutive GPS coordinates, distinct from the aircraft's heading.
-   **Robust Data Transformation**: A comprehensive library of transform functions converts raw FSUIPC offset data (e.g., BCD, bitfields, scaled integers) into standardized, human-readable units (degrees, knots, kHz, etc.).

<br>

## 🏛️ High-Level Architecture

The bridge operates as a central hub with two primary components running asynchronously, orchestrated by Python's `asyncio` library.

1.  **FSUIPC WebSocket Client**:
    *   Connects to the **FSUIPC WebSocket Server** (running within MSFS).
    *   Declares a list of required data "offsets" (memory addresses) from the simulator.
    *   Subscribes to receive updates for these offsets at a high frequency (default 4Hz).
    *   Receives raw data, processes it through a transformation layer, and updates a central `SimData` state object.

2.  **Shirley WebSocket Server**:
    *   Listens for incoming connections from clients like **Shirley AI**.
    *   On connection, it sends a `Capabilities` message, detailing all available data points and control commands.
    *   Continuously broadcasts the complete, formatted flight data snapshot from the `SimData` object to all connected clients.
    *   Receives `SetSimData` command messages from clients, encodes them into the appropriate FSUIPC format, and forwards them to the FSUIPC client for execution in the simulator.

This dual-client/server architecture, built on a non-blocking I/O model, ensures a high-performance, responsive data pipeline from the simulator to the AI and back.

<br>

## 🚀 Getting Started

Follow these steps to get the bridge up and running.

### Prerequisites
-   **Microsoft Flight Simulator 2020/2024**
-   **FSUIPC7**: The registered version is not required for WebSocket Server functionality.
-   **Python 3.8+**
-   The `websockets` Python library.

### Installation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/yourusername/fsuipc-shirley-bridge.git
    cd fsuipc-shirley-bridge
    ```

2.  **Install the required Python libraries:**
    ```bash
    pip install -r requirements.txt
    ```

    Or manually:
    ```bash
    pip install websockets python-dotenv
    ```

3.  **Configure FSUIPC WebSocket Server:**
    *   In the FSUIPC7 settings dialog within MSFS, navigate to the "WebSocket Server" tab.
    *   Check the box to **Enable WebSocket Server**.
    *   Set the server port to `2048` (this is the default expected by the script).
    *   Ensure that `localhost` access is permitted.

### Configuration

The bridge can be configured using **environment variables** or a `.env` file. This makes it easy to change settings without modifying the code.

1.  **Copy the example configuration file:**
    ```bash
    cp .env.example .env
    ```

2.  **Edit `.env` to customize your settings:**
    ```bash
    # FSUIPC Connection
    FSUIPC_WS_URL=ws://localhost:2048/fsuipc/

    # Shirley WebSocket Server
    WS_HOST=localhost
    WS_PORT=2992
    WS_PATH=/api/v1

    # Data transmission rate (in seconds)
    SEND_INTERVAL=0.25

    # Logging configuration
    LOG_LEVEL=INFO
    # LOG_FILE=fsuipc_shirley_bridge.log  # Uncomment to enable file logging

    # Debug mode
    DEBUG_FSUIPC_MESSAGES=false
    ```

**Configuration Options:**

| Variable | Default | Description |
|----------|---------|-------------|
| `FSUIPC_WS_URL` | `ws://localhost:2048/fsuipc/` | FSUIPC WebSocket Server URL |
| `WS_HOST` | `localhost` | Host for the Shirley WebSocket server |
| `WS_PORT` | `2992` | Port for the Shirley WebSocket server |
| `WS_PATH` | `/api/v1` | WebSocket path (for logging only) |
| `SEND_INTERVAL` | `0.25` | Data broadcast interval in seconds (4 Hz) |
| `LOG_LEVEL` | `INFO` | Logging level: `DEBUG`, `INFO`, `WARNING`, `ERROR`, `CRITICAL` |
| `LOG_FILE` | _(none)_ | Optional path to log file. If not set, logs only to console |
| `DEBUG_FSUIPC_MESSAGES` | `false` | Enable detailed FSUIPC message debugging |

**Note:** You can also set these as environment variables directly without using a `.env` file:
```bash
export LOG_LEVEL=DEBUG
export WS_PORT=3000
python fsuipc_shirley_bridge.py
```

<br>

## 🛫 Running the Bridge

1.  Start **Microsoft Flight Simulator** and load into a flight.
2.  Ensure **FSUIPC7** is running (it should start automatically with the sim), and enable the **WebSocket Server**
3.  Execute the bridge script from your terminal:
    ```bash
    python fsuipc_shirley_bridge.py
    ```
    
Upon successful execution, you will see the following output in your terminal, confirming that both connections are active:
```
2025-01-20 15:30:00 - fsuipc_shirley_bridge - INFO - ============================================================
2025-01-20 15:30:00 - fsuipc_shirley_bridge - INFO - FSUIPC-Shirley-Bridge Configuration
2025-01-20 15:30:00 - fsuipc_shirley_bridge - INFO - ============================================================
2025-01-20 15:30:00 - fsuipc_shirley_bridge - INFO - FSUIPC WebSocket URL: ws://localhost:2048/fsuipc/
2025-01-20 15:30:00 - fsuipc_shirley_bridge - INFO - Shirley WebSocket: ws://localhost:2992/api/v1
2025-01-20 15:30:00 - fsuipc_shirley_bridge - INFO - Send Interval: 0.25s (4.0 Hz)
2025-01-20 15:30:00 - fsuipc_shirley_bridge - INFO - Debug FSUIPC Messages: False
2025-01-20 15:30:00 - fsuipc_shirley_bridge - INFO - Log Level: INFO
2025-01-20 15:30:00 - fsuipc_shirley_bridge - INFO - ============================================================
2025-01-20 15:30:01 - fsuipc_shirley_bridge - INFO - Connecting to FSUIPC at ws://localhost:2048/fsuipc/
2025-01-20 15:30:01 - fsuipc_shirley_bridge - INFO - Connected to FSUIPC (subprotocol=fsuipc)
2025-01-20 15:30:01 - fsuipc_shirley_bridge - INFO - Declared 42 FSUIPC offsets
2025-01-20 15:30:01 - fsuipc_shirley_bridge - INFO - Started reading FSUIPC offsets every 250 ms
2025-01-20 15:30:01 - fsuipc_shirley_bridge - INFO - Shirley WebSocket server listening on ws://localhost:2992/api/v1
```

<br>

## 🛠️ Technical Deep Dive

This section details the internal mechanics of the bridge.

### Core Components

The application is built around three main classes:

-   `FSUIPCWSClient`: Manages the connection to the FSUIPC WebSocket Server. It is responsible for declaring offsets, receiving raw simulator data, and sending write commands to the simulator.
-   `ShirleyWebSocketServer`: Manages connections from Shirley AI client. It broadcasts the simulator state and listens for incoming commands.
-   `SimData`: The central state manager. This class holds the latest processed data from the simulator in a structured format. It uses `asyncio.Lock` to ensure that data updates from FSUIPC and data reads for broadcasting are thread-safe, preventing race conditions.

### The Data Pipeline (Read Path)

This is the flow of data from the simulator to Shirley:

1.  **Reception**: The FSUIPC server sends a JSON payload containing raw offset values (e.g., `{"IASraw_U32": 15360}`).
2.  **Handling**: `FSUIPCWSClient._handle_incoming` receives the payload.
3.  **Lookup & Transform**: The client looks up each key (e.g., `IASraw_U32`) in the `READ_SIGNALS` dictionary. It then calls the corresponding function from the `TRANSFORMS` registry (e.g., `knots128_to_kts(15360)`), which returns a clean value (`120.0`).
4.  **State Update**: The clean value is dispatched to the `SimData` class based on its `sink` definition (e.g., `("gps", "ias_kts")`). A call is made to `sim_data.update_gps_partial(ias_kts=120.0)`.
5.  **Snapshot Assembly**: The `ShirleyWebSocketServer`'s broadcast loop calls `sim_data.get_snapshot()`. This method gathers all the latest values from its internal groups (`_gps_data`, `_att_data`, etc.) and assembles them into a single, clean JSON object conforming to the Shirley schema.
6.  **Broadcast**: The final JSON snapshot is sent to Shirley connected client.

### The Command Pipeline (Write Path)

This is the flow of commands from Shirley to the simulator:

This is the flow of commands from Shirley to the simulator:

1.  **Reception**: `ShirleyWebSocketServer.handler` receives a `SetSimData` message. Shirley sends it as a bare nested object with the same shape as the data it receives — no `type` key, no command list: `{"levers": {"landingGearHandlePercentDown": 100}}`.
2.  **Flattening**: `_flatten_set_simdata` walks the object down to its leaves, producing dotted paths such as `levers.landingGearHandlePercentDown` or `systems.batteryOn.main`.
3.  **Lookup**: Each path is looked up in the `SET_FIELDS` dictionary, which names the mechanism to use and how to encode the value. Paths that MSFS 2024 cannot set are rejected with the reason rather than silently accepted.
4.  **Execution**: `FSUIPCWSClient` dispatches by mechanism — a control event over `vars.calc`, a read-then-toggle for events that only flip, or a direct `offsets.write` for the two offsets where that is verified.
5.  **Acknowledgment**: The server replies with a `SetSimDataAck` carrying one result per field.

#### Why control events instead of offset writes

**Writing an offset that MSFS 2024 does not apply produces no error.** FSUIPC keeps the written value in its own buffer and then serves it back on every read — across disconnects, reconnects and re-declarations. One write to `0x0E8C` leaves the bridge reporting an invented outside air temperature for the rest of the session, and Shirley consumes it as flight data.

So the bridge never probes an offset to find out whether it is writable. The writable set is fixed in `WRITABLE_OFFSETS`, everything else goes through a control event, and `write_offset` refuses anything not on the list. See [`docs/FSUIPC-SETPOINT-CAPABILITIES.md`](docs/FSUIPC-SETPOINT-CAPABILITIES.md) for the field-by-field matrix and how each entry was verified.

### Key Data Structures

The bridge's behavior is primarily defined by two declarative dictionaries, making it highly extensible.

#### The `READ_SIGNALS` Dictionary

This dictionary is the heart of the data reading process. Each entry maps a custom name to an FSUIPC offset and defines how to process its data.

**Example:**
```python
"COM1_FREQ": {
    "address": 0x034E,
    "type": "uint",
    "size": 2,
    "transform": "bcd_to_freq_com_official",
    "sink": ("radios", "com1_active_khz")
},
```

-   `address`: The hexadecimal memory offset to read from FSUIPC.
-   `type`: The data type that FSUIPC should use to interpret the memory (`uint`, `int`, `float`, `string`, etc.).
-   `size`: The number of bytes to read for this offset.
-   `transform`: (Optional) The name of a function in the `TRANSFORMS` registry. This function is responsible for converting the raw value from FSUIPC into a standardized, human-readable unit.
-   `sink`: A tuple `("group", "field")` that tells the `SimData` class where to store the final, processed value. This organizes the data into logical groups for the final JSON snapshot.

#### The `SET_FIELDS` Dictionary

This dictionary defines everything Shirley can set, keyed by the exact `SetSimData` path.

**Example:**
```python
"levers.landingGearHandlePercentDown": {
    "kind": "event", "code": "{v} (>K:GEAR_SET)", "encode": _enc_pct_flag, "verified": True,
},
```

-   `kind`: the mechanism.
    -   `event` — MSFS calculator code (RPN) sent through `vars.calc`. The default and the safe one.
    -   `toggle` — the event only flips, so the bridge reads the current state from `READ_SIGNALS` first and fires only when it differs, keeping the operation idempotent.
    -   `offset` — a direct `offsets.write`, restricted to `WRITABLE_OFFSETS`.
    -   `custom` — needs its own logic (flaps by detent, the `altitudeMode` enum, Zulu time).
-   `code`: the calculator code. `{v}` is replaced with the encoded value; `code_off` gives the falsy-value variant where one exists.
-   `encode`: converts the schema's units into the event's parameter — percent to `0…16383`, kHz to 4-digit BCD, inches of mercury to millibars × 16.
-   `state`: for `toggle`, which `(group, field)` holds the current value.
-   `verified`: whether that mechanism was exercised against a running MSFS 2024 with a mirror confirming the simulator applied it.

Fields the schema defines but MSFS 2024 cannot set live in `SET_FIELDS_UNSUPPORTED`, each with its reason, so a client gets an explanation instead of silence.

### The Transformation Layer

FSUIPC often provides data in raw, encoded, or scaled formats. The transformation layer is a collection of Python functions, registered in the `TRANSFORMS` dictionary, designed to decode this data into clean, usable formats.

**Types of Transformations:**

-   **Simple Scaling**: Many values are integers that need to be divided by a factor.
    > `knots128_to_kts(raw)` simply returns `float(raw) / 128.0`.

-   **Unit Conversion**: These functions perform complex conversions involving multiple constants.
    > `vs_raw_to_fpm(raw)` converts a value from (meters/second * 256) to feet/minute using `SECONDS_PER_MINUTE`, `METERS_TO_FEET`, and the FSUIPC scaling factor.

-   **BCD Decoding**: Radio frequencies and transponder codes are often stored in Binary Coded Decimal (BCD) format.
    > `bcd_to_freq_com_official(raw)` meticulously extracts 4-bit "nibbles" from the raw integer and reconstructs a radio frequency. For example, it converts the hex value `0x2345` into the frequency `123.45` MHz.

-   **Bitfield Processing**: Some offsets, like `0x0D0C` for lights, use individual bits of an integer as on/off flags. The bridge handles this by performing bitwise `AND` operations to check the status of each light.
    > `nav_on = bool(raw_value & (1<<0))` checks if the first bit is set.

-   **Derived Values**: Some data points are not read directly but are calculated from more than one offset.
    > The altimeter setting prefers `0x0330` (altimeter 1) and falls back to `0x0332` (the second altimeter on a twin-altimeter panel) only when the first is out of plausible range. AGL comes from MSL altitude minus ground altitude, and magnetic heading from true heading minus magnetic variation.

-   **No fabricated values**: a transform returns `None` when the raw value is out of range, and the field is then omitted from the snapshot rather than filled with a plausible-looking constant.

<br>

## 📡 Shirley WebSocket API

The bridge exposes a WebSocket server endpoint at `ws://localhost:2992/api/v1`.

**On Connection:**
The server immediately sends a `Capabilities` message:
```json
{
  "type": "Capabilities",
  "reads": [
    {"key": "LatitudeDeg", "group": "gps", "field": "latitude"},
    ...
  ],
  "writes": ["autopilot.altitudeBugFt", "levers.landingGearHandlePercentDown", ...]
}
```

**Data Broadcasts:**
The server sends JSON snapshots at the `SEND_INTERVAL` rate. The structure matches the Shirley schema:
```json
{
  "position": {
    "latitudeDeg": 40.7128,
    "longitudeDeg": -74.0060,
    "mslAltitudeFt": 1500.5
  },
  "attitude": {
    "trueHeadingDeg": 359.8,
    "pitchAngleDegUp": 2.5
  },
  "levers": {
      "throttlePercentOpen": {"engine1": 85.0}
  },
  ...
}
```

**Receiving Commands:**
Clients send `SetSimData` as a bare nested object — the same shape as the data they receive, with no wrapper. Each message is acknowledged with a `SetSimDataAck` carrying one result per field.
```json
// Client sends:
{
  "levers": {"landingGearHandlePercentDown": 100},
  "autopilot": {"altitudeBugFt": 5000}
}

// Server responds:
{
  "type": "SetSimDataAck",
  "results": [
    {"field": "levers.landingGearHandlePercentDown", "ok": true},
    {"field": "autopilot.altitudeBugFt", "ok": true}
  ]
}
```

A field the bridge cannot set comes back with the reason:
```json
{"field": "environment.groundTemperatureDegC", "ok": false,
 "error": "el clima no se puede fijar en MSFS 2024: las escrituras se aceptan y se devuelven en el eco, pero el simulador nunca las aplica"}
```

<br>

## 🐛 Troubleshooting

**Bridge won't connect to FSUIPC:**
-   **Verify FSUIPC WebSocket Server is enabled:** Double-check the settings in FSUIPC7.
-   **Check Firewall:** Ensure your firewall is not blocking connections on port `2048`.
-   **Confirm MSFS and FSUIPC are running:** The bridge can only connect when the simulator and FSUIPC are active.

**No data is being sent to Shirley:**
-   **Check Client Connection:** Ensure your client is successfully connected to `ws://localhost:2992/api/v1`.
-   **Enable Debug Mode:** In `fsuipc_shirley_bridge.py`, set `DEBUG_FSUIPC_MESSAGES = True`. This will print all incoming FSUIPC messages and outgoing Shirley snapshots to the console, helping you diagnose data flow issues.
-   **Verify `Capabilities` Message:** Your client should receive the capabilities message upon connecting. If not, the connection may not be properly established.

**A specific value (e.g., lights, autopilot state) is not updating:**
-   **Aircraft Compatibility:** Some third-party aircraft may use non-standard FSUIPC offsets. The offsets defined in this script are based on standard conventions and may need to be adjusted for specific add-ons.
-   **Check In-Sim Systems:** Ensure the relevant aircraft systems are powered. For example, lights will not report a status if the main battery is off.

<br>

## 🙏 Acknowledgments

-   **Juan Luis Gabriel** - *Author*
-   **[Airplane Team](https://airplane.team/)** - For creating the visionary Shirley AI.
-   **[Shirley Sim Interface Schema](https://github.com/Airplane-Team/sim-interface)** - For providing the official schema specification that makes this integration possible.
-   **Pete Dowson** - The original creator and developer of the indispensable FSUIPC.
-   **Paul Henty** - For developing the FSUIPC WebSocket Server that enables modern integrations like this one.
-   **Microsoft & Asobo Studio** - For creating the incredible Microsoft Flight Simulator platform.

<br>

## 📄 License

This project is licensed under the MIT License.

Copyright (c) 2025 Juan Luis Gabriel

---

***Ready to fly with an AI copilot? 🛩️✨***












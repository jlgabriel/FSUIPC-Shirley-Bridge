# FSUIPC Shirley Bridge

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python Version](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Websockets](https://img.shields.io/badge/library-websockets-green.svg)](https://websockets.readthedocs.io/)

**A high-performance WebSocket bridge that connects FSUIPC7 with Shirley AI for Microsoft Flight Simulator 2020/2024. This bridge enables real-time, bidirectional communication, streaming rich flight data to Shirley and translating its commands into in-sim actions.**

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
    - [The `WRITE_COMMANDS` Dictionary](#the-write_commands-dictionary)
  - [The Transformation Layer](#the-transformation-layer)
- [Shirley WebSocket API](#shirley-websocket-api)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)
- [Acknowledgments](#-acknowledgments)
- [License](#-license)

<br>

## ✨ Features

This bridge provides a comprehensive set of features to ensure seamless integration between the simulator and AI.

### Real-time Data Streaming (4Hz)
-   **Position & Navigation**: Full GPS data including latitude, longitude, MSL/AGL altitude, indicated airspeed (IAS), ground speed, and vertical speed.
-   **Attitude**: Precise attitude information such as true/magnetic heading, pitch, and roll angles.
-   **Aircraft Systems**: Status of main battery, pitot heat, brakes (pedal and parking), and more.
-   **Lights**: Real-time status of navigation, landing, taxi, and strobe lights, decoded from FSUIPC bitmasks.
-   **Control Surfaces & Levers**: Percentage-based positions for flaps, landing gear, throttle, mixture, and propeller levers.
-   **Engine Indicators**: Key metrics like RPM, N1%, EGT, CHT, and manifold pressure for multiple engines.
-   **Radios & Navigation**: Active and standby frequencies for COM/NAV radios and the current transponder code.
-   **Autopilot**: Full autopilot status, including engagement state, active modes (HDG, ALT, VS), and bug settings.
-   **Environment**: Ambient conditions like wind speed, direction, and outside air temperature.

### Bidirectional Aircraft Control
-   **Landing Gear**: Command the gear handle to be raised or lowered.
-   **Engine Controls**: Set throttle position with precision, from -100% to +100%.
-   **Extensible Command System**: The architecture is designed to easily add new writable commands for any FSUIPC-controllable system.

### Advanced Capabilities
-   **Dynamic Capabilities Reporting**: Automatically informs connecting clients (like Shirley) of all readable data points and writable commands upon connection.
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
-   **FSUIPC7**: The registered version is required for WebSocket server functionality.
-   **Python 3.8+**
-   The `websockets` Python library.

### Installation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/yourusername/fsuipc-shirley-bridge.git
    cd fsuipc-shirley-bridge
    ```

2.  **Install the required Python library:**
    ```bash
    pip install websockets
    ```

3.  **Configure FSUIPC WebSocket Server:**
    *   In the FSUIPC7 settings dialog within MSFS, navigate to the "WebSocket Server" tab.
    *   Check the box to **Enable WebSocket Server**.
    *   Set the server port to `2048` (this is the default expected by the script).
    *   Ensure that `localhost` access is permitted.

### Configuration

The script's primary settings can be adjusted at the top of `fsuipc_shirley_bridge.py`:

```python
# ===================== CONFIGURATION =====================
FSUIPC_WS_URL = "ws://localhost:2048/fsuipc/"  # FSUIPC WebSocket Server URL
WS_HOST = "localhost"                         # Host for the Shirley server
WS_PORT = 2992                                # Port for the Shirley server
SEND_INTERVAL = 0.25                          # Data broadcast rate in seconds (0.25 = 4 Hz)
DEBUG_FSUIPC_MESSAGES = False                 # Set to True for verbose console logging
```

<br>

## 🛫 Running the Bridge

1.  Start **Microsoft Flight Simulator** and load into a flight.
2.  Ensure **FSUIPC7** is running (it should start automatically with the sim).
3.  Execute the bridge script from your terminal:
    ```bash
    python fsuipc_shirley_bridge.py
    ```
4.  Connect your Shirley AI client or any other compatible WebSocket client to the bridge's endpoint: `ws://localhost:2992/api/v1`.

Upon successful execution, you will see the following output in your terminal, confirming that both connections are active:
```
[FSUIPCWS] Connected ws://localhost:2048/fsuipc/ (subprotocol=fsuipc)
[FSUIPCWS] Offsets declared
[FSUIPCWS] Started reading offsets every 250 ms
[ShirleyWS] Serving at ws://localhost:2992/api/v1
```

<br>

## 🛠️ Technical Deep Dive

This section details the internal mechanics of the bridge.

### Core Components

The application is built around three main classes:

-   `FSUIPCWSClient`: Manages the connection to the FSUIPC WebSocket Server. It is responsible for declaring offsets, receiving raw simulator data, and sending write commands to the simulator.
-   `ShirleyWebSocketServer`: Manages connections from Shirley AI clients. It broadcasts the simulator state and listens for incoming commands.
-   `SimData`: The central state manager. This class holds the latest processed data from the simulator in a structured format. It uses `asyncio.Lock` to ensure that data updates from FSUIPC and data reads for broadcasting are thread-safe, preventing race conditions.

### The Data Pipeline (Read Path)

This is the flow of data from the simulator to Shirley:

1.  **Reception**: The FSUIPC server sends a JSON payload containing raw offset values (e.g., `{"IASraw_U32": 15360}`).
2.  **Handling**: `FSUIPCWSClient._handle_incoming` receives the payload.
3.  **Lookup & Transform**: The client looks up each key (e.g., `IASraw_U32`) in the `READ_SIGNALS` dictionary. It then calls the corresponding function from the `TRANSFORMS` registry (e.g., `knots128_to_kts(15360)`), which returns a clean value (`120.0`).
4.  **State Update**: The clean value is dispatched to the `SimData` class based on its `sink` definition (e.g., `("gps", "ias_kts")`). A call is made to `sim_data.update_gps_partial(ias_kts=120.0)`.
5.  **Snapshot Assembly**: The `ShirleyWebSocketServer`'s broadcast loop calls `sim_data.get_snapshot()`. This method gathers all the latest values from its internal groups (`_gps_data`, `_att_data`, etc.) and assembles them into a single, clean JSON object conforming to the Shirley schema.
6.  **Broadcast**: The final JSON snapshot is sent to all connected Shirley clients.

### The Command Pipeline (Write Path)

This is the flow of commands from Shirley to the simulator:

1.  **Reception**: The `ShirleyWebSocketServer.handler` receives a `SetSimData` JSON message (e.g., `{"type": "SetSimData", "commands": [{"name": "GEAR_HANDLE", "value": 1}]}`).
2.  **Handling**: The server iterates through the commands and calls `_handle_command` for each.
3.  **Lookup & Encode**: The command name (`GEAR_HANDLE`) is looked up in the `WRITE_COMMANDS` dictionary. The `encode` lambda function is executed with the provided value (`lambda v: 16383 if int(float(v)) else 0`), converting the simple `1` into the raw integer `16383` that FSUIPC expects.
4.  **Forwarding**: The encoded command is passed to `fsuipc.write_offset`, which constructs a final JSON payload for FSUIPC.
5.  **Execution**: The `FSUIPCWSClient` sends the write command to the FSUIPC server, which then modifies the simulator's memory, causing the gear handle to move.
6.  **Acknowledgment**: The Shirley server sends a `SetSimDataAck` message back to the client to confirm the command was processed.

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

#### The `WRITE_COMMANDS` Dictionary

This dictionary defines all the actions that Shirley can command the aircraft to perform.

**Example:**
```python
"GEAR_HANDLE": {
    "type": "offset",
    "address": 0x0BE8, "size": 4, "dtype": "int",
    "encode": lambda v: 16383 if int(float(v)) else 0,
},
```
-   `type`: The command type. Currently, `offset` is used for direct memory writes.
-   `address`, `size`, `dtype`: Define the target memory location, size, and data type for the write operation.
-   `encode`: A crucial `lambda` function that translates a high-level, user-friendly value (e.g., `1` for "down", `0` for "up") into the raw numeric value that FSUIPC requires to perform the action. This creates a powerful abstraction layer.

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

-   **Derived Values**: Some data points are not read directly but are calculated from multiple offsets.
    > The `brakes_on` status is logically derived by checking the values of the left brake, right brake, and parking brake offsets against predefined thresholds.

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
  "writes": ["GEAR_HANDLE", "throttle", ...]
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
Clients can send a `SetSimData` message to control the aircraft. The command is acknowledged with a `SetSimDataAck`.
```json
// Client sends:
{
  "type": "SetSimData",
  "commands": [
    {"name": "GEAR_HANDLE", "value": 1} // 1 for down
  ]
}

// Server responds:
{
  "type": "SetSimDataAck",
  "results": [
    {"name": "GEAR_HANDLE", "ok": true}
  ]
}
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

## 🤝 Contributing

Contributions are welcome! If you would like to contribute, please follow these steps:

1.  Fork the repository.
2.  Create a new branch (`git checkout -b feature/AmazingFeature`).
3.  Make your changes.
4.  Commit your changes (`git commit -m 'Add some AmazingFeature'`).
5.  Push to the branch (`git push origin feature/AmazingFeature`).
6.  Open a Pull Request.

Please ensure your code follows a similar style and that you document any new functionality. Adding or improving FSUIPC offsets and transformations is a great way to contribute.

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

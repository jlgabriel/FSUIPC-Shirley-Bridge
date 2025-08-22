# FSUIPC Shirley Bridge

*By Juan Luis Gabriel*

A high-performance WebSocket bridge that connects **FSUIPC WebSocket Server** with **Shirley AI** for Microsoft Flight Simulator 2020/2024. This bridge enables real-time flight data streaming and aircraft control commands, allowing Shirley to have rich, context-aware conversations about your flight.

## ✈️ Features

### Real-time Data Streaming (4Hz)
- **Position & Navigation**: GPS coordinates, altitude (MSL/AGL), airspeeds, ground track
- **Attitude**: Heading (true/magnetic), pitch, roll with wind drift calculations  
- **Aircraft Lights**: Navigation, landing, taxi, strobe, beacon lights status
- **Extensible Architecture**: Ready for autopilot, systems, levers, and indicators

### Aircraft Control
- **Landing Gear**: Raise/lower gear handle
- **Engine Controls**: Throttle management (-100% to +100%)
- **Expandable**: Architecture supports any FSUIPC-controllable system

### Advanced Capabilities
- **Ground Track Calculation**: True bearing over ground (separate from aircraft heading)
- **Magnetic Variation**: Automatic true-to-magnetic heading conversion
- **Bitfield Processing**: Efficient extraction of multiple signals from single FSUIPC offsets
- **Dynamic Capabilities**: Auto-generated signal lists for client discovery

## 🚀 Quick Start

### Prerequisites
- **Microsoft Flight Simulator 2020/2024**
- **FSUIPC7** with WebSocket Server enabled
- **Python 3.8+** with `websockets` and `asyncio`
- **Shirley AI** or compatible WebSocket client

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/yourusername/fsuipc-shirley-bridge.git
   cd fsuipc-shirley-bridge
   ```

2. **Install dependencies**
   ```bash
   pip install websockets
   ```

3. **Configure FSUIPC WebSocket Server**
   - Enable WebSocket Server in FSUIPC7 settings
   - Set port to `2048` (default)
   - Ensure `localhost` access is allowed

### Running the Bridge

1. **Start MSFS and FSUIPC7**
2. **Run the bridge**
   ```bash
   python fsuipc_shirley_bridge.py
   ```
3. **Connect Shirley** to `ws://localhost:2992/api/v1`

You should see:
```
[FSUIPCWS] Connected ws://localhost:2048/fsuipc/ (subprotocol=fsuipc)
[FSUIPCWS] Offsets declared
[FSUIPCWS] Started reading offsets every 250 ms
[ShirleyWS] Serving at ws://localhost:2992/api/v1
```

## 📡 Data Output

### JSON Stream Example
```json
{
  "position": {
    "latitudeDeg": 40.758896,
    "longitudeDeg": -73.985130,
    "mslAltitudeFt": 5000.0,
    "aglAltitudeFt": 4987.2,
    "indicatedAirspeedKts": 120.5,
    "gpsGroundSpeedKts": 118.3,
    "verticalSpeedUpFpm": 850.0
  },
  "attitude": {
    "trueHeadingDeg": 85.5,
    "magneticHeadingDeg": 83.2,
    "trueGroundTrackDeg": 87.1,
    "pitchAngleDegUp": 5.3,
    "rollAngleDegRight": -2.1
  },
  "lights": {
    "navigationLightsSwitchOn": true,
    "landingLightsSwitchOn": false,
    "strobeLightsSwitchOn": true,
    "beaconLightsSwitchOn": false,
    "taxiLightsSwitchOn": false
  }
}
```

### Dynamic Capabilities
The bridge automatically advertises available signals:
```json
{
  "type": "Capabilities",
  "reads": [
    "position.latitudeDeg",
    "attitude.trueHeadingDeg", 
    "lights.navigationLightsSwitchOn",
    "..."
  ],
  "writes": [
    "GEAR_HANDLE",
    "throttle"
  ]
}
```

## 🎮 Aircraft Control

### Sending Commands
```json
{
  "type": "SetSimData",
  "commands": [
    {"name": "GEAR_HANDLE", "value": 1},
    {"name": "throttle", "value": 0.75}
  ]
}
```

### Available Commands
| Command | Description | Values |
|---------|-------------|---------|
| `GEAR_HANDLE` | Landing gear | `0` = up, `1` = down |
| `throttle` | Engine throttle | `-1.0` to `1.0` or raw values |

## 🧪 Testing in MSFS

### Light Controls
- **Navigation Lights**: `L` key
- **Landing Lights**: `Ctrl+L` 
- **Strobe Lights**: `O` key
- **Beacon Lights**: `Shift+O`
- **Taxi Lights**: `Shift+Ctrl+L`

### Expected Behavior
1. **Lights off**: No `lights` group in JSON
2. **Toggle any light**: `lights` group appears with current states
3. **Ground movement**: `trueGroundTrackDeg` shows actual movement direction
4. **Wind conditions**: Ground track ≠ aircraft heading (shows drift)

## 🏗️ Architecture

### Modular Design
```
FSUIPC WebSocket ←→ FSUIPCWSClient ←→ SimData ←→ ShirleyWebSocketServer ←→ Shirley AI
```

### Key Components
- **FSUIPCWSClient**: Connects to FSUIPC, handles offset reading/writing
- **SimData**: Central data model with partial updates and derived calculations  
- **ShirleyWebSocketServer**: Serves data to clients, handles commands
- **Sink Maps**: Declarative mapping from FSUIPC signals to Shirley schema

### Extensible Signal Processing
```python
READ_SIGNALS = {
    "LIGHT_NAV": {
        "address": 0x0D0C, 
        "type": "ushort", 
        "size": 2,
        "transform": "bit0_to_bool", 
        "sink": ("lights", "nav_on")
    }
}
```

## 🔧 Configuration

### Bridge Settings
```python
FSUIPC_WS_URL = "ws://localhost:2048/fsuipc/"  # FSUIPC WebSocket Server
WS_HOST = "localhost"                          # Shirley server host  
WS_PORT = 2992                                 # Shirley server port
SEND_INTERVAL = 0.25                           # 4Hz update rate
```

### Debug Mode
```python
DEBUG_FSUIPC_MESSAGES = True  # Enable verbose FSUIPC logging
```

## 🛠️ Development

### Adding New Signal Groups
1. **Define sink mapping**:
   ```python
   _SYSTEMS_SINK_TO_SHIRLEY = {
       "pitot_heat_on": "systems.pitotHeatSwitchOn"
   }
   ```

2. **Add offsets to READ_SIGNALS**:
   ```python
   "PITOT_HEAT": {"address": 0x029C, "type": "byte", "size": 1, "sink": ("systems", "pitot_heat_on")}
   ```

3. **Update dispatcher** in `_handle_incoming()`

The architecture handles the rest automatically!

### Adding Write Commands
```python
WRITE_COMMANDS = {
    "FLAPS_SET": {
        "type": "offset",
        "address": 0x0BDC, 
        "size": 4, 
        "dtype": "int",
        "encode": lambda v: int(float(v) * 163.83)  # 0-100% to 0-16383
    }
}
```

## 📋 Roadmap

### Planned Expansions
- **Systems Group**: Pitot heat, brakes, battery status
- **Autopilot Group**: AP engage, heading/altitude bugs, vertical speed
- **Levers Group**: Flaps position, gear status, throttle feedback
- **Indicators Group**: Altimeter setting, stall warning, engine parameters
- **Environment Group**: Wind data, weather conditions
- **Radio Group**: COM/NAV frequencies, transponder

### Advanced Features
- **Configuration Files**: YAML-based signal customization
- **Performance Metrics**: Latency monitoring, connection health
- **Auto-discovery**: Automatic FSUIPC detection
- **Executable Packaging**: Standalone .exe distribution

## 🐛 Troubleshooting

### Common Issues

**Bridge won't connect to FSUIPC**
- Ensure FSUIPC7 WebSocket Server is enabled and running
- Check firewall settings for port 2048
- Verify MSFS is running with FSUIPC loaded

**No data in Shirley**
- Check WebSocket connection to `ws://localhost:2992/api/v1`
- Verify Capabilities message is received
- Enable debug mode: `DEBUG_FSUIPC_MESSAGES = True`

**Lights not updating**
- Confirm aircraft has working electrical system
- Try different aircraft (some have non-standard light implementations)
- Check FSUIPC offset compatibility for your aircraft

### Debug Logging
```
[FSUIPCWS] Valid data fields: 15/18        # Signal quality
[WARNING] Invalid position data detected   # Data validation
[CMD] GEAR_HANDLE -> 1 (raw=16383) ok=True # Command execution
```

## 📄 License

MIT License

Copyright (c) 2025 Juan Luis Gabriel

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## 🤝 Contributing

Contributions welcome! Please read the development section for architecture details.

This bridge implements the [Shirley Sim Interface Schema](https://github.com/Airplane-Team/sim-interface) for compatibility with Shirley AI and other flight simulation tools.

## 🙏 Acknowledgments

- **[Airplane Team](https://airplane.team/)** - Creators of Shirley AI
- **[Shirley Sim Interface Schema](https://github.com/Airplane-Team/sim-interface)** - Official schema specification
- **Pete Dowson** - FSUIPC development
- **Paul Henty** - FSUIPC WebSocket Server
- **Microsoft** - Flight Simulator 2020/2024

---

**Ready to fly with AI copilot conversations?** 🛩️✨

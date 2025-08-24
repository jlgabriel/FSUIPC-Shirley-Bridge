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

MIT License - Copyright (c) 2025 Juan Luis Gabriel

## 🙏 Acknowledgments

- **[Airplane Team](https://airplane.team/)** - Creators of Shirley AI
- **[Shirley Sim Interface Schema](https://github.com/Airplane-Team/sim-interface)** - Official schema specification
- **Pete Dowson** - FSUIPC development
- **Paul Henty** - FSUIPC WebSocket Server
- **Microsoft** - Flight Simulator 2020/2024

---

**Ready to fly with AI copilot conversations?** 🛩️✨

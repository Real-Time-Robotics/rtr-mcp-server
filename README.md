````markdown
# MCP Server

A lightweight wrapper around DroneKit that exposes vehicle APIs as asynchronous MCP tools over HTTP or messaging.

## 📖 Overview

`mcp_server.py` connects to a DroneKit `Vehicle` and registers a set of async functions decorated with `@mcp.tool()`. This lets clients call common flight-controller operations through a simple interface.

## 🚀 Installation

1. Clone or download this repository.  
2. Install dependencies:
   ```bash
   pip install dronekit pymavlink aiohttp
````

## ⚙️ Configuration

* By default, the server reads the MAVLink connection string from the `MAVLINK_URL` environment variable (e.g. `udp:127.0.0.1:14550`).
* You can also hardcode or override it in `mcp_server.py` if needed.

## ▶️ Running

```bash
python mcp_server.py
```

The server will listen on `http://0.0.0.0:8080` by default.

## 📡 Available MCP Tools

| Tool                                       | Parameters                               | Description                                                     |
| ------------------------------------------ | ---------------------------------------- | --------------------------------------------------------------- |
| `arm()`                                    | —                                        | Arm or disarm the vehicle.                                      |
| `switch_mode(mode: str)`                   | `mode` — flight mode name                | Change flight mode (e.g. `"GUIDED"`, `"LOITER"`, `"RTL"`).      |
| `status()`                                 | —                                        | Return current connection status and vehicle state.             |
| `takeoff(alt: float)`                      | `alt` — relative altitude (meters)       | Command the vehicle to take off.                                |
| `goto(lat, lon, alt: float)`               | `lat`, `lon`, `alt` — global target      | Fly to the specified global coordinate.                         |
| `land()`                                   | —                                        | Command the vehicle to land.                                    |
| `set_home(lat, lon, alt)`                  | `lat`, `lon`, `alt` — home location      | Set the vehicle’s home location.                                |
| `home_location()`                          | —                                        | Get the current home location.                                  |
| `location_global()`                        | —                                        | Get the vehicle’s global position.                              |
| `location_global_relative()`               | —                                        | Get the vehicle’s global position with relative altitude.       |
| `distance_home()`                          | —                                        | Compute distance from current location to home.                 |
| `get_attitude()`                           | —                                        | Read vehicle attitude (roll, pitch, yaw).                       |
| `get_battery()`                            | —                                        | Read battery status (voltage, current, level).                  |
| `get_rangefinder()`                        | —                                        | Read rangefinder distance.                                      |
| `get_gimbal_attitude()`                    | —                                        | Read gimbal orientation.                                        |
| `set_gimbal_orientation(pitch, roll, yaw)` | `float`                                  | Point the gimbal to the specified orientation.                  |
| `release_gimbal()`                         | —                                        | Release control of the gimbal.                                  |
| `get_gps_info()`                           | —                                        | Return GPSInfo: `eph`, `epv`, `fix_type`, `satellites_visible`. |
| `list_parameters()`                        | —                                        | List all vehicle parameters.                                    |
| `get_parameter(name: str)`                 | `name` — parameter key                   | Get the value of a specific parameter.                          |
| `set_parameter(name: str, value)`          | `name`, `value`                          | Set a specific vehicle parameter.                               |
| `get_capabilities()`                       | —                                        | Query autopilot capabilities.                                   |
| **Mission (Waypoints)**                    |                                          |                                                                 |
| `get_command_sequence()`                   | —                                        | Download and return current mission commands.                   |
| `clear_all_mission()`                      | —                                        | Clear all mission waypoints.                                    |
| `create_command(cmds: list[dict])`         | `cmds` — list of `{type, lat, lon, alt}` | Create or append mission commands.                              |

## 🔧 Extending

To add a new tool:

1. Define an async function in `mcp_server.py`.
2. Decorate it with `@mcp.tool()`, `@mcp.resource()`, or `@mcp.prompt()`.
3. Use the global `vehicle` object and `mavutil` to implement the logic.
4. Restart the server.

## 🤝 Contributing

1. Fork the repo.
2. Create a feature branch or bugfix.
3. Submit a pull request with clear description.
4. Ensure new tools include docstrings and basic error handling.

## 📄 License

MIT © Real-Time Robotics

import collections
import _collections_abc
collections.MutableMapping = _collections_abc.MutableMapping
import os, time
from dotenv import load_dotenv
from dronekit import connect, VehicleMode, LocationGlobalRelative
from mcp.server.fastmcp import FastMCP
from dronekit_sitl import start_default

# 1. Load configuration from .env
load_dotenv()
sitl = start_default()
DRONE_CONN = sitl.connection_string()

# 2. Connect DroneKit
print(f"Connecting to drone at {DRONE_CONN}...")
vehicle = connect(DRONE_CONN, wait_ready=True, heartbeat_timeout=30)
print(f"Connected. Mode: {vehicle.mode.name}")

# 3. Initial MCP server
mcp = FastMCP("drone-mcp")

# 4. Register tools
@mcp.tool()
async def arm() -> str:
    """Arms the drone in GUIDED mode."""
    if not vehicle.is_armable:
        return "Vehicle not armable"
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    deadline = time.time() + 10
    while not vehicle.armed and time.time() < deadline:
        time.sleep(0.5)
    return "ARMED" if vehicle.armed else "Arm timeout"

# @mcp.tool()
# async def takeoff(altitude: int) -> dict[str, float]:
#     """Take off to the given altitude (meters)."""
#     vehicle.simple_takeoff(altitude)
#     while True:
#         alt = vehicle.location.global_relative_frame.alt
#         if alt >= altitude * 0.95:
#             break
#         time.sleep(0.5)
#     return {"target": altitude, "achieved": alt}

# @mcp.tool()
# async def goto(latitude: float, longitude: float, altitude: float) -> dict[str, float]:
#     """Fly to specified GPS coordinates."""
#     loc = LocationGlobalRelative(latitude, longitude, altitude)
#     vehicle.simple_goto(loc)
#     return {"lat": latitude, "lon": longitude, "alt": altitude}
#
# @mcp.tool()
# async def land() -> str:
#     """Land the drone."""
#     vehicle.mode = VehicleMode("LAND")
#     return "LANDING"
#
@mcp.tool()
async def status() -> dict:
    """Get current vehicle status."""
    return {
        "battery": vehicle.battery,
        "location": {
            "lat": vehicle.location.global_frame.lat,
            "lon": vehicle.location.global_frame.lon,
            "alt": vehicle.location.global_relative_frame.alt
        },
        "mode": vehicle.mode.name,
        "armed": vehicle.armed
    }

# 5. Run server through SSE transport
if __name__ == "__main__":
    mcp.run()

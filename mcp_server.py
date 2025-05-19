import _collections_abc
import collections
from typing import Any, Coroutine

collections.MutableMapping = _collections_abc.MutableMapping
import time, os, math, logging
from dotenv import load_dotenv
from dronekit import connect, VehicleMode, LocationGlobalRelative
from mcp.server.fastmcp import FastMCP
from pymavlink import mavutil

# 1. Load configuration from .env
load_dotenv()
logger = logging.getLogger("mcp-drone")
DRONE_CONN = os.getenv("DRONE_CONN")
if not DRONE_CONN:
    raise RuntimeError("Please set DRONE_CONN env var to your autopilot endpoint, e.g. tcp:13.213.147.174:5762")

vehicle = connect(DRONE_CONN, wait_ready=True)
print(f"Connected. Mode: {vehicle.mode.name}")

# 2. Initial MCP server
mcp = FastMCP(
    name="drone-mcp",
)

def haversine(lat1, lon1, lat2, lon2):
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat/2)**2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2)
    return 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

# 3. Register tools
@mcp.tool()
async def arm() -> str:
    """Arms the drone in GUIDED mode."""
    if not vehicle.is_armable:
        return "Vehicle not armable"
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    deadline = time.time() + 10
    while not vehicle.armed and time.time() < deadline:
        time.sleep(1)
    return "ARMED" if vehicle.armed else "Arm timeout"

@mcp.tool()
async def set_home() -> dict[str, float]:
    """Set the current position as home origin for relative altitude."""
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        0, 0, 0, 0,
        vehicle.location.global_frame.lat,
        vehicle.location.global_frame.lon,
        vehicle.location.global_frame.alt
    )
    vehicle.send_mavlink(msg)
    time.sleep(1)
    home = vehicle.home_location
    return {"home_lat": home.lat, "home_lon": home.lon, "home_alt": home.alt}

@mcp.tool()
async def takeoff(altitude: float) -> dict[str, float | Any]:
    """Take off to the given altitude (meters)."""
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(altitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from the function just below the target altitude.
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
    return {"Target: ": altitude, "Current: ": vehicle.location.global_relative_frame.alt}

@mcp.tool()
async def goto(latitude: float, longitude: float, altitude: float) -> dict[str, str] | dict[str, float | Any]:
    """Fly to specified GPS coordinates."""
    vehicle.airspeed = 3
    # Ensure GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    deadline = time.time() + 10
    while vehicle.mode.name != "GUIDED" and time.time() < deadline:
        time.sleep(1)
    if vehicle.mode.name != "GUIDED":
        return {"error": "Failed to set GUIDED mode"}

    # Compute absolute altitude (MSL) for target
    home = vehicle.home_location
    if home and home.alt is not None:
        abs_alt = home.alt + altitude
    else:
        # fallback to current MSL alt + relative alt
        abs_alt = vehicle.location.global_frame.alt + altitude

    target = LocationGlobalRelative(latitude, longitude, abs_alt)
    vehicle.simple_goto(target)

    # Wait until within 1m horizontally and 0.5m vertically
    start = time.time()
    while True:
        curr_lat = vehicle.location.global_frame.lat
        curr_lon = vehicle.location.global_frame.lon
        dist = haversine(curr_lat, curr_lon, latitude, longitude)
        curr_rel_alt = vehicle.location.global_relative_frame.alt
        vert_err = abs(curr_rel_alt - altitude)
        logger.info(f"Distance: {dist:.1f} m, Vert error: {vert_err:.1f} m")
        if dist <= 1.0 and vert_err <= 0.5:
            break
        # if time.time() - start > 60:
        #     return {"error": "Timeout reaching target"}
        time.sleep(1)

    return {
        "lat": latitude,
        "lon": longitude,
        "target_rel": altitude,
        "achieved_dist": dist,
        "achieved_rel": curr_rel_alt
    }

@mcp.tool()
async def land() -> str:
    """Land the drone."""
    vehicle.mode = VehicleMode("LAND")
    return "LANDING"

@mcp.tool()
async def status() -> dict:
    """Get current vehicle status."""
    return {
        "battery": vehicle.battery,
        "location": {
            "lat": vehicle.location.global_relative_frame.lat,
            "lon": vehicle.location.global_relative_frame.lon,
            "alt": vehicle.location.global_relative_frame.alt
        },
        "mode": vehicle.mode.name,
        "armed": vehicle.armed,
        "system_status": vehicle.system_status,
    }

# 5. Run server through SSE transport
if __name__ == "__main__":
    # mcp.run(transport='sse')
    mcp.run()
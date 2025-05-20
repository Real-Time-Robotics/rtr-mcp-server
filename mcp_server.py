import _collections_abc
import collections
from typing import Any

collections.MutableMapping = _collections_abc.MutableMapping
import time, os, math
from dotenv import load_dotenv
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, LocationGlobal, Command
from mcp.server.fastmcp import FastMCP
from pymavlink import mavutil

# 1. Load configuration from .env
load_dotenv()
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
async def takeoff(altitude: float | int) -> dict[str, float | Any]:
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
        current_alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {current_alt}")
        # Break and return from the function just below the target altitude.
        if current_alt >= altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
    return {"Target": altitude, "Current": current_alt}

@mcp.tool()
async def goto(latitude: float, longitude: float, altitude: float) -> dict[str, str] | dict[str, float | Any]:
    """Fly to specified GPS coordinates."""
    vehicle.airspeed = 5
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
        abs_alt = altitude
    else:
        # fallback to current MSL alt + relative alt
        abs_alt = altitude

    target = LocationGlobalRelative(latitude, longitude, abs_alt)
    vehicle.simple_goto(target)

    # Wait until within 1 m horizontally and 0.5 m vertically
    while True:
        curr_lat = vehicle.location.global_frame.lat
        curr_lon = vehicle.location.global_frame.lon
        dist = haversine(curr_lat, curr_lon, latitude, longitude)
        curr_rel_alt = vehicle.location.global_relative_frame.alt
        vert_err = abs(curr_rel_alt - altitude)
        if dist <= 1.0 and vert_err <= 0.5:
            break
        time.sleep(30)

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

@mcp.tool()
async def api_exception_info() -> str:
    """Return the APIException class name and docstring."""
    return APIException.__doc__ or APIException.__name__

@mcp.tool()
async def get_attitude() -> dict[str, float]:
    """Get current vehicle attitude: pitch, yaw, roll (radians)."""
    att = vehicle.attitude
    return {f"pitch": att.pitch, "yaw": att.yaw, "roll": att.roll}

@mcp.tool()
async def get_capabilities() -> dict[str, bool]:
    """Get vehicle capabilities."""
    caps = vehicle.capabilities
    return {attr: getattr(caps, attr) for attr in vars(caps) if not attr.startswith('_')}

@mcp.tool()
async def get_channel_overrides() -> dict[str, int]:
    """Get current RC channel overrides."""
    return vehicle.channels.overrides.copy()

@mcp.tool()
async def get_battery() -> dict[str, float | None]:
    """Get current battery status as a resource."""
    b = vehicle.battery
    return {"voltage": b.voltage, "current": b.current, "level": b.level}

@mcp.tool()
async def location_global() -> dict[str, float]:
    """Get location in a global frame relative to MSL."""
    loc = vehicle.location.global_frame
    return {"lat": loc.lat, "lon": loc.lon, "alt": loc.alt}

@mcp.tool()
async def location_global_relative() -> dict[str, float]:
    """Get location in a global frame relative to home position."""
    loc = vehicle.location.global_relative_frame
    return {"lat": loc.lat, "lon": loc.lon, "alt": loc.alt}

@mcp.tool()
async def home_location() -> dict[str, float]:
    """Get current home position."""
    home = vehicle.home_location
    return {"lat": home.lat, "lon": home.lon, "alt": home.alt}

@mcp.tool()
async def mode_info() -> str:
    """Get current vehicle mode."""
    return vehicle.mode.name

@mcp.tool()
async def get_location_local() -> dict[str, float]:
    """Create a LocationLocal object and return its north, east, down components."""
    loc = vehicle.location.local_frame
    return {"north": loc.north, "east": loc.east, "down": loc.down}

@mcp.tool()
async def distance_home() -> float:
    """Get distance from vehicle to home in meters (3D if down available, otherwise 2D)."""
    return vehicle.distance_home()

@mcp.tool()
async def list_parameters() -> dict[str, Any]:
    """List all vehicle parameters with their names and values."""
    params = {}
    for name, value in vehicle.parameters.items():
        params[name] = value
    return params

@mcp.tool()
async def get_rangefinder() -> dict[str, float]:
    """Get rangefinder distance and voltage."""
    rf = vehicle.rangefinder
    return {"distance": rf.distance, "voltage": rf.voltage}

# Gimbal API tools
@mcp.tool()
async def get_gimbal_attitude() -> dict[str, float]:
    """Get current gimbal attitude: pitch, roll, yaw (degrees or radians)."""
    g = vehicle.gimbal
    return {"pitch": g.pitch, "roll": g.roll, "yaw": g.yaw}

@mcp.tool()
async def set_gimbal_orientation(pitch: float, roll: float, yaw: float) -> str:
    """Set gimbal orientation."""
    vehicle.gimbal.rotate(pitch, roll, yaw)
    return "GIMBAL_ORIENTED"

@mcp.tool()
async def target_location(lat: float, lon: float, alt: float) -> str:
    """Point the gimbal to the specified region of interest."""
    # Use LocationGlobal for ROI
    loc = LocationGlobal(lat, lon, alt)
    vehicle.gimbal.target_location(loc)
    return "GIMBAL_TARGET_SET"

@mcp.tool()
async def release_gimbal() -> str:
    """Release control of the gimbal to RC/manual control."""
    vehicle.gimbal.release()
    return "GIMBAL_RELEASED"

# Mission Command tools
@mcp.tool()
async def get_command_sequence() -> list[dict]:
    """Get the current mission command sequence."""
    # Download and return all commands in the mission
    vehicle.commands.download()
    vehicle.commands.wait_ready()

    commands = []
    for cmd in vehicle.commands:
        commands.append({
            "target_system": cmd.target_system,
            "target_component": cmd.target_component,
            "seq": cmd.seq,
            "frame": cmd.frame,
            "command": cmd.command,
            "current": cmd.current,
            "auto_continue": cmd.autocontinue,
            "param1": cmd.param1,
            "param2": cmd.param2,
            "param3": cmd.param3,
            "param4": cmd.param4,
            "x": cmd.x,
            "y": cmd.y,
            "z": cmd.z
        })
    return commands

@mcp.tool()
async def create_command(cmds: list[dict]) -> str:
    """Add mission commands from a list of dicts with 'type', 'lat', 'lon', 'alt'."""
    # Download existing commands
    vehicle.commands.download()
    vehicle.commands.wait_ready()

    # Iterate and create commands
    for idx, c in enumerate(cmds):
        cmd_type = c.get('type')
        try:
            lat = float(c.get('lat', 0))
            lon = float(c.get('lon', 0))
            alt = float(c.get('alt', 0))
        except (TypeError, ValueError):
            return f"error: invalid coordinates in command {idx}"
        if cmd_type == 'takeoff':
            command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        elif cmd_type == 'waypoint':
            command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        else:
            return f"error: unknown command type '{cmd_type}' at index {idx}"
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        # Build command with default params
        cmd = Command(
            0, 0, idx,
            frame, command,
            0, 0,
            0, 0, 0, 0,
            lat, lon, alt
        )
        vehicle.commands.add(cmd)
    # Upload a new mission
    vehicle.commands.upload()
    return "MISSION_COMMANDS_UPLOADED"

# 5. Run server through SSE transport
if __name__ == "__main__":
    mcp.run(transport='sse')
    # mcp.run()
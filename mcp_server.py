import _collections_abc
import collections
from typing import Any

collections.MutableMapping = _collections_abc.MutableMapping
import time, os, math
from dotenv import load_dotenv
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, LocationGlobal, Command
from mcp.server.fastmcp import FastMCP
from pymavlink import mavutil
import json

# 1. Load configuration from .env
load_dotenv()
DRONE_CONN = os.getenv("DRONE_CONN")
if not DRONE_CONN:
    raise RuntimeError("Please set DRONE_CONN env var to your autopilot endpoint, e.g. udp:13.213.147.174:5762")

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

# 3. Register arm/takeoff/goto/land tools
@mcp.tool()
async def arm() -> str:
    """Arm the drone."""
    if not vehicle.is_armable:
        return "Vehicle not armable"
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(1)
    return "ARMED" if vehicle.armed else "Arm timeout"

@mcp.tool()
async def takeoff(altitude: float) -> dict[str, float | Any]:
    """Execute a take-off operation to the given altitude (meters)."""
    print("Basic pre-arm checks")

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
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
    """Fly to specified GPS coordinates or change the current altitude."""
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
        time.sleep(10)

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

# Register home and location tools
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
async def get_location_local() -> dict[str, float]:
    """Create a LocationLocal object and return its north, east, down components."""
    loc = vehicle.location.local_frame
    return {"north": loc.north, "east": loc.east, "down": loc.down}

@mcp.tool()
async def distance_home() -> float:
    """Get distance from vehicle to home in meters (3D if down available, otherwise 2D)."""
    return vehicle.distance_home()

@mcp.tool()
async def switch_mode(mode: str) -> str:
    """Switch vehicle mode."""
    vehicle.mode = VehicleMode(mode)
    return f"MODE_CHANGED_TO_{mode}"

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
        "version": vehicle.version,
        "mode": vehicle.mode.name,
        "armed": vehicle.armed,
        "system_status": vehicle.system_status,
    }

#Register vehicle info tools
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
async def get_rangefinder() -> dict[str, float]:
    """Get rangefinder distance and voltage."""
    rf = vehicle.rangefinder
    return {"distance": rf.distance, "voltage": rf.voltage}

#Register parameter tools
@mcp.tool()
async def get_parameter(name: str) -> Any:
    """Get a specific vehicle parameter by name."""
    params = vehicle.parameters
    if name not in params:
        return f"error: parameter '{name}' not found"
    return params[name]

@mcp.tool()
async def set_parameter(name: str, value: Any) -> Any:
    """Set a specific vehicle parameter by name."""
    params = vehicle.parameters
    if name not in params:
        return f"error: parameter '{name}' not found"
    try:
        vehicle.parameters[name] = value
        return vehicle.parameters[name]
    except Exception as e:
        return f"error: {e}"

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

@mcp.tool()
async def get_gps_info() -> dict[str, Any]:
    """Get GPS info: eph, epv, fix_type, satellites_visible."""
    gps_info = vehicle.gps_0
    if gps_info is None:
        return {"error": "GPS info not available"}
    return {
        "eph": gps_info.eph,
        "epv": gps_info.epv,
        "fix_type": gps_info.fix_type,
        "satellites_visible": gps_info.satellites_visible
    }

# Register mission command tools
@mcp.tool()
async def get_command_sequence() -> list[dict]:
    """Get the current mission command sequence."""
    # Download and return all commands in the mission
    vehicle.commands.download()
    vehicle.commands.wait_ready()
    vehicle.commands._use_mission_int = True

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
    """Create one or multiple mission commands."""
    # Download the current mission
    vehicle.commands.download()
    vehicle.commands.wait_ready()
    vehicle.commands._use_mission_int = True

    for idx, c in enumerate(cmds):
        # Support both 'type' and 'command'
        raw_cmd = c.get('type') or c.get('command')
        cmd_key = str(raw_cmd).lower() if raw_cmd is not None else ''

        # Parse coordinate: lat/long/alt or param5/6/7
        try:
            lat = float(c.get('lat',
                      c.get('latitude',
                        c.get('param5', 0))))
            lon = float(c.get('lon',
                      c.get('longitude',
                        c.get('param6', 0))))
            alt = float(c.get('alt',
                      c.get('altitude',
                        c.get('param7', 0))))
        except (TypeError, ValueError):
            return f"error: invalid coordinates in command {idx}"

        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        # Build MAV command
        if 'takeoff' in cmd_key or cmd_key == str(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF):
            mav_cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
            # x,y = 0 for takeoff
            cmd = Command(
                0, 0, idx,
                frame, mav_cmd,
                0, 1,
                0, 0, 0, 0,
                0, 0, alt
            )
        elif 'waypoint' in cmd_key or cmd_key == str(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT):
            mav_cmd = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            cmd = Command(
                0, 0, idx,
                frame, mav_cmd,
                0, 1,
                0, 0, 0, 0,
                lat, lon, alt
            )
        elif 'return to launch' or 'RTL' in cmd_key or cmd_key == str(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH):
            mav_cmd = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
            cmd = Command(
                0, 0, idx,
                frame, mav_cmd,
            )
        else:
            return f"error: unknown command type '{raw_cmd}' at index {idx}"

        vehicle.commands.add(cmd)

    # Upload the mission to autopilot
    vehicle.commands.upload()
    return "MISSION_COMMANDS_UPLOADED"

@mcp.tool()
async def clear_all_mission() -> str:
    """Clear all missions commands."""
    cmd = vehicle.commands
    cmd.clear()
    cmd.upload()
    return "MISSION_COMMANDS_CLEARED"

@mcp.tool()
async def readmission(file_name) -> list[Command]:
    mission_list = []
     # Load JSON and get top-level items
    with open(file_name, 'r') as f:
        data = json.load(f)
    if data.get('fileType') != 'Plan':
        raise Exception(f"Unsupported fileType: {data.get('fileType')}")

    items = data.get('mission', {}).get('items', [])
    seq = 0

    for item in items:
    # Determine if this is SimpleItem or ComplexItem
        if item.get('type') == 'SimpleItem':
            entry_list = [item]
        elif item.get('type') == 'ComplexItem':
        # find a nested dict that contains the real 'Items' list
            nested = None
            for v in item.values():
                if isinstance(v, dict) and 'Items' in v:
                    nested = v
                    break
            if nested is None:
                continue
            entry_list = nested['Items']
        else:
        # skip unsupported item types
            continue

        # Flatten each subentry into a Command

        for sub in entry_list:
            frame = sub.get('frame')
            mav_command = sub.get('command')
            params = sub.get('params', [])
            # normalize params → list of 7 floats
            if not isinstance(params, list) or len(params) != 7:
                raise Exception(f"Invalid params length at seq {seq}: {len(params)}")

            try:
                p = [float(x) if x is not None else 0.0 for x in params]
            except Exception as e:
                raise Exception(f"Param conversion error at seq {seq}: {e}")

            current = 0
            autocontinue = 1 if sub.get('autoContinue', False) else 0
            # build Command (target_system=0, target_component=0, seq=seq, frame, command, current, autocontinue, p1…p7)
            cmd = Command(
                0, 0, seq,
                sub['frame'], sub['command'],
                0, 1,
                p[0], p[1], p[2], p[3],
                p[4], p[5], p[6]
            )
            mission_list.append(cmd)
            seq += 1

    return mission_list

@mcp.tool()
async def upload_mission(file_name) -> str:
        """
        Upload a mission from a file.
        """
        #Read a mission from a file
        mission_list = await readmission(file_name)
        vehicle.commands.download()
        vehicle.commands.wait_ready()

        print ("\nUpload mission from a file: %s" % file_name)
        #Clear all existing missions from a vehicle
        print ('Clear mission')
        cmds = vehicle.commands
        cmds.clear()
        #Add a new mission to the vehicle
        for command in mission_list:
            cmds.add(command)
        print(' Upload mission')
        vehicle.commands.upload()
        return "MISSION_UPLOADED"

@mcp.tool()
async def waypoints_info() -> str:
    """Return the number of waypoints."""
    count = vehicle.commands.count
    active = vehicle.commands.next
    return "Number of waypoints: %d \nActive waypoint: %d" % (count, active)

# 5. Run server through SSE transport
if __name__ == "__main__":
    # mcp.run(transport='sse')
    mcp.run()
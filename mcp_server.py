import _collections_abc
import collections
from typing import Any, List, Dict, Union, Tuple, Optional

collections.MutableMapping = _collections_abc.MutableMapping
import time, os, math, json, re, requests
from dotenv import load_dotenv
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, LocationGlobal, Command
from shapely.geometry import Polygon, LineString
from mcp.server.fastmcp import FastMCP
from pymavlink import mavutil

# 1. Load configuration from .env
load_dotenv()
_DRONE_CONN = os.getenv("DRONE_CONN")
if not _DRONE_CONN:
    raise RuntimeError("Please set DRONE_CONN env var to your autopilot endpoint, e.g. tcp:13.213.147.174:5762")

# Initialize Google Maps client for geocoding
_gmaps_api_key = os.getenv('GOOGLE_MAPS_API_KEY')
if not _gmaps_api_key:
    raise RuntimeError("Environment variable GOOGLE_MAPS_API_KEY is not set for geocoding")

vehicle = connect(_DRONE_CONN, wait_ready=True)
print(f"Connected. Mode: {vehicle.mode.name}")

# 2. Initial MCP server
mcp = FastMCP(
    name="drone-mcp",
)

def haversine(lat1, lon1, lat2, lon2):
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = (math.sin(d_lat/2)**2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(d_lon/2)**2)
    return 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

@mcp.tool()
def geocode_address(address: str) -> Tuple[float, float]:
    """ Convert an address/place string into (lat, lon) using Google Maps Geocoding API."""
    params = {
        'address': address,
        'key': _gmaps_api_key,
        'components': 'country:VN',  # bias results to Vietnam
        'region': 'vn'
    }
    response = requests.get('https://maps.googleapis.com/maps/api/geocode/json', params=params, timeout=10)
    response.raise_for_status()
    data = response.json()
    results = data.get('results')
    if not results:
        raise ValueError(f"No geocoding results for address: '{address}'")
    location = results[0]['geometry']['location']
    return float(location['lat']), float(location['lng'])

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
    # (otherwise the command after Vehicle.simple_takeoff will execute immediately).
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
async def goto(lat_or_address: Union[str, float],
               lon: Optional[float] = None,
               alt: Optional[float] = None
               ) -> Dict[str, Union[str, float, Any]]:
    """Fly to specified GPS coordinates or change the current altitude.
    If only an address string is provided, geocode it first to get coordinates."""

    # Handle address-only input
    if isinstance(lat_or_address, str) and lon is None and alt is None:
        # Geocode the address to (latitude, longitude)
        try:
            latitude, longitude = geocode_address(lat_or_address)
        except Exception as e:
            return {"error": f"Geocoding failed: {e}"}
        # Use current relative altitude if no altitude was provided
        altitude = vehicle.location.global_relative_frame.alt
    else:
        # Standard case: latitude, longitude, and altitude are given
        latitude = float(lat_or_address)
        longitude = float(lon)
        altitude = float(alt)

    # Ensure GUIDED mode
    vehicle.airspeed = 10
    vehicle.mode = VehicleMode("GUIDED")
    deadline = time.time() + 10

    while vehicle.mode.name != "GUIDED" and time.time() < deadline:
        time.sleep(1)
    if vehicle.mode.name != "GUIDED":
        return {"error": "Failed to set GUIDED mode"}

    target = LocationGlobalRelative(latitude, longitude, altitude)
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
    """Get the current home position."""
    home = vehicle.home_location
    return {"lat": home.lat, "lon": home.lon, "alt": home.alt}

@mcp.tool()
async def get_location_local() -> dict[str, float]:
    """Get the LocationLocal object and return its north, east, down components."""
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

# Register vehicle info tools
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

# Register parameter tools
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

# Register mission planner tools
@mcp.tool()
async def get_command_sequence() -> list[dict]:
    """Get the current mission plan sequence."""
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
async def create_mission(cmds: List[Dict[str, Any]]) -> str:
    """Create a mission plan from user prompt or after read a mission file."""
    # Download the current mission
    vehicle.commands.download()
    vehicle.commands.wait_ready()

    for idx, c in enumerate(cmds):
        # Support both 'type' and 'command'
        raw = c.get('type') or c.get('command')
        key = str(raw).lower() if raw is not None else ''

        # Determine coordinates or address
        if 'address' in c and isinstance(c['address'], str):
            lat, lon = geocode_address(c['address'])
            alt = float(c.get('alt', c.get('altitude', 0)))
        else:
            try:
                lat = float(c.get('lat', c.get('latitude', c.get('param5', 0))))
                lon = float(c.get('lon', c.get('longitude', c.get('param6', 0))))
                alt = float(c.get('alt', c.get('altitude', c.get('param7', 0))))
            except (TypeError, ValueError):
                return f"error: invalid coordinates in command {idx}"

        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        # Build MAV command
        if 'takeoff' in key or key == str(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF):
            mav_cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
            # x,y = 0 for takeoff
            cmd = Command(
                0, 0, idx,
                frame, mav_cmd,
                0, 1,
                0, 0, 0, 0,
                0, 0, alt
            )
        elif 'waypoint' in key or key == str(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT):
            mav_cmd = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            cmd = Command(
                0, 0, idx,
                frame, mav_cmd,
                0, 1,
                0, 0, 0, 0,
                lat, lon, alt
            )
        elif 'return to launch' or 'RTL' in key or key == str(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH):
            mav_cmd = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
            cmd = Command(
                0, 0, idx,
                frame, mav_cmd,
            )
        else:
            return f"error: unknown command type '{raw}' at index {idx}"

        vehicle.commands.add(cmd)

    # Upload the mission to autopilot
    vehicle.commands.upload()
    # vehicle.mode = VehicleMode("AUTO")
    return "MISSION_COMMANDS_UPLOADED"

@mcp.tool()
async def clear_all_mission() -> str:
    """Clear all mission plans."""
    cmd = vehicle.commands
    cmd.clear()
    cmd.upload()
    return "MISSION_COMMANDS_CLEARED"

@mcp.tool()
async def readmission(file_name) -> list[Command]:
    """Read a mission plan file with input is file's pathname."""
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
        """Upload a mission from a file."""
        # Read a mission from a file
        mission_list = await readmission(file_name)
        vehicle.commands.download()
        vehicle.commands.wait_ready()

        print ("\nUpload mission from a file: %s" % file_name)
        # Clear all existing missions from a vehicle
        print ('Clear mission')
        cmds = vehicle.commands
        cmds.clear()
        # Add a new mission to the vehicle
        for command in mission_list:
            cmds.add(command)
        print('Upload mission')
        vehicle.commands.upload()
        return "MISSION_UPLOADED"

@mcp.tool()
async def waypoints_info() -> str:
    """Return the number of waypoints."""
    count = vehicle.commands.count
    active = vehicle.commands.next
    return "Number of waypoints: %d \nActive waypoint: %d" % (count, active)

CAMERA_PARAMS = {
     "sony_a7riv": {
         "sensor_width_mm": 23.5,
         "sensor_height_mm": 15.6,
         "focal_length_mm": 16.0
     }
 }

def normalize_name(name: str) -> str:
    """Lowercase and remove non-alphanumeric for fuzzy matching."""
    return re.sub(r'[^a-z0-9]', '', name.lower())

def parse_prompt(prompt: str) -> Dict[str, Any]:
    """Extracts GPS points, altitude, overlap, and camera from the user prompt."""
    points = re.findall(r"(-?\d+\.\d+)[, ]+(-?\d+\.\d+)", prompt)
    if len(points) < 4:
        raise ValueError("Please provide 4 GPS points as lat,long pairs in the prompt.")
    coords = [(float(lat), float(lon)) for lat, lon in points[:4]]

    alt_match = re.search(r"(\d+)(?:m| mét)", prompt)
    altitude = float(alt_match.group(1)) if alt_match else 50.0

    overlap_match = re.search(r"(\d+)%", prompt)
    overlap = float(overlap_match.group(1)) / 100.0 if overlap_match else 0.8

    camera_match = re.search(r"mapping using is ([\w\s0-9-]+)", prompt, re.IGNORECASE)
    camera_raw = camera_match.group(1).strip() if camera_match else 'sony a7riv'
    # Normalize and match against keys
    norm_input = normalize_name(camera_raw)
    matched_key = None

    # Address mode
    addr_m = re.search(r"(?:địa chỉ|address)\s+(.+?)(?:,|$)", prompt, re.IGNORECASE)
    if addr_m:
        address = addr_m.group(1).strip()
        lat, lon = geocode_address(address)
        return {'mode': 'point', 'coords': (lat, lon), 'altitude': altitude}
    for key in CAMERA_PARAMS:
        if normalize_name(key) == norm_input:
            matched_key = key
            break
    if not matched_key:
        raise ValueError(f"Camera specs for '{camera_raw}' not found.")
    camera = matched_key

    return {'points': coords, 'altitude': altitude, 'overlap': overlap, 'camera': camera}

def calculate_spacing(camera: str, altitude: float, overlap: float) -> float:
    """Calculates line spacing based on camera specs, altitude, and overlap."""
    specs = CAMERA_PARAMS[camera]
    swath = specs['sensor_width_mm'] / specs['focal_length_mm'] * altitude
    spacing_m = swath * (1 - overlap)
    # convert meters to degrees latitude (~111000 m per degree)
    return spacing_m / 111000.0

def generate_zigzag(points: List[tuple], spacing: float) -> List[tuple]:
    """Generates zigzag waypoints within the polygon defined by points."""
    poly = Polygon(points)
    minx, miny, maxx, maxy = poly.bounds
    lines: List[List[tuple]] = []
    y = miny
    idx = 0
    while y <= maxy:
        line = LineString([(minx, y), (maxx, y)])
        inter = line.intersection(poly)
        if not inter.is_empty:
            coords = list(inter.coords)
            if idx % 2:
                coords.reverse()
            lines.append(coords)
            idx += 1
        y += spacing
    return [pt for segment in lines for pt in segment]

@mcp.tool()
async def mission_survey(prompt: str) -> str:
    """
    Generate and upload a survey mission:
    - Prompt must include GPS points, altitude (m), overlap (%), and camera model.
    - Connects via MAVLink, clears an existing mission, uploads takeoff and zigzag waypoints.
    """
    # Parse user inputs
    parsed = parse_prompt(prompt)
    spacing = calculate_spacing(parsed['camera'], parsed['altitude'], parsed['overlap'])
    waypoints = generate_zigzag(parsed['points'], spacing)

    # Connect to vehicle
    cmds = vehicle.commands
    cmds.clear()

    # Add a takeoff command
    lat0, lon0 = parsed['points'][0]
    cmds.add(Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0,
        lat0, lon0, parsed['altitude']
    ))

    # Add survey waypoints
    for lat, lon in waypoints:
        cmds.add(Command(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0, 0, 0,
            lat, lon, parsed['altitude']
        ))

    # Add Return to Launch if requested
    if re.search(r"return to launch|rtl", prompt, re.IGNORECASE):
        cmds.add(Command(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0,
            0, 0, 0
        ))

    # Upload a mission
    cmds.upload()
    return f"MISSION_UPLOADED with {len(waypoints)} waypoints"

async def save_plan(file_path: str) -> str:
    """Save the current mission to a .plan file in QGroundControl WPL format."""
    while not vehicle.is_armable:
        time.sleep(1)
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    mission_list = list(cmds)

    # Build WPL file
    output = 'QGA WPL 110\n'
    home = vehicle.home_location
    output += f"0\t1\t0\t16\t0\t0\t0\t0\t{home.lat}\t{home.lon}\t{home.alt}\t1\n"
    for cmd in mission_list:
        output += (
            f"{cmd.seq}\t{cmd.current}\t{cmd.frame}\t{cmd.command}\t"
            f"{cmd.param1}\t{cmd.param2}\t{cmd.param3}\t{cmd.param4}\t"
            f"{cmd.x}\t{cmd.y}\t{cmd.z}\t{cmd.autocontinue}\n"
        )

    with open(file_path, 'w') as f:
        f.write(output)

    return f"PLAN_SAVED to {file_path}"

@mcp.tool()
async def create_plan_from_prompt(prompt: str, file_path: str) -> dict[str, str] | str | Any:
    """
    Parse a natural-language prompt to build a mission plan and save it to a .plan file.

    Supported syntax:
      - 'takeoff at <address> at altitude Xm'
      - 'waypoint [n] at <address> at altitude Xm'
      - 'return to launch'
    Clauses separated by commas.
    """
    clauses = [cl.strip() for cl in re.split(r',', prompt) if cl.strip()]
    cmds = []

    # Parse each clause into a mission command
    for cl in clauses:
        # takeoff clause
        m_to = re.match(
            r'(?:takeoff)\s+at\s+(.+?)\s+at\s+altitude\s+(\d+\.?\d*)m',
            cl, re.IGNORECASE
        )
        if m_to:
            addr, alt = m_to.group(1), float(m_to.group(2))
            cmds.append({'type': 'takeoff', 'address': addr, 'alt': alt})
            continue

        # waypoint clause
        m_wp = re.match(
            r'(?:waypoint(?:\s*\d+)?)\s+at\s+(.+?)\s+at\s+altitude\s+(\d+\.?\d*)m',
            cl, re.IGNORECASE
        )
        if m_wp:
            addr, alt = m_wp.group(1), float(m_wp.group(2))
            cmds.append({'type': 'waypoint', 'address': addr, 'alt': alt})
            continue

        # return-to-launch clause
        if re.search(r'return to launch|rtl', cl, re.IGNORECASE):
            cmds.append({'type': 'rtl'})

    # Bail out if nothing was recognized
    if not cmds:
        return "error: no valid mission steps found in prompt"

    # Build the mission in memory
    await create_mission(cmds)

    # Determine Desktop directory as writable base
    desktop_dir = os.path.join(os.path.expanduser("~"), "Desktop")
    os.makedirs(desktop_dir, exist_ok=True)

    # If the user provided a relative path, prepend the writable base
    if not os.path.isabs(file_path):
        full_path = os.path.join(desktop_dir, file_path)
    else:
        full_path = file_path

    # Attempt to save the plan file, catching any filesystem errors
    try:
        result = await save_plan(full_path)
    except OSError as e:
        return {"error": f"Failed to save plan to {full_path}: {e.strerror}"}

    # Return the save result (e.g., the path or success message)
    return result

# 5. Run server through SSE transport
if __name__ == "__main__":
    # mcp.run(transport='sse')
    mcp.run()
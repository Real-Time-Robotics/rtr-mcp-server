from mcp import ClientSession
from mcp.client.sse import sse_client

async def check():
    async with sse_client("http://0.0.0.0:8000/sse") as streams:
        async with ClientSession(*streams) as session:
            await session.initialize()

            # List avail tool
            # tools = await session.list_tools()
            # print(tools)

            # Call set_home tool
            # result = await session.call_tool("land")
            # print(result)

            # Call arm tool
            # result = await session.call_tool("arm", arguments={"command": "arm"})
            # print(result)

            # Call set_home tool
            result = await session.call_tool("waypoints_info", arguments={"command": "waypoint info"})
            print(result)

            # Call takeoff tool
            # result = await session.call_tool("readmission", arguments={"file_name": "/Users/maichieuthuy/Desktop/z.plan"})
            # print(result)

            # Call takeoff tool
            # result = await session.call_tool("upload_mission", arguments={"file_name": "/Users/maichieuthuy/Desktop/z.plan"})
            # print(result)

            # Call goto tool
            # result = await session.call_tool("goto", arguments={"latitude": 10.841626, "longitude": 106.773167, "altitude": 30.0})
            # print(result)

            # Call landing tool
            # result = await session.call_tool("land", arguments={"command": "land"})
            # print(result)

            # Call status tool
            # result = await session.call_tool("switch_mode", arguments={"mode": "GUIDED"})
            # print(result)

            # Call status tool
            # result = await session.call_tool("status")
            # print(result)

            # Get a vehicle's mission command sequence
            # mission_cmds = [
            #     {"type": "takeoff", "lat": 0.0, "lon": 0.0, "alt": 10.0},
            #     {"type": "waypoint", "lat": 10.8418328, "lon": 106.775515, "alt": 30.0},
            # ]
            #
            # # Gọi tool add_mission_commands
            # result = await session.call_tool(
            #     "create_command",
            #     arguments={"cmds": mission_cmds}
            # )
            # print("add_mission_commands response:", result)

            # Call status tool
            # result = await session.call_tool("list_parameters")
            # print(result)

            # Call get parameter tool
            # result = await session.call_tool("clear_all_mission")
            # print(result)

            # Call get parameter tool
            # result = await session.call_tool(
            #     "create_command",arguments={"cmds": [{"type": "waypoint", "lat": 10.8418328, "lon": 106.775515, "alt": 30.0}]})
            # print(result)
            #
            # # Lấy lại sequence để verify
            # sequence = await session.call_tool("get_command_sequence")
            # print("\nMission command sequence:")
            # for cmd in sequence:
            #     print(cmd)

if __name__ == "__main__":
    import asyncio
    asyncio.run(check())

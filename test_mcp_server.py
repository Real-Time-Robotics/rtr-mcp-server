from mcp import ClientSession
from mcp.client.sse import sse_client

async def check():
    async with sse_client("http://0.0.0.0:8000/sse") as streams:
        async with ClientSession(*streams) as session:
            await session.initialize()

            # List avail tool
            tools = await session.list_tools()
            print(tools)

            # Call arm tool
            result = await session.call_tool("arm", arguments={"command": "arm"})
            print(result)

            # Call set_home tool
            result = await session.call_tool("set_home", arguments={"command": "set home"})
            print(result)

            # Call takeoff tool
            result = await session.call_tool("takeoff", arguments={"altitude": 3.0})
            print(result)

            # Call goto tool
            result = await session.call_tool("goto", arguments={"latitude": 10.842207, "longitude": 106.775404, "altitude": 4.0})
            print(result)

            # Call landing tool
            result = await session.call_tool("land", arguments={"command": "land"})
            print(result)

            #Call status tool
            result = await session.call_tool("status")
            print(result)

if __name__ == "__main__":
    import asyncio
    asyncio.run(check())

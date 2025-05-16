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

            # Call takeoff tool
            # result = await session.call_tool("takeoff", arguments={"altitude": 10})
            # print(result)

            #Call status tool
            result = await session.call_tool("status")
            print(result)

            # # Get tax code
            # result = await session.read_resource("resource://ma_so_thue")
            # print("Tax code = {}".format(result))

            # # Say hi
            # result = await session.read_resource("resource://say_hi/Thuy")
            # print("Say hi = {}".format(result))

            # prompt = await session.get_prompt("review_sentence", arguments={"sentence":"So chung minh nhan dan la 123456789"})
            # print("Prompt = {}".format(prompt))


if __name__ == "__main__":
    import asyncio
    asyncio.run(check())

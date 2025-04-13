import asyncio
import websockets

async def handle_connection(websocket, path):
    print("客户端已连接")
    try:
        async for message in websocket:
            print(f"收到指令：{message}")
            response = f"收到：{message}"
            await websocket.send(response)
    except websockets.exceptions.ConnectionClosed:
        print("客户端断开连接")

start_server = websockets.serve(handle_connection, "0.0.0.0", 8765)

print("WebSocket 服务器启动中（端口 8765）...")
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()

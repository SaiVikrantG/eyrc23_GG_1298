import asyncio
import websockets

async def connect():
    uri = "ws://192.168.0.112/ws"
    async with websockets.connect(uri) as websocket:
        print("Connected to ESP32 WebSocket")
        await websocket.send("Hello ESP32!")
        while True:
            message = await websocket.recv()
            print(f"Received message from ESP32: {message}")

asyncio.get_event_loop().run_until_complete(connect())

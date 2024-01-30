import asyncio
import websockets

async def receive_data(websocket):
    async for message in websocket:
        print("Received data from ESP32:", message)
        await send_data(websocket)

async def send_data(websocket):
    async for message in websocket:
        data_to_send = "sent data"
        await websocket.send(data_to_send)
        await receive_data(websocket)

async def main():
    uri = "ws://192.168.0.107/ws"
    async with websockets.connect(uri) as websocket:
        await receive_data(websocket)

if __name__ == "__main__":
    asyncio.run(main())
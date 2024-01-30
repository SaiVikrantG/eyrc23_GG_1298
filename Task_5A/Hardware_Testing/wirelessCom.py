
import asyncio
import websockets
import time

async def receive_data(websocket):
    async for message in websocket:
        print("Received data from ESP32:", message)
        await send_data(websocket)
        # await asyncio.sleep(5)
        # return message

async def send_data(websocket):
    async for message in websocket:
        data_to_send = "sent data"
        
        await websocket.send(data_to_send)
        # print("sent")
        await receive_data(websocket)
        # await asyncio.sleep(5)
    # while True:
        # if m2 is not None:
    
    # data_to_send = "sent data"
    # # if data_to_send.lower() == 'exit':
    # #     break
    # websocket.send(data_to_send)
    # time.sleep(1)

async def main():
    uri = "ws://192.168.0.107/ws"
    async with websockets.connect(uri) as websocket:
        # await send_data(websocket)
        await receive_data(websocket)
        

        # Start tasks for receiving and sending data concurrently
        # m1 = receive_data(websocket)
        # tasks = [send_data(websocket)]
        # if m1 is not None:
        #      send_data(websocket)
        # Wait for both tasks to complete (e.g., user decides to exit)
        # await asyncio.gather(*tasks)
        # asyncio.run(send_data(websocket))

if __name__ == "__main__":
    asyncio.run(main())




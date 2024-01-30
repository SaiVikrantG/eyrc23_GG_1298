import asyncio
import websockets
import time

# WebSocket server URL
ws_url = "ws://10.25.16.110/ws"

# Function to handle incoming messages
async def message_handler(websocket):
    async for message in websocket:
        print("Received data:", message)
        await process_message(message, websocket)

# Function to process individual messages
async def process_message(message, websocket):
    # Process the message and send a response
    time.sleep(3)
    # response = f"Response to {message}"
    # await websocket.send(response)
    # print("Sent response:", response)

# Function to send series of data
async def send_data(websocket):
    data_to_send = ["data1", "data2", "data3"]  # List of data values to send
    for data in data_to_send:
        await websocket.send(data)
        print("Data sent to server:", data)

# Main function
async def main():
    async with websockets.connect(ws_url) as websocket:
        # Start a task to handle incoming messages
        message_task = asyncio.create_task(message_handler(websocket))

        # Send series of data
        await send_data(websocket)

        # Wait for the message handler to finish
        await message_task

# Run the main function
asyncio.run(main())

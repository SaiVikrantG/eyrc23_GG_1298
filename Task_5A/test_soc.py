import socket

# Define server IP address and port
SERVER_IP = '10.25.16.119'  # Listen on all interfaces
SERVER_PORT = 8080

# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address and port
server_socket.bind((SERVER_IP, SERVER_PORT))

# Start listening for incoming connections
server_socket.listen(5)
print(f"Server listening on {SERVER_IP}:{SERVER_PORT}")

while True:
    # Accept incoming connection
    client_socket, client_address = server_socket.accept()
    print(f"Connection from {client_address}")

    # Receive data from client
    data = client_socket.recv(1024).decode('utf-8')
    print(f"Received data: {data}")

    # Process received data (if needed)
    # For simplicity, we'll just echo it back
    response = "Server received: " + data

    # Send response back to client
    client_socket.sendall(response.encode('utf-8'))

    # Close the connection with the client
    client_socket.close()

import socket
import tkinter as tk
from tkinter import Scale
import json

def send_values():
    # Get the values from sliders and send them through the socket
    p_value = p_slider.get()
    i_value = i_slider.get()
    d_value = d_slider.get()
    message = {'P':int(p_value), 'I':int(i_value), 'D':int(d_value)}
    print(type(message))
    json_str = json.dumps(message)
    conn.sendall(json_str.encode())

# Create a tkinter window
root = tk.Tk()
root.title("PID Values Sender")

# Create three sliders for P, I, and D values
p_slider = Scale(root, label="P Value", from_=0, to=100, orient="horizontal")
p_slider.pack()
i_slider = Scale(root, label="I Value", from_=0, to=100, orient="horizontal")
i_slider.pack()
d_slider = Scale(root, label="D Value", from_=0, to=100, orient="horizontal")
d_slider.pack()

# Button to send values
send_button = tk.Button(root, text="Send Values", command=send_values)
send_button.pack()

# Socket communication code (to be implemented according to your needs)
ip = "192.168.0.101"  # Enter IP address
port = 8002

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((ip, port))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        root.mainloop()  # Start the tkinter main loop

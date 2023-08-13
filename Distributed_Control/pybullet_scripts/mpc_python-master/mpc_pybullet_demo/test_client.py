import socket
from random import randrange, uniform
import time

HEADER = 64
PORT = 8080
FORMAT = 'utf-8'
SERVER = "192.168.98.116"
ADDR = (SERVER, PORT)

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
err_code = client.connect(ADDR)
print(err_code)

def send(msg):
    message = msg.encode(FORMAT)
    msg_length = len(message)
    send_length = str(msg_length).encode(FORMAT)
    send_length += b''*(HEADER-len(send_length))
    client.send(send_length)
    client.send(message)
    print("Published message: ", msg)
    print(client.recv(2048).decode(FORMAT))

while True:
    randNumber = str(uniform(20.0, 23.0))
    send(randNumber)
    time.sleep(5)

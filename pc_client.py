# TCPclient.py

import socket

target_host = "192.168.0.122" #服务器端地址
target_port = 56050  #必须与服务器的端口号一致

while True:

    client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    client.connect((target_host,target_port))   

    data = input(">")

    if not data:
        break

    client.send(data.encode())

    response = client.recv(1024)

    print(response)

client.close()
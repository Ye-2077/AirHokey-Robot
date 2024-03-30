import socket
import threading

bind_ip = "0.0.0.0"
bind_port = 59630

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((bind_ip, bind_port))

print(f"[*] Listening on {bind_ip}:{bind_port}")

server.listen(5)

def handle_client(client_socket):
    # 发送两个数字
    numbers_to_add = "5,3"  # 例如发送5和3
    client_socket.send(numbers_to_add.encode())
    
    # 接收客户端发送回来的结果
    result = client_socket.recv(1024).decode()
    print(f"[*] Received sum: {result}")
    
    client_socket.close()

while True:
    client, addr = server.accept()
    print(f"[*] Accepted connection from {addr[0]}:{addr[1]}")
    
    client_handler = threading.Thread(target=handle_client, args=(client,))
    client_handler.start()

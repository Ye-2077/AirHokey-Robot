import socket
import threading

# 服务器配置
bind_ip = "0.0.0.0"
bind_port = 59630

def handle_client_connection(client_socket):
    # 发送目标值给客户端
    # 例如，发送目标角度为5.5 (你可以根据需求修改这个值)
    target_value = "0\n"
    client_socket.send(target_value.encode("utf-8"))
    print(f"Sent target value: {target_value} to client.")

    # 关闭客户端连接
    # client_socket.close()

def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((bind_ip, bind_port))
    server.listen(5)  # 最大连接数
    print(f"[*] Listening on {bind_ip}:{bind_port}")

    while True:
        client_sock, address = server.accept()
        print(f"[*] Accepted connection from {address[0]}:{address[1]}")
        client_handler = threading.Thread(
            target=handle_client_connection,
            args=(client_sock,)
        )
        client_handler.start()

if __name__ == "__main__":
    main()

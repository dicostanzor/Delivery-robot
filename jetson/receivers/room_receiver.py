import socket
import subprocess

HOST = '0.0.0.0'
PORT = 5005

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)

print("Room receiver listening...")

while True:
    conn, addr = server.accept()
    data = conn.recv(1024).decode().strip()
    print(f"Received: {data}")
    subprocess.run([
        'ros2', 'topic', 'pub', '--once',
        '/destination_request',
        'std_msgs/msg/String',
        f'{{"data": "{data}"}}'
    ])
    conn.close()

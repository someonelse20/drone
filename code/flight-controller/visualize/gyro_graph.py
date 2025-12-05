import socket

PORT = 5000

s = socket.socket()

s.connect(('drone', PORT))

print(s.recv(1024))

s.close()


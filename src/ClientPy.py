
import socket
import sys
import time

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ('192.168.0.193', 5020)

sock.connect(server_address)
# id = 0

# Esperar por la recepción de Datos -RTUData-
conn = sock.accept()

while 1:
  RTUdata = conn.recv(1024)
  if not data: break
  RTUData = clientConnected.recv(1024)
  conn.send("HMI".encode());

while 1:
    time.sleep(1)
    rList = [0xFF,2,0xaf,4,5]
    sock.send(bytes(rList))
    #time.sleep(1)
    #sock.send(bytes.fromhex('AA 11 FE'))
    #message = "SM13 Test"
    #sock.send(message.encode())
    break

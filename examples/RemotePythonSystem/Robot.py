import socket
import struct
import time
import numpy as np

IP = "127.0.0.1"
MY_PORT = 20001

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

#Initial condition
x = np.matrix("-5;5;0");

sock.bind((IP, MY_PORT))

print 'Ready!'

while True:
    
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    
    if data=='CLOSE':
        break

    numOfValues = len(data) / 8

    unp = struct.unpack('>' + 'd' * numOfValues, data)

    u = np.matrix(unp).T;

    print 'recived :'

    print u

    theta = x[2]

    R = np.array([[np.cos(theta),-np.sin(theta)],
                   [np.sin(theta), np.cos(theta)]])

    R = np.matrix(R)
    
    v = np.vstack((u[0],0))

    xDot = np.vstack((R*v,u[1]))

    x = x + 0.1*xDot
    
    t = tuple(x)

    data=struct.pack('>' + 'd' * len(x),*t)

    sock.sendto(data,addr)

	
sock.close()
print 'Socker closed'

import socket
import numpy as np
import cv2

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    try:
        data, addr = sock.recvfrom(65536) # buffer size is 65536 bytes
    except Exception as e:
        print(f"Failed to receive data: {e}")
        continue

    nparr = np.frombuffer(data, np.uint8)
    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if frame is None:
        print("Failed to decode image")
        continue

    cv2.imshow('frame',frame)
    print(f"Received frame from {addr}")
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

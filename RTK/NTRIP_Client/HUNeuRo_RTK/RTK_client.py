import socket
import serial
import base64

# NTRIP server details
caster_address = '212.156.70.42'
port = 2101
mount_point = 'DOGU_TG20_BROADCASTRTCM'  # replace with your stream's mount point
username = 'K0706071701'  # replace with your username if authentication is needed
password = 'maQjEp'  # replace with your password if authentication is needed

# Connect to NTRIP caster using raw TCP
s = socket.create_connection((caster_address, port))

# If your caster requires authentication, you might send an NTRIP request
# Otherwise, this step can be skipped
request = f'GET /{mount_point} HTTP/1.0\r\nUser-Agent: NTRIP PyClient/0.1\r\nAuthorization: Basic {base64.b64encode(f"{username}:{password}".encode()).decode()}\r\n\r\n'
s.send(request.encode())

# Setup serial port for your device
serial_port = '/dev/tty.usbserial-1420'  # replace with your device's serial port
baud_rate = 115200  # replace with your device's baud rate
ser = serial.Serial(serial_port, baud_rate, timeout=1)

try:
    while True:
        data = s.recv(1024)  # read up to 1024 bytes from the caster
        if not data:
            break
        ser.write(data)  # write correction data to the serial device
        print(str(data.hex()))
finally:
    s.close()
    ser.close()
    exit()

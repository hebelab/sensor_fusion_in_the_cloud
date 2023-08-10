import serial
import socket
import threading

# Parameters for the virtual serial port
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
DATA_BITS = 8
PARITY = 'N'
STOP_BITS = 1

# Parameters for the TCP server
TCP_IP = '0.0.0.0'
TCP_PORT = 12345
BUFFER_SIZE = 1024

# Create and configure the virtual serial port
ser = serial.Serial(
    port=SERIAL_PORT,
    baudrate=BAUD_RATE,
    bytesize=DATA_BITS,
    parity=PARITY,
    stopbits=STOP_BITS,
    timeout=1
)

# Function to handle data transfer from serial port to TCP
def serial_to_tcp(tcp_socket):
    while True:
        data = ser.read(BUFFER_SIZE)
        if data:
            print("serial2tcp: ", data)
            tcp_socket.send(data)

# Function to handle data transfer from TCP to serial port
def tcp_to_serial(tcp_socket):
    while True:
        data = tcp_socket.recv(BUFFER_SIZE)
        if data:
            print("tcp2serial: ", data)
            ser.write(data)

# Create the TCP server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)
print(f"Listening for incoming connections on {TCP_IP}:{TCP_PORT}")

conn, addr = s.accept()
print(f"Connection from: {addr}")

# Start the data transfer between the two interfaces
threading.Thread(target=serial_to_tcp, args=(conn,)).start()
threading.Thread(target=tcp_to_serial, args=(conn,)).start()

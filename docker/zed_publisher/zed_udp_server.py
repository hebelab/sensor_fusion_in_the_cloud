import cv2
import numpy as np
import pyzed.sl as sl
import socket

UDP_IP = "192.168.123.1"
# UDP_IP = "<Destination IP>"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

# Create a ZED camera object
zed = sl.Camera()

print(zed.get_camera_information().serial_number)
print(zed.get_device_list())

# Set configuration parameters
init_params = sl.InitParameters()
# init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
# init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use millimeter units (for depth measurements)

# Open the camera
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print(sl.toVerbose(sl.ERROR_CODE))
    print(f"Camera opening failed with error: {err}")
    exit(-1)

# Create and set runtime parameters
runtime_parameters = sl.RuntimeParameters()
runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD

# Prepare new image size to retrieve half-resolution images
image_size = sl.Resolution(640, 360)

# Create a ZED depth image (4 Channels) and a color image (4 Channels)
zed_depth = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
zed_image = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

# Capture 50 images and depth, then stop
i = 0
image_count = 50
while i < image_count:
    # A new image is available if grab() returns SUCCESS
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # Retrieve left image
        zed.retrieve_image(zed_image, sl.VIEW.LEFT, resolution=image_size)
        # Retrieve depth map. Depth is aligned on the left image
        zed.retrieve_measure(zed_depth, sl.MEASURE.DEPTH, resolution=image_size)
        
        frame = zed_depth.get_data()
        
        if frame is None:
            print("Frame data is None.")
        else:
            print(f"Frame shape: {frame.shape}")
        
        cv2.imwrite('test_image.jpg', frame)
        
        result, data = cv2.imencode('.jpg', frame)
        if not result:
            print("Failed to encode image")
            continue

        try:
            sock.sendto(data, (UDP_IP, UDP_PORT))
            print(f"Sent frame {i} to {UDP_IP}:{UDP_PORT}")
        except Exception as e:
            print(f"Failed to send frame: {e}")
        
        i += 1
    else:
        print("Frame grabbing failed.")

# Close the camera
zed.close()
